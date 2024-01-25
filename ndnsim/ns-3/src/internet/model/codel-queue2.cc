/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 Andrew McGregor
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Codel, the COntrolled DELay Queueing discipline
 * Based on ns2 simulation code presented by Kathie Nichols
 *
 * This port based on linux kernel code by
 * Authors:	Dave Täht <d@taht.net>
 *		Eric Dumazet <edumazet@google.com>
 *
 * Ported to ns-3 by: Andrew McGregor <andrewmcgr@gmail.com>
 */

#include "ns3/log.h"
#include "ns3/enum.h"
#include "ns3/uinteger.h"
#include "ns3/abort.h"
#include "codel-queue2.h" 
#include <assert.h>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE("CoDelQueue2");

/**
 * Performs a reciprocal divide, similar to the
 * Linux kernel reciprocal_divide function
 * \param A numerator
 * \param R reciprocal of the denominator B
 * \return the value of A/B
 */
/* borrowed from the linux kernel */

static inline uint64_t
ReciprocalDivide(uint64_t A, uint64_t R)
{
  return (uint64_t) (((uint64_t) A * R) >> 32);
}

/* end kernel borrowings */

/**
 * Returns the current time translated in CoDel time representation
 * \return the current time
 */
static uint64_t
CoDelGetTime(void)
{
  Time time = Simulator::Now();
  uint64_t ns = time.GetNanoSeconds();

  return ns >> CODEL2_SHIFT;
}

/**
 * CoDel time stamp, used to carry CoDel time informations.
 */
class CoDelTimestampTag : public Tag
{
public:
  CoDelTimestampTag();
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId
  GetTypeId(void);
  virtual TypeId
  GetInstanceTypeId(void) const;

  virtual uint32_t
  GetSerializedSize(void) const;
  virtual void
  Serialize(TagBuffer i) const;
  virtual void
  Deserialize(TagBuffer i);
  virtual void
  Print(std::ostream &os) const;

  /**
   * Gets the Tag creation time
   * @return the time object stored in the tag
   */
  Time
  GetTxTime(void) const;
private:
  uint64_t m_creationTime; //!< Tag creation time
};

NS_OBJECT_ENSURE_REGISTERED (CoDelQueue2);

//initiate Cwnd
double CoDelQueue2::Cwnd = 0.0;

TypeId
CoDelQueue2::GetTypeId(void)
{
  static TypeId tid =
      TypeId("ns3::CoDelQueue2").SetParent<Queue>().SetGroupName("Internet")
          .AddConstructor<CoDelQueue2>().AddAttribute("Mode",
          "Whether to use Bytes (see MaxBytes) or Packets (see MaxPackets) as the maximum queue size metric.",
          EnumValue(QUEUE_MODE_BYTES), MakeEnumAccessor(&CoDelQueue2::SetMode),
          MakeEnumChecker(QUEUE_MODE_BYTES, "QUEUE_MODE_BYTES", QUEUE_MODE_PACKETS,
              "QUEUE_MODE_PACKETS")).AddAttribute("MaxPackets",
          "The maximum number of packets accepted by this CoDelQueue.",
          UintegerValue(DEFAULT_CODEL_LIMIT),
          MakeUintegerAccessor(&CoDelQueue2::m_maxPackets),
          MakeUintegerChecker<uint32_t>()).AddAttribute("MaxBytes",
          "The maximum number of bytes accepted by this CoDelQueue.",
          UintegerValue(1500 * DEFAULT_CODEL_LIMIT),
          MakeUintegerAccessor(&CoDelQueue2::m_maxBytes), MakeUintegerChecker<uint32_t>())
          .AddAttribute("Interval", "The CoDel algorithm interval", StringValue("100ms"),
          MakeTimeAccessor(&CoDelQueue2::m_interval), MakeTimeChecker()).AddAttribute(
          "Target", "The CoDel algorithm target queue delay", StringValue("5ms"),
          MakeTimeAccessor(&CoDelQueue2::m_target), MakeTimeChecker()).AddTraceSource(
          "Count", "CoDel count", MakeTraceSourceAccessor(&CoDelQueue2::m_markedCount),
          "ns3::TracedValue::Uint32Callback").AddTraceSource("DropCount",
          "CoDel drop count", MakeTraceSourceAccessor(&CoDelQueue2::m_dropCount),
          "ns3::TracedValue::Uint32Callback").AddTraceSource("LastCount",
          "CoDel lastcount", MakeTraceSourceAccessor(&CoDelQueue2::m_lastCount),
          "ns3::TracedValue::Uint32Callback").AddTraceSource("BytesInQueue",
          "Number of bytes in the queue",
          MakeTraceSourceAccessor(&CoDelQueue2::m_bytesInQueue),
          "ns3::TracedValue::Uint32Callback").AddTraceSource("Sojourn",
          "Time in the queue", MakeTraceSourceAccessor(&CoDelQueue2::m_sojourn),
          "ns3::Time::TracedValueCallback").AddTraceSource("DropNext",
          "Time until next packet drop",
          MakeTraceSourceAccessor(&CoDelQueue2::m_nextMarkingTime),
          "ns3::TracedValue::Uint32Callback");

  return tid;
}

CoDelQueue2::CoDelQueue2()
    : Queue(),
        m_packets(),
        m_maxBytes(),
        m_bytesInQueue(0),
        m_markedCount(0),
        m_dropCount(0),
        m_lastCount(0),
        m_recInvSqrt(~0U >> REC_INV_SQRT_SHIFT),
        m_firstAboveTime(0),
        m_nextMarkingTime(0),
        m_state1(0),
        m_state2(0),
        m_state3(0),
        m_states(0),
        m_dropOverLimit(0),
        m_sojourn(0),
        sojournTime_before(0),
        probability(0),
        m_overTargetForInterval(false),
        m_markNext(false)
{
  NS_LOG_FUNCTION(this);
}

CoDelQueue2::~CoDelQueue2()
{
  NS_LOG_FUNCTION(this);
}

void
CoDelQueue2::NewtonStep(void)
{
  NS_LOG_FUNCTION(this);
  uint32_t invsqrt = ((uint32_t) m_recInvSqrt) << REC_INV_SQRT_SHIFT;
  uint32_t invsqrt2 = ((uint64_t) invsqrt * invsqrt) >> 32;
  uint64_t val = (3ll << 32) - ((uint64_t) m_markedCount * invsqrt2);

  val >>= 2; /* avoid overflow */
  val = (val * invsqrt) >> (32 - 2 + 1);
  m_recInvSqrt = val >> REC_INV_SQRT_SHIFT;
}

uint64_t
CoDelQueue2::getNextMarkingTime(uint64_t t)
{
  NS_LOG_FUNCTION(this);

  double rec_inv_sqrt = 1.1 / sqrt(m_markedCount);

  uint64_t addTime = Time2CoDel(m_interval) * rec_inv_sqrt;

  return t + addTime;
}

void
CoDelQueue2::SetMode(CoDelQueue2::QueueMode mode)
{
  NS_LOG_FUNCTION(mode);
  m_mode = mode;
}

CoDelQueue2::QueueMode
CoDelQueue2::GetMode(void)
{
  NS_LOG_FUNCTION(this);
  return m_mode;
}

bool
CoDelQueue2::DoEnqueue(Ptr<Packet> p)
{
  NS_LOG_FUNCTION(this << p);

  if (m_mode == QUEUE_MODE_PACKETS && (m_packets.size() + 1 > m_maxPackets)) {
    NS_LOG_LOGIC("Queue full (at max packets) -- droppping pkt");
//		std::cout << "Queue full (at max packets) -- droppping pkt";
    Drop(p);
    ++m_dropOverLimit;
    return false;
  }

  if (m_mode == QUEUE_MODE_BYTES && (m_bytesInQueue + p->GetSize() > m_maxBytes)) {
    NS_LOG_LOGIC("Queue full (packet would exceed max bytes) -- droppping pkt");
//		std::cout << "Queue full (packet would exceed max bytes) -- droppping pkt";
    Drop(p);
    ++m_dropOverLimit;
    return false;
  }

  // Tag packet with current time for DoDequeue() to compute sojourn time
  CoDelTimestampTag tag;
  p->AddPacketTag(tag);

  m_bytesInQueue += p->GetSize();
  m_packets.push(p);

  NS_LOG_LOGIC("Number packets " << m_packets.size());
  NS_LOG_LOGIC("Number bytes " << m_bytesInQueue);

  return true;
}

bool
CoDelQueue2::checkSojournTime(Ptr<Packet> p, uint64_t now)
{
  NS_LOG_FUNCTION(this);
  CoDelTimestampTag tag;
  bool found = p->RemovePacketTag(tag);
  NS_ASSERT_MSG(found, "found a packet without an input timestamp tag");
  NS_UNUSED(found);    //silence compiler warning
  Time delta = Simulator::Now() - tag.GetTxTime();
  NS_LOG_INFO("Sojourn time " << delta.GetSeconds());
  uint64_t sojournTime = Time2CoDel(delta);

  assert(m_bytesInQueue >= 0);
  // Sojourn time above target:
  if (CoDelTimeAfter(sojournTime, Time2CoDel(m_target))) {
    m_overTargetForInterval = false;

    if (m_firstAboveTime == 0) {
      /* just went above from below. If we stay above
       * for at least q->interval we'll say it's ok to drop
       */
      NS_LOG_LOGIC(
          "Sojourn time has just gone above target from below, need to stay above for "
              "at least q->interval before packet can be dropped. ");
      m_overTargetForInterval = false;
      m_firstAboveTime = now;
    }
    else {
      if (CoDelTimeAfter(now, (m_firstAboveTime + Time2CoDel(m_interval)))) {
        /* Queue has been over limit for longer than interval!
         * Set class state to mark next packet:
         
        m_sojourn = delta;
        m_overTargetForInterval = true;*/
        m_sojourn = delta;
        m_overTargetForInterval = true;
        probability = probability+0.125*(delta.GetSeconds()-m_target.GetSeconds())+1.25*(delta.GetSeconds()-sojournTime_before);
        if(probability<=0)  probability = 0;
        if(probability>=1)  probability = 1;
        Cwnd = (0.125*(m_target.GetSeconds()-delta.GetSeconds())-probability)/(1.375*delta.GetSeconds());
        sojournTime_before = delta.GetSeconds();
      }
    }

  }
  // Sojourn time below target:
  else {
    NS_LOG_LOGIC("Sojourn time is below target");
    m_firstAboveTime = 0;
    m_overTargetForInterval = false;
  }
  return m_overTargetForInterval;
}

Ptr<Packet>
CoDelQueue2::DoDequeue(void)
{
  NS_LOG_FUNCTION(this);

  assert(m_bytesInQueue >= 0);

  // If queue is empty: leave marking state
  if (m_packets.empty()) {
    m_overTargetForInterval = false;
    m_firstAboveTime = 0;
    NS_LOG_LOGIC("Queue empty");
    return 0;
  }

  uint64_t now = CoDelGetTime();
  Ptr<Packet> p = m_packets.front();
  m_packets.pop();
  m_bytesInQueue -= p->GetSize();

  NS_LOG_LOGIC("Popped " << p);
  NS_LOG_LOGIC("Number packets remaining " << m_packets.size());
  NS_LOG_LOGIC("Number bytes remaining " << m_bytesInQueue);

  bool okToMark = checkSojournTime(p, now);

  // If sojourn time over target for at least interval: Mark packets according
  // to decreasing interval length
  if (okToMark) {

    if (CoDelTimeAfterEq(now, m_nextMarkingTime)) {
      ++m_markedCount;

      m_markNext = true;
      m_nextMarkingTime = getNextMarkingTime(now);
    }
  }
  // If sojourn time falls below target: Reset markedCount
  else {
    m_markedCount = 0;
  }
  ++m_states;
  return p;
}

uint32_t
CoDelQueue2::GetQueueSize(void)
{
  NS_LOG_FUNCTION(this);
  if (GetMode() == QUEUE_MODE_BYTES) {
    return m_bytesInQueue;
  }
  else if (GetMode() == QUEUE_MODE_PACKETS) {
    return m_packets.size();
  }
  else {
    NS_ABORT_MSG("Unknown mode.");
    return 0;
  }
}

uint32_t
CoDelQueue2::GetDropOverLimit(void)
{
  return m_dropOverLimit;
}

uint32_t
CoDelQueue2::GetDropCount(void)
{
  return m_dropCount;
}

Time
CoDelQueue2::GetTarget(void)
{
  return m_target;
}

Time
CoDelQueue2::GetInterval(void)
{
  return m_interval;
}

uint32_t
CoDelQueue2::GetDropNext(void)
{
  return m_nextMarkingTime;
}

Ptr<const Packet>
CoDelQueue2::DoPeek(void) const
{
  NS_LOG_FUNCTION(this);

  if (m_packets.empty()) {
    NS_LOG_LOGIC("Queue empty");
    return 0;
  }

  Ptr<Packet> p = m_packets.front();

  NS_LOG_LOGIC("Number packets " << m_packets.size());
  NS_LOG_LOGIC("Number bytes " << m_bytesInQueue);

  return p;
}

bool
CoDelQueue2::CoDelTimeAfter(uint64_t a, uint64_t b)
{
  return ((int64_t) (a) - (int64_t) (b) > 0);
}

bool
CoDelQueue2::CoDelTimeAfterEq(uint64_t a, uint64_t b)
{
  return ((int64_t) (a) - (int64_t) (b) >= 0);
}

bool
CoDelQueue2::CoDelTimeBefore(uint64_t a, uint64_t b)
{
  return ((int64_t) (a) - (int64_t) (b) < 0);
}

bool
CoDelQueue2::CoDelTimeBeforeEq(uint64_t a, uint64_t b)
{
  return ((int64_t) (a) - (int64_t) (b) <= 0);
}

uint64_t
CoDelQueue2::Time2CoDel(Time t)
{
  return (t.GetNanoSeconds() >> CODEL2_SHIFT);
}

void
CoDelQueue2::Drop(Ptr<Packet> p)
{
  m_nTotalDroppedPackets++;
  m_nTotalDroppedBytes += p->GetSize();

  NS_LOG_LOGIC("m_traceDrop (p)");

  Queue::Drop(p);
}

//bool CoDelQueue2::isInDroppingState()
//{
//	return m_dropping.Get();
//}

bool
CoDelQueue2::traceOkToDrop()
{
  return m_overTargetForInterval;
}

// TODO: Make sure to only call this at one place!
bool
CoDelQueue2::isOkToMark()
{
  if (m_markNext) {
    m_markNext = false;
    if (m_overTargetForInterval == false) {
      std::cout << "Marking next packet, but sojourn time fell below target value!\n";
    }
    return m_overTargetForInterval;
  }
  else {
    return false;
  }
}

bool
CoDelQueue2::isQueueOverLimit(double limit)
{
  assert(limit >= 0 && limit <= 1);
  if (m_bytesInQueue > m_maxBytes * limit || m_packets.size() > m_maxPackets * limit) {
    return true;
  }
  else {
    return false;
  }
}

int64_t
CoDelQueue2::getTimeOverLimitInNS()
{
  uint64_t now = CoDelGetTime();

  if (m_firstAboveTime == 0) {
    return 0;
  }
  else {
    if (((int64_t)now - (int64_t)m_firstAboveTime) < 0){
      std::cout << "ERROR: firstAboveTime shouldn't be after current time!\n";
    }
    return (now - m_firstAboveTime);
  }

}

} // namespace ns3


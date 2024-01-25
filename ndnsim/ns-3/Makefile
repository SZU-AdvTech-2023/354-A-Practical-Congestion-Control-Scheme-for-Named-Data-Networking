# Makefile wrapper for waf

all:
	./waf

# free free to change this part to suit your requirements
configure:
	#./waf configure --enable-examples --enable-tests
	./waf configure -d optimized --enable-examples --disable-python --disable-tests

build:
	./waf build

install:
	./waf install

clean:
	./waf clean

distclean:
	./waf distclean

CMAKE_BUILD_TYPE=$(shell if [ $(DEBUG) ]; then echo Debug; else echo RelWithDebInfo; fi)

all:
	@mkdir -p build; rm -f build/CMakeCache.txt rave/defines.h rave/classhashes.h
	@if [ $(prefix) ]; then \
		cd build && cmake -DCMAKE_INSTALL_PREFIX=$(prefix) -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) ..; \
	else \
		cd build && cmake -DCMAKE_VERBOSE_MAKEFILE=OFF -DCMAKE_BUILD_TYPE=$(CMAKE_BUILD_TYPE) ..; \
	fi
	cd build && $(MAKE) $(PARALLEL_JOBS)

install:
	cd build && $(MAKE) $(PARALLEL_JOBS) install

uninstall:
	cd build && $(MAKE) uninstall

test: all
	cd test && python test_ikfast.py

clean:
	-cd build && $(MAKE) clean
	rm -rf build

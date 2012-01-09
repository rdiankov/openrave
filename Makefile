CMAKE_BUILD_TYPE=$(shell if [ $(DEBUG) ]; then echo Debug; else echo RelWithDebInfo; fi)

all:
	@mkdir -p build; rm -f build/CMakeCache.txt
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
	cd build && $(MAKE) $(PARALLEL_JOBS) install
	export PATH=`sh build/openrave-config --prefix`/bin:$(PATH) && export PYTHONPATH=`openrave-config --python-dir`:$(PYTHONPATH) && export LD_LIBRARY_PATH=`openrave-config --prefix`/lib:$(LD_LIBRARY_PATH) && cd test && python run_tests.py $PARALLEL_JOBS

docs: all
	cd build && $(MAKE) $(PARALLEL_JOBS) install
	export PATH=`sh build/openrave-config --prefix`/bin:$(PATH) && export PYTHONPATH=`openrave-config --python-dir`:$(PYTHONPATH) && export LD_LIBRARY_PATH=`openrave-config --prefix`/lib:$(LD_LIBRARY_PATH) && cd docs && ./build.sh

clean:
	-cd build && $(MAKE) clean
	rm -rf build

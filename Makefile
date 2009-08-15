 all:
	@mkdir -p build
	@if [ $(prefix) ]; then \
		cd build && cmake -DCMAKE_INSTALL_PREFIX=$(prefix) -DCMAKE_BUILD_TYPE=RelWithDebInfo ..; \
	else \
		cd build && cmake -DCMAKE_VERBOSE_MAKEFILE=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo ..; \
	fi
	cd build && $(MAKE) $(PARALLEL_JOBS)

install:
	cd build && $(MAKE) $(PARALLEL_JOBS) install

uninstall:
	cd build && $(MAKE) uninstall

test: all
	cd build && $(MAKE) test

test-future: all
	cd build && $(MAKE) test-future

clean:
	-cd build && $(MAKE) clean
	rm -rf build

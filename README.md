CppGC: C++ Garbage Collector
=============

The repository is a read-only mirror of v8/cppgc. Not intended to be used for development.

### Steps to build:
1. git clone ...
2. git submodule update --init #for gtest
3. mkdir build && cd build
4. cmake ../ -DCPPGC_YOUNG_GENERATION=ON && make -j
5. ./cppgc_sample

### Build and run tests:
1. make -j tests
2. ./cppgc_unittests

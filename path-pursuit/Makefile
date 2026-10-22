CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

LIB     = libpath.a
OBJ     = src/path.o

all: $(LIB) follow test_path

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/path.o: src/path.c include/path.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

follow: examples/follow.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_path: test/test_path.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_path cpp-smoke
	./test_path

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: follow
	./follow

clean:
	rm -f $(OBJ) $(LIB) follow test_path test_cpp_linkage

.PHONY: all check cpp-smoke run clean

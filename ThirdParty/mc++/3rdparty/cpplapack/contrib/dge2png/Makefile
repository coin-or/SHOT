###############################################################################
##                                 Makefile                                  ##
###############################################################################

A.OUT = dge2png

###############################################################################

#include Makefile.compiler
include Makefile.g++
#include Makefile.icpc

###############################################################################

HEADERS:= $(shell find -iname '*.hpp')
SOURCES:= $(shell find -iname '*.cpp')
OBJECTS:= $(SOURCES:%.cpp=%.o)

###############################################################################

all: depend $(OBJECTS)
	$(CXX) $(OBJECTS) $(LFLAGS) $(LIB_DIRS) $(LIBS) -o $(A.OUT)
	@echo

.SUFFIXES: .cpp .o
.cpp.o:
	$(CXX) -c $< $(CFLAGS) $(INCLUDE_DIRS) $(MACROS) -o $@
	@echo

depend:
#	touch main.cpp
	makedepend -f- -Y $(SOURCES) > Makefile.depend 2> /dev/null
#	gccmakedep -- -I./ -MM -- $(SOURCES)
#	$(CXX) -MM -I./ $(SOURCES) > Makefile.depend
	@echo

clean:
	rm -f $(OBJECTS)

fullclean:
	rm -f $(shell find -name '*.o') std err Makefile.depend $(A.OUT)

remake: clean all

###############################################################################
-include Makefile.depend

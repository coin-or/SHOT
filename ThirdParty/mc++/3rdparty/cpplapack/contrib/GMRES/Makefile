###############################################################################
##                                 Makefile                                  ##
###############################################################################

A.OUT = A.OUT

###############################################################################

include Makefile.g++

###############################################################################

HEADERS:= $(shell find ./ -iname '*.hpp')
SOURCES:= $(shell find ./ -iname '*.cpp')
#SOURCES:= $(wildcard */*.cpp *.cpp)
SOURCES:= $(sort $(SOURCES))
#SOURCES:= $(filter-out ./aho.cpp, $(SOURCES))

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
#	$(CXX) -MM $(INCLUDE_DIRS) $(SOURCES) > Makefile.depend
	@echo

clean:
	rm -f $(OBJECTS)

fullclean:
	rm -f $(shell find -name '*.o') std err Makefile.depend $(A.OUT)

remake: clean all

###############################################################################
-include Makefile.depend

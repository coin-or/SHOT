CPP = CC

#COMP_FLAGS = -n32 -Ofast -LANG:std
COMP_FLAGS = -mp -n32 -Ofast -LANG:std
LINK_FLAGS = -mp -n32 -Ofast -LANG:std

#MACROS = -DCPPL_VERBOSE -DCPPL_DEBUG

CPPLAPACK_INCLUDE_DIR = $(HOME)/local/lib/cpplapack/include

INCLUDE_DIRS = -I$(CPPLAPACK_INCLUDE_DIR)
LIB_DIRS = 

#LIBS = -lscs -lfastm -lm
LIBS = -lscs_mp -lfastm -lm
OBJECTS = main.o

###############################################################################

A.OUT: $(OBJECTS)
	$(CPP) $(OBJECTS) $(LINK_FLAGS) $(LIB_DIRS) $(LIBS) -o $@

.SUFFIXES: .cpp .o

.cpp.o:
	$(CPP) -c $< $(COMP_FLAGS) $(INCLUDE_DIRS) $(MACROS)

do: A.OUT
	./A.OUT > std 2> err

clean:
	rm -f *~ $(OBJECTS)

fullclean:
	rm -f *~ $(OBJECTS) A.OUT std err 

remake: clean A.OUT

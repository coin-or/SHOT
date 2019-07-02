###############################################################################
##                                 Makefile                                  ##
###############################################################################

A.OUT = A.OUT

###############################################################################

include $(HOME)/local/cpplapack/makefiles/Makefile.g++

###############################################################################

HEADERS:= $(shell find ./ -iname '*.hpp')
SOURCES:= $(shell find ./ -iname '*.cpp')
SOURCES:= $(sort $(SOURCES))
OBJECTS:= $(SOURCES:%.cpp=%.o)
THIS_FILE:= $(firstword $(MAKEFILE_LIST))

###############################################################################
###############################################################################
###############################################################################

release: depend
	@echo -e "##################################"
	@echo -e "#### building in release mode ####"
	@echo -e "##################################\n"
	@$(MAKE) -f $(THIS_FILE) --no-print-directory build RELEASE=1

profile: depend
	@echo -e "##################################"
	@echo -e "#### building in profile mode ####"
	@echo -e "##################################\n"
	@$(MAKE) -f $(THIS_FILE) --no-print-directory build PROFILE=1

debug: depend
	@echo -e "################################"
	@echo -e "#### building in debug mode ####"
	@echo -e "################################\n"
	@$(MAKE) -f $(THIS_FILE) --no-print-directory build DEBUG=1

verbose: depend
	@echo -e "##################################"
	@echo -e "#### building in verbose mode ####"
	@echo -e "##################################\n"
	@$(MAKE) -f $(THIS_FILE) --no-print-directory build DEBUG=1 VERBOSE=1

###############################################################################

.SUFFIXES: .cpp .o
.cpp.o:
	$(CXX) -c $< $(CFLAGS) $(INCLUDE_DIRS) $(MACROS) -o $@
	@echo

build: $(OBJECTS)
	$(CXX) $(OBJECTS) $(LFLAGS) $(LIB_DIRS) $(LIBS) -o $(A.OUT)
	@echo

###############################################################################

depend:
	makedepend -f- -Y $(SOURCES) > Makefile.depend 2> /dev/null
#	gccmakedep -- -I./ -MM -- $(SOURCES)
#	$(CXX) -MM -I./ $(SOURCES) > Makefile.depend
#	$(CXX) -MM $(INCLUDE_DIRS) $(SOURCES) > Makefile.depend
	@echo

clean:
	rm -f $(OBJECTS)

fullclean:
	rm -f $(shell find -name '*.o') Makefile.depend std err $(A.OUT)

remake: clean all

###############################################################################
-include Makefile.depend

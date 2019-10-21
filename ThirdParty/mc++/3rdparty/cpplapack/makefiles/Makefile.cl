CPP = cl.exe

COMP_FLAGS = /nologo /EHsc /w /MD
LINK_FLAGS = /link /nologo

#MACROS = -DCPPL_VERBOSE -DCPPL_DEBUG

INCLROOT=C:\lib\cpplapack\cpplapack
CPPLAPACK_INCLUDE_DIR = "$(INCLROOT)\include"
INCLUDE_DIRS = /I $(CPPLAPACK_INCLUDE_DIR) 

LIBSROOT=C:\lib\CLAPACK3-Windows
LIBS = "$(LIBSROOT)\clapack\Release\clapack.lib" "$(LIBSROOT)\clapack\BLAS\Release\blas.lib"

OBJECTS = main.obj

###############################################################################

A.EXE: $(OBJECTS)
	$(CPP) $(OBJECTS) /o $@ $(LINK_FLAGS) $(LIBS)

.SUFFIXES: .cpp .obj

.cpp.obj:
	$(CPP) /c $(COMP_FLAGS) $(INCLUDE_DIRS) $<

do: A.OUT
	.\A.EXE > std 2> err

clean:
	del $(OBJECTS)

fullclean:
	del $(OBJECTS) A.EXE std err 

remake: clean A.EXE

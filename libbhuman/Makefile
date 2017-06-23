# Makefile used to build libraries
include ../Makefile.am

TARGETS=all clean
.PHONY: $(TARGETS)

NaoQiSDK=/usr
NaoQi_INC = /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include
NaoQi_LIB = -L/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib -lalcommon -lalerror -lalproxies -lalmemoryfastaccess
INCLUDES=$(NaoQi_INC)

CXXFLAGS +=-DDEBUG

all: bhlowcmd

%.o: %.cpp %.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

bhlowcmd: lua_dcmII.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -llua $(NaoQi_LIB)

clean:
	rm -f *.$(SHLIBEXT) *.o
# Makefile used to build libraries
include ../Makefile.am

TARGETS=all clean
.PHONY: $(TARGETS)

NaoQiSDK=/usr
NaoQi_INC = /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/include
NaoQi_LIB = -L/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib -lalcommon -lalerror -lalproxies -lalmemoryfastaccess
INCLUDES=$(NaoQi_INC)

CXXFLAGS +=-DDEBUG

all: dcmII dcm_test

%.o: %.cpp %.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ -c $<

libbhuman: bhuman.o

	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -llua $(NaoQi_LIB)

bhlowcmd: lua_dcmII.o
	$(LD) $(LDFLAGS) -o $@.$(SHLIBEXT) $(LIBOPTS) $^ $(LIB_DIR) -lm -llua $(NaoQi_LIB)

libdcmII: dcmII.o
	$(LD) $(LDFLAGS) -o $@.so $(LIBOPTS) $^ $(LIB_DIR) -lm -llua $(NaoQi_LIB) -Wl,-rpath,/home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib/ $(NaoQi_LIB)
	# libalcommon.so /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib/libalerror.so /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib/libalproxies.so /home/nao/newnaoSDK/naoqi-sdk-2.1.4.13-linux32/lib/libalmemoryfastaccess.so
	# ar rvs libdcmII.a dcmII.o 

dcm_test: test_dcmII.o 
	$(LD) -o $@ $(LIBOPTS) $^ $(LIB_DIR) -lm $(NaoQi_LIB) $(INCLUDES)

# dcm_test: test_dcmII.o 
	# $(LD) $(LDFLAGS) -o $@ $(LIBOPTS) $^ $(LIB_DIR) -lm -llua $(NaoQi_LIB)

clean:
	rm -f *.$(SHLIBEXT) *.o

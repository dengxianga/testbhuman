include ../../../../../../Makefile.am

TARGETS=all clean
.PHONY: $(TARGETS)

NaoQi_INC = -I/home/nao/naoqi-sdk-2.1.4.13-linux32/include
NaoQi_LIB = -L/home/nao/naoqi-sdk-2.1.4.13-linux32/lib -lalcommon -lalerror -lalproxies -lalmemoryfastaccess

BHuman_INC = -I/home/UPennDev/Lib/Platform/NaoV4/NaoQiV5/testbhuman/libbhuman
BHuman_LIB = -L/UPennDev/Lib/Platform/NaoV4/NaoQiV5/testbhuman/libbhuman/libbhuman.so

INCLUDESN=$(NaoQi_INC)
INCLUDESB=$(BHuman_INC)

CXXFLAGS +=-DDEBUG

all: testb 

%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDESB) $(INCLUDESN) -o $@ -c $<

testb: testb.o
	# $(CXX) -o testb testb.o $(BHuman_LIB) 
	$(LD) $(LDFLAGS) -o $@ $(LIBOPTS) $(LIB_DIR) $^ -lm -lluajit-5.1 $(BHuman_LIB) 

# bhumanemu: bhumanemu.o
# 	# $(CXX) -o $@ $^ $(BHuman_LIB) 
# 	$(LD) $(LDFLAGS) -o $@ $(LIBOPTS) $(LIB_DIR) $^ -lm -lluajit-5.1 $(BHuman_LIB) $(NaoQi_LIB) 

clean:
	rm -f testb bhumanemu *.o
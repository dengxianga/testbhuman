cd ~/testbhuman/libbhuman/cmakebhuman/build/
rm -r *
CXX=g++ cmake ..
make
cd ~/testbhuman/libbhuman/cmakedcmII/build/
rm -r *
CXX=g++ cmake ..
make
cd ~/testbhuman/libbhuman/cmakeluadcmII/build/
rm -r *
CXX=g++ cmake ..
make
cd ~/testbhuman/libbhuman/test_dcmII/build/
rm -r *
CXX=g++ cmake ..
make
scp ~/testbhuman/libbhuman/test_dcmII/build/test_dcmII ~/testbhuman/libbhuman/bhlowcmd_wrapper.lua ~/testbhuman/libbhuman/test_NaoBody.lua nao@192.168.1.117:~/UPennDev/Player
scp ~/testbhuman/libbhuman/libdcmII.so nao@192.168.1.117:~/testbhuman/libbhuman
scp ~/testbhuman/libbhuman/libbhuman.so ~/testbhuman/libbhuman/libbhlowcmd.so ~/testbhuman/libbhuman/NaoBodyII.lua nao@192.168.1.117:~/UPennDev/Player/Lib

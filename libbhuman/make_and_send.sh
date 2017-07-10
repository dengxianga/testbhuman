cd ~/testbhuman/libbhuman/cmakebhuman/build/
rm -r *
CXX=g++ cmake ..
make
echo "libbhuman.so done!"
# cd ~/testbhuman/libbhuman/cmakedcmII/build/
# rm -r *
# CXX=g++ cmake ..
# make
# echo "dcmII.so done!"
cd ~/testbhuman/libbhuman/
make all
echo "bhlowcmd.so done!"
# cd ~/testbhuman/libbhuman/test_dcmII/build/
# rm -r *
# CXX=g++ cmake ..
# make
# echo "test_dcmII done!"
scp ~/testbhuman/libbhuman/test_dcmII/build/test_dcmII ~/testbhuman/libbhuman/bhlowcmd_wrapper.lua ~/testbhuman/libbhuman/test_NaoBodyII.lua nao@192.168.1.119:~/UPennDev/Player
# scp ~/testbhuman/libbhuman/libdcmII.so nao@192.168.1.119:~/testbhuman/libbhuman
scp ~/testbhuman/libbhuman/libbhuman.so ~/testbhuman/libbhuman/bhlowcmd.so ~/testbhuman/libbhuman/NaoBodyII.lua nao@192.168.1.119:~/UPennDev/Player/Lib

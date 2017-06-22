cd ~/testbhuman/libbhuman/cmakedcmII/build/
rm -r *
CXX=g++ cmake ..
make
cd ~/testbhuman/libbhuman/test_dcmII/build/
rm -r *
CXX=g++ cmake ..
make
sudo scp ~/testbhuman/libbhuman/test_dcmII/build/test_dcmII nao@192.168.1.117:~/UPennDev/Player
scp ~/testbhuman/libbhuman/libdcmII.so nao@192.168.1.117:~/testbhuman/libbhuman


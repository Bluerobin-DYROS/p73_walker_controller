# 1. Prerequisites
## 1. Pinocchio
```
sudo apt install ros-jazzy-pinocchio
```
## 2. br_driver
```
git clone https://github.com/Bluerobin-DYROS/p73_sys_ws.git
cd p73_sys_ws
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```
## 3. OSQP
```
cd ~/Downloads
git clone https://github.com/osqp/osqp
cd osqp
mkdir build && cd build
cmake -G "Unix Makefiles" ..
make
sudo make install
```
## 4. OSQP-Eigen
```
cd ~/Downloads
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build
cd build
cmake ..
make
sudo make install
```

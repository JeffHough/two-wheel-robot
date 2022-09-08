## To run this tutorial:
- Install cmake: `sudo apt update && sudo apt upgrade && sudo apt install cmake`
- Install git: `sudo apt install git-all`
- Install pip3: `sudo apt install python3-pip`
- Install kivy: `pip3 install kivy`
- Install Eigen3 (see below)
```
git clone https://gitlab.com/libeigen/eigen.git 
cd eigen  
mkdir build && cd build 
cmake ..
cmake --build .
sudo cmake --install .
```
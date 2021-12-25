https://gitlab.com/sdurobotics/ur_rtde

##### Installation #####

### x64 ###

1. sudo add-apt-repository ppa:sdurobotics/ur-rtde
2. sudo apt-get update && sudo apt upgrade
3. sudo apt install librtde librtde-dev
4. python -m pip install ur_rtde --user

### arm64 ###

sudo apt install libc6-dev libboost-all-dev libncurses5-dev libncursesw5-dev
git clone --recurse-submodules https://gitlab.com/sdurobotics/ur_rtde.git
cd ur_rtde
mkdir build && cd $_
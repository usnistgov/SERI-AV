#!/bin/bash


#Carla Server
apt update && apt install libomp5 xdg-utils -y
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.12.tar.gz
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/AdditionalMaps_0.9.12.tar.gz

tar -xzf CARLA_0.9.12.tar.gz --one-top-level
mv AdditionalMaps_0.9.12.tar.gz ./CARLA_0.9.12/Import/
cd CARLA_0.9.12 && ./ImportAssets.sh && cd ..
useradd carla_user
chown -R carla_user .

#Carla Client
pip install --upgrade --ignore-installed pip
pip3 install carla==0.9.12

#Scenario Runner
git clone https://github.com/carla-simulator/scenario_runner.git
cd scenario_runner && git checkout v0.9.12
mv ../av_files/Reveal_wEgo52.xosc .
pip3 install --user -r requirements.txt
cd ..

#Autoware Map
#TODO: host and download carla-town-4.zip
mkdir autoware_map && mv av_files/carla-town-4.zip autoware_map
cd autoware_map && unzip carla-town-4.zip && cd ..

#Autoware
cat av_files/extra-repos.txt >> /autoware/autoware.repos
cd /autoware && vcs import --recursive src < autoware.repos
rosdep update --include-eol-distros
source /opt/ros/galactic/setup.bash
rosdep install -y --from-paths src --ignore-src --rosdistro galactic -y -r
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

#Config file changes
cp -R /seri_av/av_files/install .

#Other deps
pip3 install pygame

#Demo workspace
source /autoware/install/setup.bash
cd /seri_av/seri_ros_ws && colcon build

#Bashrc
cat /seri_av/av_files/bash-function.txt >> ~/.bashrc
source ~/.bashrc

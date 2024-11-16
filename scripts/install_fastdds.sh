#!/bin/bash

FASTCDR_VERSION=v1.0.24
FOONATHAN_VERSION=v2.4.1
FASTDDS_VERSION=v2.6.1
FASTGEN_VERSION=v2.1.3

SOURCE_PATH=~/Software/fastdds
INSTALL_PATH=/opt/robosw

## install dependency
# build tools
sudo apt install -y cmake g++ python3-pip wget git
# minimal dependencies
sudo apt install -y libasio-dev libtinyxml2-dev
sudo apt install -y libssl-dev
# dependencies for security features
sudo apt install -y libp11-dev libengine-pkcs11-openssl softhsm2
# dependencies for IDL generator
sudo apt install -y openjdk-8-jdk gradle

# create folder 
mkdir -p $SOURCE_PATH

# Fast-CDR
cd $SOURCE_PATH
git clone https://github.com/eProsima/Fast-CDR.git
mkdir -p Fast-CDR/build && cd Fast-CDR/build
git checkout $FASTCDR_VERSION
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH ..
make -j
sudo make install

# Foonathan memory
cd $SOURCE_PATH
git clone https://github.com/eProsima/foonathan_memory_vendor.git
mkdir -p foonathan_memory_vendor/build && cd foonathan_memory_vendor/build
git checkout $FOONATHAN_VERSION
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH -DBUILD_SHARED_LIBS=ON ..
make -j
sudo make install

# FastDDS
cd $SOURCE_PATH
git clone https://github.com/eProsima/Fast-DDS.git
mkdir -p Fast-DDS/build && cd Fast-DDS/build
git checkout $FASTDDS_VERSION
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_PATH -DCOMPILE_EXAMPLES=ON ..
make -j
sudo make install

# Fast-RTPS-Gen
cd $SOURCE_PATH
git clone --recursive https://github.com/eProsima/Fast-DDS-Gen.git
cd Fast-DDS-Gen
git checkout $FASTGEN_VERSION
./gradlew assemble

# Additional setup
sudo usermod -a -G softhsm $USER
echo -e "\nexport FASTDDS_DIR=/opt/robosw" >> ~/.bashrc 
echo 'export PATH=$PATH:~/Software/fastdds/Fast-DDS-Gen/scripts' >> ~/.bashrc 
echo 'export LD_LIBRARY_PATH=$FASTDDS_DIR:$LD_LIBRARY_PATH' >> ~/.bashrc

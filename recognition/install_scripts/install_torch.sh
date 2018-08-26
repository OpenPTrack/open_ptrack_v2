#!/bin/bash
git clone https://github.com/torch/distro.git ~/torch --recursive
cd ~/torch
sudo bash install-deps
sudo ./install.sh
source ~/.bashrc

cd install/bin
sudo ./luarocks install dpnn
sudo ./luarocks install csvigo

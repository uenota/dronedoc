#!/bin/bash

# install pip for python3
sudo apt update && sudo apt -y upgrade
sudo apt install python3-pip -y

# update pip and other packages for sphinx
sudo -H pip3 install -U pip
sudo -H pip3 install -U sphinx sphinx-rtd-theme sphinx-sitemap
sudo -H pip3 install -U setuptools

# install sphinx-contrib.youtube 
cd /var/tmp
git clone https://github.com/sphinx-contrib/youtube.git
cd youtube
sudo python3 setup.py install
cd ..
sudo rm -rf youtube

cd ~
#!/bin/bash
cd /tmp

rm -rf dlib
git clone https://github.com/davisking/dlib.git
cd dlib
git checkout v19.4
mkdir python_examples/build
cd python_examples/build

#wget http://dlib.net/files/dlib-19.4.tar.bz2
#wget https://github.com/davisking/dlib/releases/download/v18.16/dlib-18.16.tar.bz2

# tar xzvf v19.4.tar.gz
# cd dlib-19.4/python_examples
# mkdir build
# cd build

# USE_AVX_INSTRUCTION may cause a compilation error on an old PC
# this flag should be switchable
cmake ../../tools/python -DUSE_AVX_INSTRUCTIONS=ON
cmake --build . --config Release
sudo cp dlib.so /usr/local/lib/python2.7/dist-packages

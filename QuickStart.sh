#bin/bash

chmod +x

rm -rf build
mkdir build

cd build

cmake ..
make

cd bin

./findarmor

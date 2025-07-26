#!/bin/bash

export PICO_SDK_PATH="~/code/pico/pico-sdk"

if [ -d "$build" ]; then
rm -rf ./build
fi

mkdir build
cd build
cmake .. -DFAMILY=rp2040
make -j4

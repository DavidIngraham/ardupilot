#!/bin/sh
set -eu
cd "$(dirname "$0")"
g++ -O3 -std=c++17 -shared -fPIC -I/usr/include/eigen3 dynamics.cpp -o libdynamics.so

#!/bin/bash

cmake -B build
cmake --build build --target mono_euroc -j 4
cmake --build build --target mono_tumrgbd -j 4

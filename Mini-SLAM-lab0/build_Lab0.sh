#!/bin/bash

cmake -B build
cmake --build build --target task_1 -j 8
cmake --build build --target task_2 -j 8
cmake --build build --target task_3 -j 8
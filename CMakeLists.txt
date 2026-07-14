cmake_minimum_required(VERSION 3.16)
set(EXTRA_COMPONENT_DIRS common drivers app/lsm6dsox gpio)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(NanoESP32)

# This example is designed to be built as a standalone ESP-IDF project
# It references the bno085 component via a relative path

cmake_minimum_required(VERSION 3.16)

# Set up component directories to include the bno085 component
set(EXTRA_COMPONENT_DIRS "../../")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bno085_basic_read)

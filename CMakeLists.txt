# Builds under ESP-IDF v5.5.x and v6.0.
#
# A build directory belongs to the IDF version that configured it. Its CMake
# cache holds absolute paths to that version's compiler, so pointing a
# different IDF at it fails during configure with
#
#     Tool doesn't match supported version from list ['esp-15.2.0_20251204']
#
# naming the *other* version's toolchain. Nothing is wrong with the sources:
# delete the build directory (or run `idf.py fullclean`) and configure again.
# Alternating often is easier with one directory each, `idf.py -B build-v6`.
#
# The v6.0-specific parts of the port are in components/cloud (cJSON moved to
# the component registry) and components/wifi (a removed enum).

# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(indoorV10-fw-birdy)

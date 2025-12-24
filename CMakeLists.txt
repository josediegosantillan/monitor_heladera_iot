cmake_minimum_required(VERSION 3.5)

# Export compile database for IDE IntelliSense (e.g., VS Code C/C++).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(monitor_heladera_iot)

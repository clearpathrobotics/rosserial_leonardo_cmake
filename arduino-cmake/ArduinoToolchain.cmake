#=============================================================================#
# Author: Tomasz Bogdal (QueezyTheGreat)
# Home:   https://github.com/queezythegreat/arduino-cmake
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this file,
# You can obtain one at http://mozilla.org/MPL/2.0/.
#=============================================================================#
set(CMAKE_SYSTEM_NAME Arduino)

set(CMAKE_C_COMPILER   avr-gcc)
set(CMAKE_CXX_COMPILER avr-g++)

#@[if DEVELSPACE]@
# Arduino location in develspace
#set(ARDUINO_SDK_PATH "@(CMAKE_BINARY_DIR)/arduino-1.0.5")
#@[else]@
# Arduino location in installspace
# set(ARDUINO_SDK_PATH "${rosserial_leonardo_cmake_DIR}/../../../@(CATKIN_PACKAGE_SHARE_DESTINATION)/arduino-1.0.5")
message("SDK Path: ${ARDUINO_SDK_PATH}")
#@[end if]@

# Add current directory to CMake Module path automatically
if(EXISTS ${CMAKE_CURRENT_LIST_DIR}/Platform/Arduino.cmake)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_LIST_DIR})
endif()

list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${ARDUINO_SDK_PATH}/hardware/tools/avr/bin)
list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${ARDUINO_SDK_PATH}/hardware/tools/avr/utils/bin)

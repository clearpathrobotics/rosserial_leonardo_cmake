cmake_minimum_required(VERSION 2.8.3)

project(rosserial_leonardo_cmake)

find_package(catkin REQUIRED)
catkin_package(
  CFG_EXTRAS rosserial_leonardo_cmake-extras.cmake)

set(ARDUINO_VERSION "1.0.5")

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(ARDUINO_DOWNLOAD "arduino-${ARDUINO_VERSION}-linux64.tgz")
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(ARDUINO_DOWNLOAD "arduino-${ARDUINO_VERSION}-linux32.tgz")
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

# Download and unzip the Arduino tarball.
file(DOWNLOAD http://arduino.googlecode.com/files/${ARDUINO_DOWNLOAD}
  ${PROJECT_BINARY_DIR}/${ARDUINO_DOWNLOAD})
execute_process(
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  COMMAND tar xzf ${ARDUINO_DOWNLOAD} -C ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
                                      --exclude=reference
                                      --exclude=examples
                                      --exclude=arduino-${ARDUINO_VERSION}/arduino
                                      --exclude=*.jar
                                      --exclude=*.so
                                      --exclude=lib/theme
                                      --exclude=arduino-${ARDUINO_VERSION}/tools
                                      --exclude=arduino-${ARDUINO_VERSION}/hardware/tools
  )

# Legacy target from when the download/untar happened at build time rather than configure time.
add_custom_target(${PROJECT_NAME} ALL
  DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/arduino-${ARDUINO_VERSION} 
)

# Install shared resources, including the sources extracted from the Arduino tarball.
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/cmake
  ${PROJECT_SOURCE_DIR}/arduino-cmake
  ${PROJECT_SOURCE_DIR}/bootloader
  ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/arduino-${ARDUINO_VERSION}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install scripts
install(
  PROGRAMS ${PROJECT_SOURCE_DIR}/scripts/upload
           ${PROJECT_SOURCE_DIR}/scripts/enter_bootloader
           ${PROJECT_SOURCE_DIR}/scripts/program_bootloader
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 2.8.3)

project(rosserial_leonardo_cmake)

find_package(catkin REQUIRED COMPONENTS rosserial_msgs)
catkin_package(CFG_EXTRAS rosserial_leonardo_cmake-extras.cmake)

set(ARDUINO_VERSION "1.0.5")

if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(ARDUINO_DOWNLOAD "arduino-${ARDUINO_VERSION}-linux64.tgz")
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(ARDUINO_DOWNLOAD "arduino-${ARDUINO_VERSION}-linux32.tgz")
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

# Download and unzip the Arduino tarball.
add_custom_command(
  OUTPUT ${PROJECT_BINARY_DIR}/arduino-${ARDUINO_VERSION}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
  COMMAND wget http://arduino.googlecode.com/files/${ARDUINO_DOWNLOAD}
  COMMAND tar xzf ${ARDUINO_DOWNLOAD}
)

# Copy avrdude.conf to the devel share folder.
add_custom_command(
  DEPENDS ${PROJECT_BINARY_DIR}/arduino-${ARDUINO_VERSION}
  OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/avrdude.conf
  COMMAND cp ${PROJECT_BINARY_DIR}/arduino-${ARDUINO_VERSION}/hardware/tools/avrdude.conf 
             ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/avrdude.conf
)

# Copy appropriate avrdude binary to devel lib folder.
add_custom_command(
  DEPENDS ${PROJECT_BINARY_DIR}/arduino-${ARDUINO_VERSION}
  OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/avrdude
  COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}
  COMMAND cp ${PROJECT_BINARY_DIR}/arduino-${ARDUINO_VERSION}/hardware/tools/avrdude
             ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/avrdude
)

# Target to make all this stuff happen.
add_custom_target(${PROJECT_NAME} ALL
  DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/avrdude
          ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/avrdude.conf
)

# Install shared resources.
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/cmake
  ${PROJECT_SOURCE_DIR}/arduino-cmake
  ${PROJECT_SOURCE_DIR}/bootloader
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install avrdude configuration.
install(
  FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/avrdude.conf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install scripts and avrdude binary.
install(
  PROGRAMS ${PROJECT_SOURCE_DIR}/scripts/upload
           ${PROJECT_SOURCE_DIR}/scripts/enter_bootloader
           ${PROJECT_SOURCE_DIR}/scripts/program_bootloader
           ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/avrdude
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

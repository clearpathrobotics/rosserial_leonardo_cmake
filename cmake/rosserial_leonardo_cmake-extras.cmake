cmake_minimum_required(VERSION 2.8.3)

set(ARDUINO_VERSION "1.0.5")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(ARDUINO_DOWNLOAD "arduino-${ARDUINO_VERSION}-linux64.tgz")
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(ARDUINO_DOWNLOAD "arduino-${ARDUINO_VERSION}-linux32.tgz")
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

function(rosserial_leonardo_firmware FIRMWARE)
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${FIRMWARE})

  # Download and unzip the Arduino headers.
  add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/arduino-${ARDUINO_VERSION}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
    COMMAND wget http://arduino.googlecode.com/files/${ARDUINO_DOWNLOAD}
    COMMAND tar xzf ${ARDUINO_DOWNLOAD}
  )

  # Generate the rosserial messages and ros_lib code
  add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/${FIRMWARE}/ros_lib
    COMMAND rosrun rosserial_arduino make_libraries.py ${PROJECT_BINARY_DIR}/${FIRMWARE}
    COMMAND cp ${PROJECT_SOURCE_DIR}/${FIRMWARE}/ros_lib/* ${PROJECT_BINARY_DIR}/${FIRMWARE}/ros_lib/
  )

  add_custom_target(${PROJECT_NAME}_${FIRMWARE}_ros_lib DEPENDS ${PROJECT_BINARY_DIR}/${FIRMWARE}/ros_lib)

  # Add more message-build dependencies if your firmware requires messages.
  add_dependencies(${PROJECT_NAME}_${FIRMWARE}_ros_lib rosserial_msgs_genpy std_msgs_genpy)

  # Generate a call to CMake inside the firmware subdirectory (firmware itself is generated outside of catkin).
  execute_process(COMMAND catkin_find rosserial_leonardo_cmake arduino-cmake/ArduinoToolchain.cmake --first-only
    OUTPUT_VARIABLE ARDUINO_TOOLCHAIN)
  add_custom_command(
    DEPENDS ${PROJECT_BINARY_DIR}/arduino-${ARDUINO_VERSION}
            ${PROJECT_BINARY_DIR}/${FIRMWARE}/ros_lib
    OUTPUT ${PROJECT_BINARY_DIR}/${FIRMWARE}/${FIRMWARE}.hex
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${FIRMWARE}
    COMMAND ${CMAKE_COMMAND} ${PROJECT_SOURCE_DIR}/${FIRMWARE}
        -DARDUINO_SDK_PATH=${PROJECT_BINARY_DIR}/arduino-${ARDUINO_VERSION}
        -DCMAKE_TOOLCHAIN_FILE=${ARDUINO_TOOLCHAIN}
    COMMAND make
  )

  # Copy generated firmware from build space to devel space.
  add_custom_command(
    DEPENDS ${PROJECT_BINARY_DIR}/${FIRMWARE}/${FIRMWARE}.hex
    OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/${FIRMWARE}.hex
    COMMAND mkdir -p ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}
    COMMAND cp ${PROJECT_BINARY_DIR}/${FIRMWARE}/${FIRMWARE}.hex
        ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/${FIRMWARE}.hex
  )

  add_custom_target(${PROJECT_NAME}_${FIRMWARE} ALL DEPENDS 
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/${FIRMWARE}.hex
  )
  add_dependencies(${PROJECT_NAME}_${FIRMWARE} ${PROJECT_NAME}_${FIRMWARE}_ros_lib)

  # Generate a target for the upload command, so that is accessible from catkin_make.
  add_custom_target(
    ${PROJECT_NAME}_${FIRMWARE}_upload
    COMMAND rosrun rosserial_arduino_cmake upload ${PROJECT_NAME} ${FIRMWARE}.hex
  )
  add_dependencies(${PROJECT_NAME}_${FIRMWARE}_upload ${PROJECT_NAME}_${FIRMWARE})

  install(
    FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/${FIRMWARE}.hex
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

endfunction()

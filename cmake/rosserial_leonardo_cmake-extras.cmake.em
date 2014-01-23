cmake_minimum_required(VERSION 2.8.3)

# See: http://answers.ros.org/question/104916/catkin_find-functionality-at-build-time/?answer=104936#post-id-104936
# TODO: Handle the uploader target/script the same way?
@[if DEVELSPACE]@
set(ROSSERIAL_ARDUINO_TOOLCHAIN "@(CMAKE_CURRENT_SOURCE_DIR)/arduino-cmake/ArduinoToolchain.cmake")
set(ROSSERIAL_ARDUINO_SDK_PATH "${CATKIN_DEVEL_PREFIX}/share/rosserial_leonardo_cmake/arduino-1.0.5")
@[else]@
set(ROSSERIAL_ARDUINO_TOOLCHAIN "${rosserial_leonardo_cmake_DIR}/../../../@(CATKIN_PACKAGE_SHARE_DESTINATION)/arduino-cmake/ArduinoToolchain.cmake")
set(ROSSERIAL_ARDUINO_SDK_PATH "${rosserial_leonardo_cmake_DIR}/../../../@(CATKIN_PACKAGE_SHARE_DESTINATION)/arduino-1.0.5")
@[end if]@

set(ROSSERIAL_ARDUINO_MAKE_LIBRARIES "${rosserial_arduino_DIR}/../make_libraries.py")
message("Location of Arduino SDK: ${ROSSERIAL_ARDUINO_SDK_PATH}")

set(ARDUINO_VERSION "1.0.5")
if( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(ARDUINO_DOWNLOAD "arduino-${ARDUINO_VERSION}-linux64.tgz")
else( CMAKE_SIZEOF_VOID_P EQUAL 8 )
  set(ARDUINO_DOWNLOAD "arduino-${ARDUINO_VERSION}-linux32.tgz")
endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

function(rosserial_leonardo_firmware FIRMWARE)
  file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/${FIRMWARE})

  # Generate the rosserial messages and ros_lib code
  add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/${FIRMWARE}/ros_lib
    COMMAND ${ROSSERIAL_ARDUINO_MAKE_LIBRARIES} ${PROJECT_BINARY_DIR}/${FIRMWARE}
    COMMAND cp ${PROJECT_SOURCE_DIR}/${FIRMWARE}/ros_lib/* ${PROJECT_BINARY_DIR}/${FIRMWARE}/ros_lib/
  )

  # Create a target for the ros_lib generation, since we need to depend on specific
  # message packages being built for that.
  add_custom_target(${PROJECT_NAME}_${FIRMWARE}_ros_lib DEPENDS ${PROJECT_BINARY_DIR}/${FIRMWARE}/ros_lib)
  add_dependencies(${PROJECT_NAME}_${FIRMWARE}_ros_lib rosserial_msgs_genpy std_msgs_genpy)

  # Generate a call to CMake inside the firmware subdirectory (firmware itself is generated outside of catkin).
  add_custom_command(
    OUTPUT ${PROJECT_BINARY_DIR}/${FIRMWARE}/CMakeCache.txt
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${FIRMWARE}
    COMMAND ${CMAKE_COMMAND} ${PROJECT_SOURCE_DIR}/${FIRMWARE}
      -DCMAKE_TOOLCHAIN_FILE=${ROSSERIAL_ARDUINO_TOOLCHAIN}
      -DARDUINO_SDK_PATH=${ROSSERIAL_ARDUINO_SDK_PATH}
  )
  add_custom_target(${PROJECT_NAME}_${FIRMWARE} ALL make
    DEPENDS ${PROJECT_BINARY_DIR}/${FIRMWARE}/CMakeCache.txt
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/${FIRMWARE}
    # Unfortunately, this manual copy is necessary, as the Arduino toolchain doesn't appear to
    # respect the EXECUTABLE_OUTPUT_PATH variable.
    COMMAND ${CMAKE_COMMAND} -E copy ${PROJECT_BINARY_DIR}/${FIRMWARE}/${FIRMWARE}.hex 
                                     ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/
  )
  add_dependencies(${PROJECT_NAME}_${FIRMWARE} ${PROJECT_NAME}_${FIRMWARE}_ros_lib rosserial_leonardo_cmake)

  # Generate a target for the upload command, so that is accessible from catkin_make.
  add_custom_target(
    ${PROJECT_NAME}_${FIRMWARE}_upload
    COMMAND rosrun rosserial_leonardo_cmake upload ${PROJECT_NAME} ${FIRMWARE}.hex
  )
  add_dependencies(${PROJECT_NAME}_${FIRMWARE}_upload ${PROJECT_NAME}_${FIRMWARE})

  install(
    FILES ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/${FIRMWARE}.hex
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

endfunction()

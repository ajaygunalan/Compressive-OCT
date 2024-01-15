# Install script for directory: C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/oct_msgs/srv" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/srv/Depth.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/oct_msgs/cmake" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/oct_msgs/catkin_generated/installspace/oct_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/include/oct_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "C:/Users/OCT/AppData/Roaming/MathWorks/MATLAB/R2022b/ros1/win64/venv/Scripts/python.exe" -m compileall "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/lib/site-packages/oct_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/site-packages" TYPE DIRECTORY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/lib/site-packages/oct_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/oct_msgs/catkin_generated/installspace/oct_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/oct_msgs/cmake" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/oct_msgs/catkin_generated/installspace/oct_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/oct_msgs/cmake" TYPE FILE FILES
    "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/oct_msgs/catkin_generated/installspace/oct_msgsConfig.cmake"
    "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/oct_msgs/catkin_generated/installspace/oct_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/oct_msgs" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/lib/oct_msgs_matlab.lib")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/bin/oct_msgs_matlab.dll")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/m/" TYPE DIRECTORY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/m/" FILES_MATCHING REGEX "/[^/]*\\.m$")
endif()


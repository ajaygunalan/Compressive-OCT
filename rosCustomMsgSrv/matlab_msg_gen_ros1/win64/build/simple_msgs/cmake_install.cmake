# Install script for directory: C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/simple_msgs

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_msgs/msg" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/simple_msgs/msg/Num.msg")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_msgs/srv" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/simple_msgs/srv/AddTwoInts.srv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_msgs/cmake" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/simple_msgs/catkin_generated/installspace/simple_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/include/simple_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "C:/Users/OCT/AppData/Roaming/MathWorks/MATLAB/R2022b/ros1/win64/venv/Scripts/python.exe" -m compileall "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/lib/site-packages/simple_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/site-packages" TYPE DIRECTORY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/lib/site-packages/simple_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/simple_msgs/catkin_generated/installspace/simple_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_msgs/cmake" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/simple_msgs/catkin_generated/installspace/simple_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_msgs/cmake" TYPE FILE FILES
    "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/simple_msgs/catkin_generated/installspace/simple_msgsConfig.cmake"
    "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/build/simple_msgs/catkin_generated/installspace/simple_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/simple_msgs" TYPE FILE FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/simple_msgs/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/simple_msgs/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/lib/simple_msgs_matlab.lib")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/devel/bin/simple_msgs_matlab.dll")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/m/" TYPE DIRECTORY FILES "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/simple_msgs/m/" FILES_MATCHING REGEX "/[^/]*\\.m$")
endif()


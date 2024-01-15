# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "oct_msgs: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:C:/Program Files/MATLAB/R2022b/sys/ros1/win64/ros1/share/std_msgs/cmake/../msg;-Istd_msgs:C:/Program Files/MATLAB/R2022b/sys/ros1/win64/ros1/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(oct_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/srv/Depth.srv" NAME_WE)
add_custom_target(_oct_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "oct_msgs" "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/srv/Depth.srv" "std_msgs/Float64"
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(oct_msgs
  "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/srv/Depth.srv"
  "${MSG_I_FLAGS}"
  "C:/Program Files/MATLAB/R2022b/sys/ros1/win64/ros1/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/oct_msgs
)

### Generating Module File
_generate_module_cpp(oct_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/oct_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(oct_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(oct_msgs_generate_messages oct_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/srv/Depth.srv" NAME_WE)
add_dependencies(oct_msgs_generate_messages_cpp _oct_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(oct_msgs_gencpp)
add_dependencies(oct_msgs_gencpp oct_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS oct_msgs_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(oct_msgs
  "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/srv/Depth.srv"
  "${MSG_I_FLAGS}"
  "C:/Program Files/MATLAB/R2022b/sys/ros1/win64/ros1/share/std_msgs/cmake/../msg/Float64.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/oct_msgs
)

### Generating Module File
_generate_module_py(oct_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/oct_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(oct_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(oct_msgs_generate_messages oct_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/Ajay_OCT/OCT-Guided-AutoCALM/rosCustomMsgSrv/matlab_msg_gen_ros1/win64/src/oct_msgs/srv/Depth.srv" NAME_WE)
add_dependencies(oct_msgs_generate_messages_py _oct_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(oct_msgs_genpy)
add_dependencies(oct_msgs_genpy oct_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS oct_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/oct_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/oct_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(oct_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(oct_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/oct_msgs)
  install(CODE "execute_process(COMMAND \"C:/Users/OCT/AppData/Roaming/MathWorks/MATLAB/R2022b/ros1/win64/venv/Scripts/python.exe\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/oct_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/oct_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(oct_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(oct_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()

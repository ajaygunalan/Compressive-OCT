# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "simple_msgs: 1 messages, 1 services")

set(MSG_I_FLAGS "-Isimple_msgs:C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/msg;-Istd_msgs:C:/Program Files/MATLAB/R2022b/sys/ros1/win64/ros1/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(simple_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/msg/Num.msg" NAME_WE)
add_custom_target(_simple_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simple_msgs" "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/msg/Num.msg" ""
)

get_filename_component(_filename "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_simple_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "simple_msgs" "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(simple_msgs
  "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_msgs
)

### Generating Services
_generate_srv_cpp(simple_msgs
  "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_msgs
)

### Generating Module File
_generate_module_cpp(simple_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(simple_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(simple_msgs_generate_messages simple_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/msg/Num.msg" NAME_WE)
add_dependencies(simple_msgs_generate_messages_cpp _simple_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(simple_msgs_generate_messages_cpp _simple_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simple_msgs_gencpp)
add_dependencies(simple_msgs_gencpp simple_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simple_msgs_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(simple_msgs
  "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_msgs
)

### Generating Services
_generate_srv_py(simple_msgs
  "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_msgs
)

### Generating Module File
_generate_module_py(simple_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(simple_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(simple_msgs_generate_messages simple_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/msg/Num.msg" NAME_WE)
add_dependencies(simple_msgs_generate_messages_py _simple_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "C:/Ajay_OCT/OCTAssistedSurgicalLaserbot/rosCustomMessages/matlab_msg_gen_ros1/win64/src/simple_msgs/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(simple_msgs_generate_messages_py _simple_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(simple_msgs_genpy)
add_dependencies(simple_msgs_genpy simple_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS simple_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/simple_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(simple_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_msgs)
  install(CODE "execute_process(COMMAND \"C:/Users/OCT/AppData/Roaming/MathWorks/MATLAB/R2022b/ros1/win64/venv/Scripts/python.exe\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/simple_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(simple_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()

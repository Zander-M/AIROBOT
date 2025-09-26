# Install script for directory: /Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/src/googletest/googlemock

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/install/gmock_vendor")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gmock_vendor/cmake" TYPE FILE FILES "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/src/googletest/googlemock/gmock_vendorConfig.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gmock_vendor" TYPE FILE FILES "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/src/googletest/googlemock/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/gmock_vendor" TYPE FILE RENAME "CMakeLists.txt" FILES "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/src/googletest/googlemock/CMakeLists.txt.install")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/gmock_vendor/include" TYPE DIRECTORY FILES "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/src/googletest/googlemock/include/gmock")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/src/gmock_vendor" TYPE DIRECTORY FILES "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/src/googletest/googlemock/src")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/build/gmock_vendor/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_dev/build/gmock_vendor/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()

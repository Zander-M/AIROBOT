# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/src/micro-CDR")
  file(MAKE_DIRECTORY "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/src/micro-CDR")
endif()
file(MAKE_DIRECTORY
  "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/build/microcdr"
  "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/build/microcdr/ucdr-prefix"
  "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/build/microcdr/ucdr-prefix/tmp"
  "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/build/microcdr/ucdr-prefix/src/ucdr-stamp"
  "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/build/microcdr/ucdr-prefix/src"
  "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/build/microcdr/ucdr-prefix/src/ucdr-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/build/microcdr/ucdr-prefix/src/ucdr-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/zdrrrm/Desktop/Projects/AIROBOT/Software/firmware/components/micro_ros_espidf_component/micro_ros_src/build/microcdr/ucdr-prefix/src/ucdr-stamp${cfgdir}") # cfgdir has leading slash
endif()

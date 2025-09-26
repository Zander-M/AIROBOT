option(BUILD_ETHERNET_INTERFACE "Build ethernet interface support" OFF)
option(BUILD_WLAN_INTERFACE "Build wlan interface support" ON)

set(COMPONENT_SRC "")

if(BUILD_ETHERNET_INTERFACE)
    list(APPEND COMPONENT_SRC "network_interfaces/uros_ethernet_netif.c")
endif()

if(BUILD_WLAN_INTERFACE)
    list(APPEND COMPONENT_SRC "network_interfaces/uros_wlan_netif.c")
endif()

idf_component_register(
    SRCS ${COMPONENT_SRC}
    INCLUDE_DIRS
        "include"
        "include_override"
        "network_interfaces"
    REQUIRES
        nvs_flash esp_wifi esp_eth lwip
)

# Register the precompiled Micro-ROS core library
add_prebuilt_library(libmicroros-prebuilt
    "${COMPONENT_DIR}/lib/libmicroros.a"
    REQUIRES lwip
)

target_include_directories(libmicroros-prebuilt INTERFACE
    "${COMPONENT_DIR}/include"
)

# Forward includes of all message subfolders (e.g., geometry_msgs/, std_msgs/)
file(GLOB include_folders LIST_DIRECTORIES true CONFIGURE_DEPENDS ${COMPONENT_DIR}/include/*)
foreach(include_folder ${include_folders})
    if(IS_DIRECTORY ${include_folder})
        target_include_directories(libmicroros-prebuilt INTERFACE ${include_folder})
    endif()
endforeach()

# Link into the component
target_link_libraries(${COMPONENT_LIB} INTERFACE libmicroros-prebuilt)


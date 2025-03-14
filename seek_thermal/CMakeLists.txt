cmake_minimum_required(VERSION 3.0)
project(seek_thermal)

# libusb-1.0-dev
find_library(_USB_LIB usb-1.0)
find_path(_USB_INCLUDE NAMES libusb.h PATH_SUFFIXES libusb-1.0)

if(_USB_LIB AND _USB_INCLUDE)
  add_library(Usb INTERFACE)
  target_link_libraries(Usb INTERFACE ${_USB_LIB})
  target_include_directories(Usb INTERFACE ${_USB_INCLUDE})
else()
  message(FATAL_ERROR "Could not find libusb-1.0 development libraries")
endif()

# OpenCV
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(seek_thermal SHARED
  src/SeekCam.cpp
  src/SeekDevice.cpp
  src/SeekThermal.cpp
  src/SeekThermalPro.cpp
  src/SeekLogging.cpp
)

target_include_directories(seek_thermal
  PRIVATE
    include/seek_thermal
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(seek_thermal PUBLIC Usb ${OpenCV_LIBS})

target_compile_options(seek_thermal PRIVATE
  -fpic -fPIC
)

install(
  DIRECTORY include/seek_thermal
  DESTINATION include
)
 
install(
  TARGETS seek_thermal
  EXPORT seek_thermal
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

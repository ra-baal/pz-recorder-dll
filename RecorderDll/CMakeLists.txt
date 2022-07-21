# # CMake
# - Jako miejsce budowania wybrać katalog głowny, a nie podkatalog build?
# - Trzeba mieć zainstalowaną wersję Visual Studio, na którą będzie tworzona solucja.
#
# W przypadku błędu "could not find any instance of Visual Studio"
# należy do Visual Studio doinstalować "Desktop development with C++".


cmake_minimum_required( VERSION 2.8.12 )
set( CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" ${CMAKE_MODULE_PATH} )

project( RecorderDll )

# Tutaj dodawać wszystkie pliki.
add_executable(
	RecorderDll 
	./source/main.cpp
	./source/functions.h
	./source/functions.cpp
)


set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "RecorderDll" )

# Find Packages
find_package( PCL 1.8 REQUIRED )
find_package( KinectSDK2 REQUIRED )

if( PCL_FOUND AND KinectSDK2_FOUND )
  # Additional Include Directories
  include_directories( ${PCL_INCLUDE_DIRS} )
  include_directories( ${KinectSDK2_INCLUDE_DIRS} )

  # Preprocessor Definitions
  add_definitions( ${PCL_DEFINITIONS} )

  # Additional Library Directories
  link_directories( ${PCL_LIBRARY_DIRS} )
  link_directories( ${KinectSDK2_LIBRARY_DIRS} )

  # Additional Dependencies
  target_link_libraries( RecorderDll ${PCL_LIBRARIES} )
  target_link_libraries( RecorderDll ${KinectSDK2_LIBRARIES} )
endif()

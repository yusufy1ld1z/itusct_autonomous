# Install script for directory: /home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/src/sct_img_filter

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/install/sct_img_filter")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_publisher")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_publisher"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter" TYPE EXECUTABLE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/img_publisher")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_publisher" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_publisher")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_publisher"
         OLD_RPATH "/usr/local/cuda-11.8/lib64:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_publisher")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_filterer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_filterer")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_filterer"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter" TYPE EXECUTABLE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/img_filterer")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_filterer" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_filterer")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_filterer"
         OLD_RPATH "/usr/local/cuda-11.8/lib64:/opt/ros/humble/lib/x86_64-linux-gnu:/opt/ros/humble/lib:/usr/local/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/sct_img_filter/img_filterer")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter" TYPE DIRECTORY FILES
    "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/src/sct_img_filter/launch"
    "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/src/sct_img_filter/config"
    "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/src/sct_img_filter/data"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/sct_img_filter")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/sct_img_filter")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter/environment" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter/environment" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_index/share/ament_index/resource_index/packages/sct_img_filter")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter/cmake" TYPE FILE FILES
    "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_core/sct_img_filterConfig.cmake"
    "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/ament_cmake_core/sct_img_filterConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/sct_img_filter" TYPE FILE FILES "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/src/sct_img_filter/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/yildiz/Desktop/forgit/itusct_autonomous/itusct_autonomous_weekly7/weekly7_ws/build/sct_img_filter/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

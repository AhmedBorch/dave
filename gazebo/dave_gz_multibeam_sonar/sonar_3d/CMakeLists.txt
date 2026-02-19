cmake_minimum_required(VERSION 3.8)
project(sonar_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-rendering8 REQUIRED OPTIONAL_COMPONENTS ogre ogre2)
find_package(gz-sim8 REQUIRED)
find_package(gz-sensors8 REQUIRED)

set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})
set(GZ_RENDERING_VER ${gz-rendering8_VERSION_MAJOR})
set(GZ_SENSORS_VER ${gz-sensors8_VERSION_MAJOR})
add_subdirectory(../multibeam_sonar ${CMAKE_BINARY_DIR}/multibeam_sonar)
add_library(${PROJECT_NAME} SHARED src/Sonar3D.cc)
target_link_libraries(${PROJECT_NAME}
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
  multibeam_sonar
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ../multibeam_sonar
)

ament_target_dependencies(${PROJECT_NAME}
    gz-common${GZ_COMMON_VER}
    ${GZ_RENDERING_TARGET}
    gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}
    gz-sensors${GZ_SENSORS_VER}
  )

# Install targets
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

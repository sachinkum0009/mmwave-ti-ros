cmake_minimum_required(VERSION 3.8)
project(radar2scan LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenMP REQUIRED)

# Optional CUDA support
option(ENABLE_CUDA "Enable CUDA GPU acceleration" OFF)

if(ENABLE_CUDA)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    set(USE_CUDA TRUE)
    add_definitions(-DUSE_CUDA)
    message(STATUS "CUDA found - GPU acceleration enabled")
    
    # Set CUDA architecture (auto-detect or use common architectures)
    if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
      set(CMAKE_CUDA_ARCHITECTURES "native")
    endif()
    
    # Add CUDA source
    set(CUDA_SOURCES src/radar_cuda_kernel.cu)
  else()
    set(USE_CUDA FALSE)
    set(CUDA_SOURCES "")
    message(STATUS "CUDA not found - CPU-only build")
  endif()
else()
  set(USE_CUDA FALSE)
  set(CUDA_SOURCES "")
  message(STATUS "CUDA disabled - using optimized CPU (OpenMP) build")
endif()

include_directories(include)

# Create executable with or without CUDA
add_executable(radar2scan_node 
  src/radar_processing.cpp
  ${CUDA_SOURCES}
)

ament_target_dependencies(radar2scan_node rclcpp sensor_msgs std_msgs)

# Link OpenMP
if(OpenMP_CXX_FOUND)
  target_link_libraries(radar2scan_node OpenMP::OpenMP_CXX)
endif()

# Link CUDA if available
if(USE_CUDA)
  # Set CUDA standard and properties
  set_target_properties(radar2scan_node PROPERTIES 
    CUDA_SEPARABLE_COMPILATION OFF
    CUDA_STANDARD 14
    POSITION_INDEPENDENT_CODE ON
  )
  
  # Link CUDA runtime
  target_link_libraries(radar2scan_node CUDA::cudart)
  
  # Set CUDA flags to avoid static library linking issues
  set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -Xcompiler -fPIC")
endif()
install(TARGETS
  radar2scan_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
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

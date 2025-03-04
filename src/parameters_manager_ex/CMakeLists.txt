cmake_minimum_required(VERSION 3.8)
project(parameters_manager_ex)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_parse_2_param tests/test_yaml.py)
endif()

if(DEFINED INCLUDE_CPACK)
  include(${INCLUDE_CPACK})
endif()
ament_package()

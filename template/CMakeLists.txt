cmake_minimum_required(VERSION 3.5)
project(@PROJECT@)

# These are needed only if the project defines new ROS types
# or contains C/C++ components
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies.  ament_cmake and rclswi are always required
find_package(ament_cmake REQUIRED)
find_package(rclswi REQUIRED)
# rosidl_default_generators is needed if the project defines new ROS types
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
#  "srv/Command.srv"
#  "msg/Event.msg"
)

install(FILES prolog/rclswi_setup.pl
	      prolog/main.pl
#	      prolog/squirrel_teleop.pl
#	      prolog/squirrel_client.pl
  DESTINATION prolog/${PROJECT_NAME}
)
install(PROGRAMS script/run
     DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# Some rclswi examples

This directory contains several example that   use rclswi. An example is
typically loaded from our ROS2 workspace using

    swipl -p library=install/rclswi/prolog src/rclswi/examples/<file.pl>

Below is a list of the examples.   Some of these illustrate a particular
part of the ROS interface, some are small  tests and some have been used
for performance analysis.

  - constants.pl
    Shows how constants from ROS2 message files can be accessed in
    applications.
  - demo.pl
    Some elementary demos.  See source code for comments and how to use them.
  - doc.pl
    Load the libary and start an HTTP server to view the documentations.
    Surf to this place to get an overview of the docs.

      - http://localhost:8080/pldoc/doc/_CWD_/install/rclswi/prolog/ros.pl
  - logging.pl
    Shows interaction with the ROS2 logging primitives.
  - multi_array.pl
    Illustrates handling of somewhat more complicated messages.
  - num3.pl
    Very simple test and demo for dealing with a fixed-length array.
  - params.pl
    Show node parameter handling
  - pingpong.pl
    Ping pong messages between two nodes, either in the same process or
    in two processes.
  - add_two_ints.pl
    Classical ROS2 tutorial adding two integers.  Both client and server.
  - std_msgs.pl
    Tests round trip of all basic message types.
  - times.pl
    Demos fixed and dynamic arrays of (Time) structures.
  - turtlesim_node.pl
    Implements an headless version of the action protocol for turtlesim.
  - turtlesim_teleop.pl
    Prolog implementation of the turtlesim driver, providing messages,
    actions and parameters.

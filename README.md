# rclswi: a SWI-Prolog client for ROS2

This repo provides an _RCL_ (_ROS  Client   Library_)  that allows for a
Prolog process to create one or more nodes  in a ROS network. The aim of
this library is to support demanding   applications. The client exploits
SWI-Prolog's scalable multi-threading capabilities that lets you use all
cores. The file EVAL.md is a very   crude evaluation that indicates that
the performance is  significantly  better   than  `rclpy`,  the standard
Python client library for ROS.


__WARNING__

> This repo provides a first version of  the SWI-Prolog ROS2 bridge. Not
> all functionality is provided and most of   the  interface is not well
> tested. Still, the current version should be enough for evaluation and
> prototyping.

## Building

 - Make sure to have SWI-Prolog installed (8.3.22 or later) and accessible
   in $PATH as `swipl`.
 - clone into `src` below your ROS2 workspace
 - Build using `colcon build`

## Platforms

 - Currently the library has only been tested using the ROS2 __foxy__
   distribution on Linux (Ubuntu 20.04, AMD64).
 - Most likely it will run on any Linux platform, both 32 and 64
   bits.
 - It may work on most other POSIX like systems.  It will not (yet)
   work on Windows.
 - Some of the RCL/RMW ROS2 core library functions have changed since
   _foxy_, so running on the latest ROS2 will require a few (minor)
   changes.

## Examples

The directory `rclswi/examples` contains several small test programs and
examples. To run any  of  these,   from  your  ROS2  workspace, assuming
default directory structure

    swipl -p library=install/rclswi/prolog src/rclswi/examples/demo.pl

See examples/README.md for an overview and   the  content of the various
files for details.

## Documentation

The library is documented using PlDoc. To   view  the documentation in a
browser  run  the  command  below   to    create   an   HTTP  server  at
http://localhost:8080.

    swipl -p library=install/rclswi/prolog src/rclswi/examples/doc.pl [option ...]

Options processed are ``--port=Port``  to   select  an  alternative port
(default 8080) and ``--no-doc`` to just load the code without starting a
web server.

Use the directory menu on  the   top-left  to select the `rclswi/prolog`
directory and select `ros.pl`.

## Feedback

For comments and questions, please use  either the [SWI-Prolog Discourse
forum](https://swi-prolog.discourse.group/)  or  [ROS    questions   and
answers](https://answers.ros.org/questions/).  Bug  fixes   and  feature
requests can use the issue tracker of the GitHub repository.

All contributions are welcome: example   code, fixes, better interfaces,
ports, etc.  Please provide as Git pull requests if possible.


## Status

The listb below is lists parts  of   the  RCL  functionality and what is
supported by the current version of rclswi.

  - [ ] Type handling
    - [x] Convert message from C to Prolog
      - [x] Deal with arrays
        - [x] Fixed arrays
	- [x] Dynamic arrays
      - [ ] Deal with wide strings (are these UTF-16?)
    - [x] Convert message from Prolog to C
      See https://github.com/osrf/dynamic_message_introspection
    - [ ] At the moment dict key names are created on the fly from the
      type introspection data.  This is needlessly slow.
    - [ ] Should we allow the user to tweak the conversion by
      providing dict tags and keys and choices for text conversion
      (atom/string)?
  - [ ] Object live-time handling, i.e., make nodes depend on objects
        associated with the node handle.
  - [x] Parameter handling
    - [x] Declare node parameters
    - [x] Populate declared parameters from arguments and launch data
    - [x] Change parameters
    - [x] Publish changes on `/parameter_events`
    - [x] Provide a callback on parameter changes
    - [x] Enumerate, get and set parameters on named nodes
  - [x] Graph tracking
    - [x] Query the ROS node graph
      - [x] Enumerate nodes
      - [x] Enumerate topics and types
      - [x] Enumerate services and types
      - [x] Enumerate actions and types
  - [x] Deal with logging
    - [x] configure and generate ROS log messages
    - [x] Bridge from SWI-Prolog `print_message/2` and `debug/3`
    - [x] print_message/2 is only forwarded for non-interactive sessions.
    - [ ] Defaults should depend on whether a session is interactive
  - [ ] Deal with services
    - [x] Low level interface: read/write request/response, wait for
	  client and service.
    - [ ] High level interface.
      - [ ] Client
        - [x] Synchronous calls
        - [ ] ASynchronous calls
      - [x] Service
        - [x] Read/compute/reply loop
        - [x] Callback based
  - [ ] Deal with actions
    - [x] Action type introspection
    - [x] Low level interface for creating an action client or server
    - [x] Receiving feedback and status
    - [x] Access to the goals, results and cancel services
    - [x] QoS support
    - [ ] High level action API
      - [x] DCG based action client
      - [ ] Callback based client
      - [x] action server
  - [ ] Deal with clocks
    - [x] Basics for creating a clock and asking its time
  - [ ] Deal with timers
  - [x] Deal with guard conditions
  - [ ] Advanced configuration
    - [x] Provide access to the QoS policies
  - [ ] ROS2 Lifecycle support?
  - [ ] Integration into ROS deployment
    - [ ] Build, portability
      - [ ] Find the right SWI-Prolog
      - [ ] Use `rclutils` to get some portable alternatives
    - [ ] Allow running a ROS node interactively from Prolog
      -	[ ] Make installation follow the SWI-Prolog _pack_
            structure so we can attach the ROS API as a pack.
    - [ ] Allow deploying a ROS node using `ros2 run`
      - [ ] How to do that?
      - [x] Pass command line options into `rcl_init()` and `rcl_node_init()`
      - [ ] Provide a skeleton project
    - [x] Query the `share` directory of a package
  - [ ] Process cleanup
    - [x] Properly handle signals asynchronously (Control-C)
    - [ ] Finish and reclaim resources in the right order.
  - [ ] Documentation
    - [x] In code documentation
    - [x] Local HTTP server for documentation (see above)
    - [ ] Generate a stand-alone document
  - [ ] Testing
  - [ ] Performance evaluation

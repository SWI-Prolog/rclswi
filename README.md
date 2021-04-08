# rclswi: a SWI-Prolog client for ROS2

__WARNING__

> This is work in progress. The current library is a fairly complete
> ROS2 client library.  The interfaces are not completely settled.

## Building

 - Make sure to have SWI-Prolog installed (8.3.22 or later)
 - clone into `src` below your ROS2 workspace
 - Build using `colcon build`

## Running the demos

The directory `rclswi/examples` contains several small test programs and
examples. To run any  of  these,   from  your  ROS2  workspace, assuming
default directory structure

    swipl -p library=install/rclswi/prolog src/rclswi/examples/demo.pl

See the file content for details.

## Documentation

The library is documented using PlDoc. To   view  the documentation in a
browser run (same as above, adding ``--pldoc``).

    swipl --pldoc -p library=install/rclswi/prolog src/rclswi/examples/all.pl

Use the directory menu on  the   top-left  to select the `rclswi/prolog`
directory or one of its subdirectories.


## Design

The `rclswi` package consists of a   C-based  wrapper between Prolog and
the ROS2 `rcl` and  `rmw`  libraries  that   deal  with  managing  a ROS
context, nodes, subscriptions, etc. The  wrapper provides an abstraction
level that is  comparable  to  `rclpy`.   Pointers  to  ROS  objects are
represented                        as                         SWI-Prolog
[blobs](https://www.swi-prolog.org/pldoc/man?section=blob) with a  small
layer on top of them  that  provide   functionality  that  is similar to
Python _Capsules_.   This results in e.g.

    ?- ros_default_context(X).
    X = <ros_context>(0x560482bf2180).

These pointers are subject to  (atom)   garbage  collection.  Whenever a
pointer is garbage collected a C  finialization function is called. This
layer allows for creating contexts, nodes, subscriptions, etc.

### Messages and message types

The `rclswi` package uses _type  introspection_ to dynamically translate
between  the  C   representation   used    by   e.g.   `rcl_take()`  and
`rcl_publish()` and the Prolog representation. Given a message type, the
Prolog interface compute the names of  the involved shared libraries and
the functions that provide access to the type support structure required
to subscribe or publish to a topic,  the functions to create and destroy
messages of the  requested  type  and   functions  to  access  the  type
introspection information.  For example:

    ?- ros:ros_type_support('geometry_msgs/msg/Twist', TypeSupport).
    TypeSupport = <ros_message_type>(0x55886a3fa3c0).

The message information is need for registering with topics and services
and stored along with the subscription and similar object. This provides
the simple interface:

    ros_take(+Subscription, -Message, -MessageInfo)

The interface provides a  systematic  translation   from  the  ROS  type
introspection data to Prolog. Typically, ROS   names use _CamelCase_. As
this conflicts with Prolog variable  naming   as  consistent  mapping is
applied. The mapping is available   through ros_identifier_prolog/2 from
library(ros/types).

The mapping from ROS types to Prolog types is defined as:

  - ``ROS_TYPE_MESSAGE`` is translated into a SWI-Prolog dict.  Type and
    field names are translated according to the above notice.
  - All ROS floating point numbers are represented as SWI-Prolog floats,
    which currently maps to a C double.  This implies that `long double`
    looses precision.
  - All ROS integral types are conversion to Prolog integers.
  - ``ROS_TYPE_CHAR`` and ``ROS_TYPE_WCHAR`` are converted into a one
    character atom.
  - ``ROS_TYPE_BOOLEAN`` is converted into `true` or `false`.
  - ``ROS_TYPE_STRING`` is converted into a SWI-Prolog string.

__Type introspection__ allows us to describe a type in Prolog.  The following
is available from library(ros/types):

    ?- ros_type_introspection('geometry_msgs/msg/Twist', D),
       print_term(D, []).
    twist{ angular:vector3{x:double,y:double,z:double},
           linear:vector3{x:double,y:double,z:double}
         }
    D = ...


### Spinning

At the lowest level, spinning is based  on ros_wait/3 which takes a list
of waitable objects and waits, optionally  with   a  timeout, for any of
these objects to become ready, returning a list of ready objects.


## High level API

Most applications won't have need for managing multiple ROS contexts and
ROS nodes in the same process.  The high level API provides:

  - Declare ROS context and node properties that are used to lazily
    create the default context and default node.
  - Register pub/sub on topics, etc., associating these with a callback.
  - Spin

For example, to monitor requests for the   ROS  _Turtle_ demo we need to
write:

```
:- ros_set_defaults(
       [ node(prolog, [])
       ]).

turtle_listener :-
    ros_subscribe('/turtle1/cmd_vel', on_turtle,
                  [ message_type('geometry_msgs/msg/Twist')
                  ]).

on_turtle(Message) :-
    format('Turtle asked to ~p~n', [Message]).

main :-
    turtle_listener,
    ros_spin.
```


## TODO

The current version is a proof of   concept.  It demonstrates we can use
`rcl` and `rmw` to access ROS2 and   convert  messages into Prolog using
type introspection.  Missing:

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
    - [ ] Generate a stand-alone document
  - [ ] Testing
  - [ ] Performance evaluation

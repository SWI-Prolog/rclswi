# Design

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

## Messages and message types

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


## Spinning

At the lowest level, spinning is based  on ros_wait/3 which takes a list
of waitable objects and waits, optionally  with   a  timeout, for any of
these objects to become ready, returning a list of ready objects.


# High level API

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



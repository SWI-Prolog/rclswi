/*  This example code is public domain.
*/

:- module(num3,
          [ num3/3,
            listen/0
          ]).

/** <module> Test fixed arrays

Very simple test dealing with user defined  types from the ROS2 tutorial
on interfaces.  `Num3.msg` holds:

```
int64[3] num
```
*/

:- use_module(library(ros)).

num3(A,B,C) :-
    ros_publisher('/num3',
                  [ message_type('tutorial_interfaces/msg/Num3')
                  ]),
    ros_publish('/num3',
                _{num:[A,B,C]}).

listen :-
    ros_subscribe('/num3', on_num,
                  [ message_type('tutorial_interfaces/msg/Num3')
                  ]),
    ros_spin([thread(spinner)]).

on_num(Msg) :-
    format('Received ~p~n', [Msg]).

/*  This example code is public domain.
*/

:- module(num3,
          [ num3/3,
            listen/0
          ]).

/** <module> Test fixed arrays

*/

:- reexport(install/rclswi/prolog/ros).

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
    thread_create(ros_spin, _, [detached(true)]).

on_num(Msg) :-
    format('Received ~p~n', [Msg]).

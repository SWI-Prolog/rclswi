/*  This example code is public domain.
*/

:- module(pingpong,
          [ pingpong/1,
            ping_node/0,
            pong_node/0
          ]).

/** <module> Performance test

This demo creates the topics `/ping` and  `/pong`. Pong simply echos any
message it received on `/pong` to  `/ping`.   The  ping node receives on
`/ping', decrements and sends on `/pong`.  When the count is decremented
to zero we send a thread message that we are done.

By default this demo creates two nodes in   the  same process. To use on
two terminals, add the `-l` flag to stop program initialization:

    swipl -p library=install/rclswi/prolog -l src/rclswi/examples/pingpong.pl

Run this in one terminal:

    ?- pong_node.

Now in the other run

    ?- ping_node.
    ?- pingpong(1000).
    % 2 inferences, 0.000 CPU in 0.046 seconds (0% CPU, 95785 Lips)

Indicating a round trip time of 46   microseconds, where each round trip
implies two publish and two receive events.
*/

:- reexport(library(ros)).

pingpong(Start) :-
    ros_publisher('/ping',
                  [ message_type('std_msgs/msg/Int64')
                  ]),
    ros_publish('/ping', _{data: Start}),
    time(thread_get_message(done)).

% Initialise both nodes at program startup.
:- initialization(ping_node, program).
:- initialization(pong_node, program).

ping_node :-
    ros_create_node(ping, Node, []),
    ros_publisher('/pong',
                  [ message_type('std_msgs/msg/Int64'),
                    node(Node)
                  ]),
    ros_subscribe('/ping', on_ping,
                  [ message_type('std_msgs/msg/Int64'),
                    node(Node)
                  ]),
    ros_spin([thread(ping), node(Node)]).

on_ping(Msg) :-
    Down is Msg.data-1,
    (   Down >= 0
    ->  ros_publish('/pong', _{data:Down})
    ;   thread_send_message(main, done)
    ).

pong_node :-
    ros_create_node(pong, Node, []),
    ros_publisher('/ping',
                  [ message_type('std_msgs/msg/Int64'),
                    node(Node)
                  ]),
    ros_subscribe('/pong', on_pong,
                  [ message_type('std_msgs/msg/Int64'),
                    node(Node)
                  ]),
    ros_spin([thread(pong), node(Node)]).

on_pong(Msg) :-
    ros_publish('/ping', Msg).

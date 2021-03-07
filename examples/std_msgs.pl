/*  This example code is public domain.
*/

:- module(std_msgs,
          [ test_std_msgs/0,
            pub/1,
            sub/1,
            pub/2,
            pub/1,
            test/1
          ]).
:- use_module(library(apply)).
:- use_module(library(debug)).
:- use_module(library(plunit)).

/** <module> Test exchange of standard message types

*/

:- reexport(install/rclswi/prolog/ros).

test_std_msgs :-
    run_tests([std_msgs]).

:- begin_tests(std_msgs).

test(float32, true) :- test(float32).
test(float64, true) :- test(float64).
test(bool,    true) :- test(bool).
test(int8,    true) :- test(int8).
test(uint8,   true) :- test(uint8).
test(int16,   true) :- test(int16).
test(uint16,  true) :- test(uint16).
test(int32,   true) :- test(int32).
test(uint32,  true) :- test(uint32).
test(int64,   true) :- test(int64).
test(uint64,  true) :- test(uint64).
test(string,  true) :- test(string).

:- end_tests(std_msgs).


trip(float32, [3.14, 2],                         ["string"]).
trip(float64, [3.14, 2],                         ["string"]).
trip(bool,    [true, false],                     [42, 1, 0]).
trip(int8,    [1,0,-42],                         [300, -200, 3.0]).
trip(uint8,   [0,1,42,255],                      [-1, 300, -200, 3.0]).
trip(int16,   [1,0,32767,-32768],                [32768, -32769, 3.0]).
trip(uint16,  [0,1,42,255,65535],                [-1, 65536, 3.0, "string"]).
trip(int32,   [1,0,-42,-2147483648],             [2147483648, -2147483649, 3.0]).
trip(uint32,  [0,1,42,255,4294967295],           [-1, -200, 3.0, 4294967296]).
trip(int64,   [1,0,-42],                         [300, -200, 3.0]).
trip(uint64,  [0,1,42,255,18446744073709551615], [-1, -200, 3.0, 18446744073709551616]).
trip(string,  ["hello", world, 42, 3.14],        [hello(world), 'text\u0000more']).

test(Type) :-
    pub_sub(Type),
    trip(Type, Good, Bad),
    maplist(test_good_trip(Type), Good),
    maplist(test_bad_trip(Type), Bad).

test_good_trip(Type, Data) :-
    thread_self(Me),
    empty_queue(Me),
    pub(Type, _{data:Data}),
    (   thread_get_message(Me, got(Reply), [timeout(10)])
    ->  ReplyData = Reply.data,
        assertion(reply_ok(ReplyData, Data))
    ;   assertion(reply)
    ).

empty_queue(Queue) :-
    thread_get_message(Queue, got(_), [timeout(0)]),
    !,
    empty_queue(Queue).
empty_queue(_).


reply :- fail.                          % to get assertion failed: reply

reply_ok(Got, Expected) :-
    Got == Expected,
    !.
reply_ok(Got, Expected) :-
    float(Got), number(Expected),
    abs(Got-Expected) < (Got+Expected)/10000.
reply_ok(Got, Expected) :-
    string(Got), atomic(Expected),
    atom_string(Expected, Got).

test_bad_trip(Type, Data) :-
    catch(pub(Type, _{data:Data}), E, true),
    E = error(Formal, _),
    debug(error, 'Trying to sent ~p as ~p: ~p',
          [ Data, Type, Formal ]),
    assertion(ok_error(Type, Data, Formal)).

ok_error(Type, Data, type_error(Type, Data)) :- !.
ok_error(_, _, type_error(_, _)).
ok_error(_, _, domain_error(_, _)).
ok_error(int8, _, representation_error(char)).
ok_error(uint8, _, representation_error(uchar)).
ok_error(int16, _, representation_error(short)).
ok_error(uint16, _, representation_error(ushort)).
ok_error(int32, _, representation_error(int)).
ok_error(uint32, _, representation_error(uint)).
ok_error(int64, _, representation_error(int64_t)).
ok_error(uint64, _, representation_error(uint64_t)).


pub(Type, Data) :-
    type_topic(Type, Topic, _MsgType),
    ros_publish(Topic, Data).

:- dynamic
    pub_sub_done/1.

pub_sub(Type) :-
    pub_sub_done(Type),
    !.
pub_sub(Type) :-
    pub(Type),
    sub(Type),
    assertz(pub_sub_done(Type)).


pub(Type) :-
    type_topic(Type, Topic, MsgType),
    ros_publisher(Topic,
                  [ message_type(MsgType)
                  ]).

sub(Type) :-
    type_topic(Type, Topic, MsgType),
    ros_subscribe(Topic, on_msg,
                  [ message_type(MsgType)
                  ]),
    thread_create(ros_spin, _, [detached(true)]).

on_msg(Msg) :-
    debug(sub, 'Received ~p~n', [Msg]),
    thread_send_message(main, got(Msg)).

type_topic(Type, Topic, MsgType) :-
    atom_concat('/example_interfaces_', Type, Topic),
    ros_identifier_prolog(RosType, Type),
    atom_concat('example_interfaces/msg/', RosType, MsgType).

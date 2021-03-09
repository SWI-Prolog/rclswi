/*  This example code is public domain.
*/

:- module(struct_sequences,
          [ pub/0,
            sub/0
          ]).

/** <module> Test fixed arrays

*/

:- reexport(library(ros)).

pub :-
    ros_publisher('/sequence',
                  [ message_type('tutorial_interfaces/msg/Seq')
                  ]).

seq(Type, Data) :-
    dict_create(Msg, _, [Type-Data]),
    ros_publish('/sequence', Msg).

int64_3(A,B,C) :-
    ros_publish('/sequence',
                _{int64_3:[A,B,C]}).

int64_seq(List) :-
    ros_publish('/sequence',
                _{int64_seq:List}).

string_seq(List) :-
    seq(string_seq, List).

sub :-
    ros_subscribe('/sequence', on_sequence,
                  [ message_type('tutorial_interfaces/msg/Seq')
                  ]),
    thread_create(ros_spin, _, [detached(true)]).

on_sequence(Msg) :-
    format('Received ~p~n', [Msg]).

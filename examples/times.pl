/*  This example code is public domain.
*/

:- module(times,
          [ pub/0,
            sub/0,
            type/0,
            pub/2,
            pub/1
          ]).
:- use_module(library(pprint)).

/** <module> Test fixed arrays

*/

:- reexport(library(ros)).

% TBD: Should not be needed

:- initialization
    ros:load_type_support(builtin_interfaces).

type :-
    ros_type_introspection('tutorial_interfaces/msg/Times', X),
    print_term(X, []).

pub :-
    ros_publisher('/times',
                  [ message_type('tutorial_interfaces/msg/Times')
                  ]).

pub(T0, T1) :-
    maplist(ros_time_prolog, Msg, [T0,T1]),
    ros_publish('/times',
                _{duration: Msg}).

pub(List) :-
    maplist(ros_time_prolog, Msg, List),
    ros_publish('/times',
                _{series: Msg}).


ros_time_prolog(time{sec:Sec, nanosec:NanoSec}, Stamp) :-
    (   nonvar(Stamp)
    ->  Sec is floor(Stamp),
        NanoSec is round(float_fractional_part(Stamp)*1000000000)
    ;   Stamp is Sec + NanoSec/1000000000.0
    ).

sub :-
    ros_subscribe('/times', on_sequence,
                  [ message_type('tutorial_interfaces/msg/Times')
                  ]),
    thread_create(ros_spin, _, [detached(true)]).

on_sequence(Msg) :-
    format('Received ~p~n', [Msg]).

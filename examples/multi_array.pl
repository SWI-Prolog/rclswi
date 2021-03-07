/*  This example code is public domain.
*/

:- module(multi_array,
          [ pub_sub/1,                  % +Type
            pub/1,                      % +Type
            sub/1,                      % +Type
            pub/3                       % +Type, +Labels, +Data
          ]).
:- use_module(library(pprint)).
:- use_module(library(ansi_term)).
:- use_module(library(apply)).
:- use_module(library(error)).
:- use_module(library(lists)).

/** <module> Test fixed arrays

This demo shows handling of arrays.  It   builds  on the ROS MultiArray,
which contains the flattened data  of   a  multi-dimensional array and a
description of the dimensions. This representation is deprecated, but to
us useful for illustrating handling of complex ROS message types.
*/

:- reexport(install/rclswi/prolog/ros).

%!  pub(+Labels, +Data) is det.
%
%   Publish a multi array from Labels and Data. Data is represented as a
%   nested Prolog list. Labels is a   list of dimension labels, starting
%   with the outermost list.  For example:
%
%       ?- pub([y,x], [[1,2,3], [2,4,5]])

pub(Type, Labels, Data) :-
    type_topic(Type, Topic, _MsgType),
    ros_multi_array_prolog(Msg, Labels, Data),
    ros_publish(Topic, Msg).

pub_sub(Type) :-
    pub(Type),
    sub(Type).

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
    (   ros_multi_array_prolog(Msg, Labels, Data)
    ->  true
    ;   format(user_error, "Failed to translated MultiArray to Prolog~n", [])
    ),
    with_output_to(user_output,
                   (   ansi_format([bold], '~NReceived:~n', []),
                       print_term(Msg, []),
                       format('~N---~n'),
                       ansi_format([bold], '~NProlog representation:~n', []),
                       format('Labels: ~p, Data: ~p~n---~n', [Labels, Data])
                   )).


type_topic(Type, Topic, MsgType) :-
    atom_concat('/multi_array_', Type, Topic),
    atom_concat(Type, '_multi_array', PlType),
    ros_identifier_prolog(RosType, PlType),
    atom_concat('std_msgs/msg/', RosType, MsgType).


%!  ros_multi_array_prolog(-Message, +Labels, +Data) is det.
%!  ros_multi_array_prolog(+Message, -Labels, -Data) is det.
%
%   Translate between a ROS2 MultiArray  and   a  Prolog nested list and
%   list of dimension labels.

ros_multi_array_prolog(Msg, Labels, Data) :-
    var(Msg),
    !,
    multi_array_dim(Labels, Data, DIM),
    flatten_data(DIM, Data, Flat),
    Msg = _{layout:_{dim:DIM}, data:Flat}.
ros_multi_array_prolog(Msg, Labels, Data) :-
    unflatten_data(Msg.layout.dim, Labels, Msg.data, Data).


%!  multi_array_dim(+Labels, +Data, -DIM) is det.

multi_array_dim(Labels, Data, DIM) :-
    multi_array_dim(Labels, Data, DIM, _Stride).

multi_array_dim([], _Data, [], 1).
multi_array_dim([L0|L], Data, [_{label:L0, size:Size, stride:Stride}|DIM], Stride) :-
    length(Data, Size),
    Data = [H|_],
    (   L \== []
    ->  check_dim(Data)
    ;   true
    ),
    multi_array_dim(L, H, DIM, Stride0),
    Stride is Stride0*Size.

check_dim(Data) :-
    maplist(length, Data, Lens),
    sort(Lens, Unique),
    (   Unique = [_]
    ->  true
    ;   type_error(multi_array, Data)
    ).

flatten_data([_], Data, Data) :-
    !.
flatten_data([_|T], Data, Flat) :-
    maplist(flatten_data(T), Data, Flat0),
    append(Flat0, Flat).

unflatten_data([D], [L], Data, Data) :-
    !,
    atom_string(L, D.label).
unflatten_data([D0,D1|DT], [L0|LT], Flat, Data) :-
    atom_string(L0, D0.label),
    chunk_list(Flat, D1.stride, Chunks),
    maplist(unflatten_data([D1|DT], LT), Chunks, Data).

chunk_list([], _, []) :-
    !.
chunk_list(Flat, Len, [H|T]) :-
    length(H, Len),
    append(H, Rest, Flat),
    chunk_list(Rest, Len, T).

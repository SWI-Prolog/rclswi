/* Copyright 2021 SWI-Prolog Solutions b.v.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http:  www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

:- module(ros_param_store,
          [ ros_parameter/2,                % +Name, +Options
            ros_delete_parameter/2,         % +Name, +Options
            ros_get_param/2,                % ?Name, -Value
            ros_get_param/3,                % ?Name, -Value, +Options
            ros_set_param/2,                % ?Name,-Value
            ros_set_param/3,                % ?Name, -Value, +Options
            ros_set_params/2,               % +Dict,+Options
            import_parameters/1,            % +Options
            ros_parameter_property/3,       % +Node, ?Param, ?Property
            ros_publish_parameter_events/1, % +Node
            ros_discard_parameter_events/1  % +Node
          ]).
:- use_module(library(ros)).
:- use_module(library(ros/detail/options)).
:- use_module(library(ros/detail/param)).
:- use_module(library(error)).
:- use_module(library(option)).
:- use_module(library(apply)).
:- use_module(library(debug)).
:- use_module(library(lists)).
:- use_module(library(broadcast)).

/** <module> Manage parameters for a node

This module implements the node parameter   declarations and storage. It
deals  with  storing  parameters,   populating    parameters   from  the
commandline and node arguments, setting   and  getting parameter values.
Changes to parameter values are published on `/parameter_events`
*/

:- dynamic
    node_param/3,                        % NodeID, Name, Value
    node_param_property/4.               % NodeID, Name, Property, Value

:- thread_local
    parameter_event/4.                   % Type, NodeID, Param, Value

:- predicate_options(ros_parameter/2, 2,
                     [ node(any),
                       default(any),
                       type(any),
                       additional_constraints(text),
                       description(text),
                       read_only(bool)
                     ]).

%!  ros_parameter(+Name, +Options) is det.
%
%   Define a parameter.  Defined options:
%
%     - node(+Node)
%       Node on which to define the parameter.
%     - default(+Value)
%       Default value.
%     - type(+Type)
%       Defined types are `bool`, `integer`, `double`, `string`,
%       `byte_array`, `bool_array`. `integer_array`, `double_array` and
%       `string_array`.  In addition the following types are accepted:
%       - integer(Low, High)
%       - integer(Low, High, Step)
%       - double(Low, High)
%       - double(Low, High, Step)
%     - additional_constraints(+Text)
%       Natural language description for additional constraints that
%       cannot be expressed in the type.
%     - description(+Text)
%       Short textual description
%     - read_only(+Bool)
%     - publish(+Bool)
%       When `false` (default `true`), do not generate a parameter
%       event.

ros_parameter(Name, Options) :-
    node_from_options(Node, Options),
    transaction(ros_parameter_(Node, Name, Options)),
    ros_publish_parameter_events(Node).

ros_parameter_(Node, Name, Options) :-
    ros_delete_param(Node, Name),
    node_id(Node, NodeID),
    assertz(node_param_property(NodeID, Name, exists, true)),
    default_and_type(NodeID, Name, Options),
    set_property(additional_constraints, string, NodeID, Name, Options),
    set_property(description, string, NodeID, Name, Options),
    set_property(read_only, string, NodeID, Name, Options),
    (   option(publish(true), Options, true)
    ->  ignore(node_param(NodeID, Name, Default)),
        assertz(parameter_event(new, NodeID, Name, Default))
    ;   true
    ).

default_and_type(Node, Name, Options) :-
    option(default(Default), Options),
    option(type(Type), Options),
    !,
    is_valid_param_type(Type),
    typecheck_parameter_value(Type, Default),
    assertz(node_param(Node, Name, Default)),
    assertz(node_param_property(Node, Name, type, Type)).
default_and_type(Node, Name, Options) :-
    option(default(Default), Options),
    !,
    value_type(Default, Type),
    assertz(node_param(Node, Name, Default)),
    assertz(node_param_property(Node, Name, type, Type)).
default_and_type(Node, Name, Options) :-
    option(type(Type), Options),
    !,
    is_valid_param_type(Type),
    assertz(node_param_property(Node, Name, type, Type)).
default_and_type(_, _, _).

set_property(Prop, Type, Node, Name, Options) :-
    Term =.. [Prop,Value0],
    option(Term, Options),
    !,
    cast_to(Type, Value0, Value),
    assertz(node_param_property(Node, Name, Prop, Value)).
set_property(_Prop, _Type, _Node, _Name, _Options).

cast_to(string, Value0, Value) =>
    atom_string(Value0, Value).
cast_to(bool, Value0, Value) =>
    must_be(bool, Value0),
    Value = Value0.

%!  ros_parameter_property(+Node, ?Param, ?Property) is nondet.
%
%   True when Property is a property of   Param on Node. Every parameter
%   has a property exists(true). Other defined  properties are below. We
%   refer to ros_parameter/2 for details.
%
%     - default(Value)
%     - type(Type)
%     - additional_constraints(String)
%     - description(Text)
%     - read_only(Bool)
%
%   This predicate is semidet if Param and the Property are nonvar.

ros_parameter_property(Node, Param, Property) :-
    node_id(Node, NodeID),
    (   compound(Property),
        Property =.. [Prop,Value]
    ->  (   atom(Param)
        ->  node_param_property(NodeID, Param, Prop, Value),
            !
        ;   node_param_property(NodeID, Param, Prop, Value)
        )
    ;   node_param_property(NodeID, Param, Prop, Value),
        Property =.. [Prop,Value]
    ).


%!  ros_delete_parameter(+Name, +Options) is det.
%
%   Delete the given parameter. Succeeds also   if  the parameter is not
%   know.

ros_delete_parameter(Name, Options) :-
    node_from_options(Node, Options),
    ros_delete_param(Node, Name),
    ros_publish_parameter_events(Node).


%!  ros_delete_param(+Node, +Name) is det.
%
%   Delete a ros parameter.

ros_delete_param(Node, Name) :-
    node_id(Node, NodeID),
    transaction(ros_delete_param_(NodeID, Name)).

ros_delete_param_(NodeID, Name) :-
    \+ node_param(NodeID, Name, _),
    \+ node_param_property(NodeID, Name, _, _),
    !.
ros_delete_param_(NodeID, Name) :-
    ignore(node_param(NodeID, Name, Value)),
    retractall(node_param(NodeID, Name, _)),
    retractall(node_param_property(NodeID, Name, _, _)),
    assertz(parameter_event(deleted, NodeID, Name, Value)).

%!  ros_get_param(?Name, -Value).
%!  ros_get_param(?Name, -Value, +Options).
%
%   True when Value is  the  current   value  associated  with  the node
%   property Name. These predicates are det  if Name is instantiated and
%   nondet of Name is unbound. If  the  value   is  not  set  it is left
%   unbound (variable).

ros_get_param(Name, Value) :-
    ros_get_param(Name, Value, []).

ros_get_param(Name, Value, Options), atom(Name) =>
    node_from_options(Node, Options),
    node_id(Node, NodeID),
    (   node_param_property(NodeID, Name, exists, true)
    ->  (   node_param(NodeID, Name, Value)
        ->  true
        ;   true
        )
    ;   existence_error(ros_parameter, Name, Node)
    ).
ros_get_param(Name, Value, Options), var(Name) =>
    node_from_options(Node, Options),
    node_id(Node, NodeID),
    node_param_property(NodeID, Name, exists, true),
    (   node_param(NodeID, Name, Value)
    ->  true
    ;   true
    ).

%!  ros_set_param(+Name, +Value) is det.
%!  ros_set_param(+Name, +Value, +Options) is det.
%!  ros_set_params(+Dict, +Options) is det.
%
%   Set parameter Name  to  Value.   The  ros_set_params/2  version sets
%   multiple parameters atomically. A  parameter  that   is  set  to its
%   current value does not produce any events.

ros_set_param(Name, Value) :-
    ros_set_param(Name, Value, []).

ros_set_param(Name, Value, Options) :-
    node_from_options(Node, Options),
    transaction(ros_set_param(Node, Name, Value, Options)),
    ros_publish_parameter_events(Node).

ros_set_params(Dict, Options) :-
    node_from_options(Node, Options),
    transaction(ros_set_params(Node, Dict, Options)),
    ros_publish_parameter_events(Node).

ros_set_params(Node, Dict, Options) :-
    forall(get_dict(Key, Dict, Value),
           ros_set_param(Node, Key, Value, Options)).


ros_set_param(Node, Name, Value, _Options), atom(Name) =>
    node_id(Node, NodeID),
    (   node_param(NodeID, Name, Value)
    ->  true
    ;   (   node_param_property(NodeID, Name, type, Type)
        ->  typecheck_parameter_value(Type, Value),
            retractall(node_param(NodeID, Name, _)),
            assertz(node_param(NodeID, Name, Value)),
            assertz(parameter_event(changed, NodeID, Name, Value))
        ;   existence_error(ros_parameter, Name, Node)
        )
    ).
ros_set_param(_, Name, _, _) =>
    type_error(atom, Name).

%!  import_parameters(+Options) is det.
%
%   Import  node  parameters  from  context    (commandline)   and  node
%   initialization options.

import_parameters(Options) :-
    node_from_options(Node, Options),
    transaction(import_parameters(Node, Options)),
    (   option(publish(true), Options, true)
    ->  ros_publish_parameter_events(Node)
    ;   ros_discard_parameter_events(Node)
    ).

import_parameters(Node, Options) :-
    import_parameters(Node, global, Options),
    import_parameters(Node, local, Options).

import_parameters(Node, Scope, Options) :-
    ros:ros_get_node_parameters(Node, Scope, Parms),
    node_id(Node, NodeID),
    maplist(import_params(NodeID, [node(Node)|Options]), Parms).

import_params(NodeID, Options, Dict) :-
    _{ node:Pattern, parameters:Pdict } :< Dict,
    matches(Pattern, NodeID, Options),
    !,
    dict_pairs(Pdict, _, Pairs),
    maplist(import_param(Options), Pairs).

%!  matches(+Pattern, +Node, +Options) is semidet.
%
%   Should do pattern matching. Not sure how these node patterns work.

matches(Node, Node, _) :- !.
matches('/**', _, _) :- !.
matches(Pattern, Node, _) :-
    atom_concat(/, Node, Pattern).

import_param(Options, Name-Value) :-
    ros_set_param(Name, Value, Options).

node_id(Node, Id), atom(Node) => Id = Node.
node_id(Node, Id) => ros_node_property(Node, name(Id)).
node_id(Node, _) => type_error(ros_node, Node).

%!  ros_publish_parameter_events(+Node) is det.
%
%   Publish pending parameter events. This predicate  has no effect when
%   called inside a transaction. This both   publishes  to the ROS topic
%   ``/parameter_events``  and  makes  the    changes   available  using
%   broadcast/1.  The broadcast message is
%
%       ros_parameter_events(Node, ListOfEvents)
%
%   Where each event is a term event(Type, Parameter, Value) and Type is
%   one of `new`, `deleted` or  `changed`.   For  example, the following
%   event indicates that the `x` parameter changed to 42.
%
%       event(changed, x, 42)

ros_publish_parameter_events(_Node) :-
    current_transaction(_),
    !.
ros_publish_parameter_events(Node) :-
    node_id(Node, NodeID),
    parameter_event(_, NodeID, _, _),
    !,
    get_time(Now),
    findall(event(Type, Param, Value),
            retract(parameter_event(Type, NodeID, Param, Value)),
            Events),
    sort(3, @>=, Events, ByParam),
    join_events(ByParam, ChangeSet),
    broadcast(ros_parameter_events(Node, ChangeSet)),
    debug(ros(parameter_events), 'Change set: ~p', [ChangeSet]),
    split_events(ChangeSet, New, Changed, Deleted),
    maplist(event_to_parameter, New, NewParams),
    maplist(event_to_parameter, Changed, ChangedParams),
    maplist(event_to_parameter, Deleted, DeletedParams),
    ros_node_property(Node, qname(QName)),
    ros_publish('/parameter_events',
                _{ stamp:Now,
                   node:QName,
                   new_parameters:NewParams,
                   changed_parameters:ChangedParams,
                   deleted_parameters:DeletedParams
                 },
                [ node(Node),
                  message_type('rcl_interfaces/msg/ParameterEvent')
                ]).
ros_publish_parameter_events(_).

%!  join_events(+EventsIn, -Events)
%
%   Merge events on the same parameter into a single change.

join_events([], []).
join_events([H|T], [E0|ET]) :-
    event_param(H, Param),
    on_same_param(T, Param, More, T1),
    merge_events([H|More], E0),
    join_events(T1, ET).

on_same_param([H|T], P, [H|TM], R) :-
    event_param(H, P),
    !,
    on_same_param(T, P, TM, R).
on_same_param(List, _, [], List).

merge_events([One], One) :-
    !.
merge_events(Events, event(Type, Param, Value)) :-
    final_value(Events, Value),
    !,
    Events = [event(FirstType, Param, _)|_],
    (   FirstType == new
    ->  Type = new
    ;   Type = changed
    ).
merge_events([event(FirstType, NodeID, Param, Value)|_], event(Type, NodeID, Param, Value)) :-
    (   FirstType == new
    ->  Type = none
    ;   Type = deleted
    ).

final_value(Events, Value) :-
    last(Events, event(Type, _, _, Value)),
    nonvar(Value),
    Type \== deleted.

event_param(event(_,Param,_), Param).
event_type(event(Type,_,_), Type).

%!  split_events(ChangeSet, New, Changed, Deleted).

split_events([], [], [], []).
split_events([H|T], New, Changed, Deleted) :-
    event_type(H, Type),
    (   Type == new
    ->  New = [H|New2],
        split_events(T, New2, Changed, Deleted)
    ;   Type == changed
    ->  Changed = [H|Changed2],
        split_events(T, New, Changed2, Deleted)
    ;   Type == deleted
    ->  Deleted = [H|Deleted2],
        split_events(T, New, Changed, Deleted2)
    ;   assertion(Type == none)
    ->  split_events(T, New, Changed, Deleted)
    ).

event_to_parameter(event(_, Param, Value),
                   _{name:Param, value:Variant}) :-
    variant_value(Variant, Value).

%!  ros_discard_parameter_events(+Node)
%
%   Discard queued parameter events for Node.

ros_discard_parameter_events(Node) :-
    node_id(Node, NodeID),
    retractall(parameter_event(_, NodeID, _, _)).

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
          [ ros_parameter/2,             % +Name, +Options
            ros_get_param/2,             % ?Name, -Value
            ros_get_param/3,             % ?Name, -Value, +Options
            ros_set_param/2,             % ?Name,-Value
            ros_set_param/3,             % ?Name, -Value, +Options
            import_parameters/1,         % +Options
            ros_parameter_property/3     % +Node, ?Param, ?Property
          ]).
:- use_module(library(ros)).
:- use_module(library(ros/detail/options)).
:- use_module(library(ros/detail/param)).
:- use_module(library(error)).
:- use_module(library(option)).

/** <module> Manage parameters for a node

This module implements the node parameter   declarations and storage. It
deals  with  storing  parameters,   populating    parameters   from  the
commandline and node arguments, setting   and  getting parameter values.
Changes to parameter values are published on `/parameter_events`
*/

:- dynamic
    node_param/3,                        % Node, Name, Value
    node_param_property/4.               % Node, Name, Property, Value

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

ros_parameter(Name, Options) :-
    transaction(ros_parameter_(Name, Options)).

ros_parameter_(Name, Options) :-
    node_from_options(Node, Options),
    ros_delete_param(Node, Name),
    node_id(Node, NodeID),
    assertz(node_param_property(NodeID, Name, exists, true)),
    default_and_type(NodeID, Name, Options),
    set_property(additional_constraints, string, NodeID, Name, Options),
    set_property(description, string, NodeID, Name, Options),
    set_property(read_only, string, NodeID, Name, Options).

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
    retractall(node_param(NodeID, Name, _)),
    retractall(node_param_property(NodeID, Name, _, _)).

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

%!  ros_set_param(?Name, -Value) is det.
%!  ros_set_param(?Name, -Value, +Options) is det.
%
%   Set parameter Name to Value.

ros_set_param(Name, Value) :-
    ros_set_param(Name, Value, []).

ros_set_param(Name, Value, Options) :-
    transaction(ros_set_param_(Name, Value, Options)).

ros_set_param_(Name, Value, Options), atom(Name) =>
    node_from_options(Node, Options),
    node_id(Node, NodeID),
    (   node_param_property(NodeID, Name, type, Type)
    ->  typecheck_parameter_value(Type, Value),
        retractall(node_param(NodeID, Name, _)),
        assertz(node_param(NodeID, Name, Value))
    ;   existence_error(ros_parameter, Name, Node)
    ).
ros_set_param_(Name, _, _) =>
    type_error(atom, Name).

%!  import_parameters(+Options) is det.
%
%   Import  node  parameters  from  context    (commandline)   and  node
%   initialization options.

import_parameters(Options) :-
    node_from_options(Node, Options),
    import_parameters(Node, global),
    import_parameters(Node, local).

import_parameters(Node, Scope) :-
    node_id(Node, NodeID),
    ros:ros_get_node_parameters(Node, Scope, Parms),
    maplist(import_params(NodeID), Parms).

import_params(NodeID, Dict) :-
    _{ node:Pattern, parameters:Pdict } :< Dict,
    matches(Pattern, NodeID),
    !,
    dict_pairs(Pdict, _, Pairs),
    maplist(import_param(NodeID), Pairs).

%!  matches(+Pattern, +Node) is semidet.
%
%   Should do pattern matching. Not sure how these node patterns work.

matches(Node, Node) :- !.
matches('/**', _) :- !.
matches(Pattern, Node) :-
    atom_concat(/, Node, Pattern).

import_param(Node, Name-Value) :-
    ros_set_param(Name, Value, [node(Node)]).


node_id(Node, Id), atom(Node) => Id = Node.
node_id(Node, Id) => ros_node_property(Node, name(Id)).
node_id(Node, _) => type_error(ros_node, Node).

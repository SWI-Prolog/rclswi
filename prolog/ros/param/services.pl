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

:- module(ros_param_services,
          [ ros_param_services/1        % +Options
          ]).
:- use_module(library(ros)).
:- use_module(library(ros/services)).
:- use_module(library(ros/detail/param)).
:- use_module(library(ros/detail/options)).
:- use_module(library(debug)).
:- use_module(library(apply)).

/** <module> ROS parameter services

This module defines the services that  are   needed  to  allow other ros
nodes to read and write the parameters of our node.
*/

%!  ros_param_services(+Options) is det.
%
%   Create the parameter services for a given node.

ros_param_services(Options) :-
    node_from_options(Node, Options),
    forall(srv_type(Which, SrvType),
           ros_param_service(Node, Which, SrvType, Options)).

ros_param_service(Node, Which, SrvType, Options) :-
    ros_node_property(Node, qname(QName)),
    atomic_list_concat([QName, Which], /, SrvName),
    debug(ros(parameters), 'Creating service for ~p, type ~p',
          [SrvName, SrvType]),
    ros_service(SrvName, SrvType, Service, Options),
    Closure =.. [Which,Node],
    ros_service_spin(Service, Closure).

:- det((list_parameters/3,
        describe_parameters/3,
        get_parameter_types/3,
        get_parameters/3,
        set_parameters/3,
        set_parameters_atomically/3
       )).
:- public
    list_parameters/3,
    describe_parameters/3,
    get_parameter_types/3,
    get_parameters/3,
    set_parameters/3,
    set_parameters_atomically/3.

%!  list_parameters(+Node, +Request, -Response) is det.
%
%   List the parameters this  node  knows   about.  Request  may contain
%   `depth` (integer) and `prefixes` (list(string)). What do these do?

list_parameters(Node, _Request, _{result:_{names:Names}}) :-
    findall(Name, ros_parameter_property(Node, Name, exists(true)), Names).

%!  get_parameter_types(+Node, +Request, -Response) is det.
%
%   Get the parameter types for a given set of parameters as integers.

get_parameter_types(Node, _{names:Strings}, _{types:TypeCodes}) :-
    maplist(parameter_type(Node), Strings, TypeCodes).

parameter_type(Node, String, TypeCode) :-
    atom_string(Param, String),
    ros_parameter_property(Node, Param, type(Type)),
    functor(Type, TypeName, _),
    ros:ros_enum_param_type(TypeCode, TypeName).

%!  describe_parameters(+Node, +Request, -Response) is det.
%
%   Compose a full parameter description from our store.

describe_parameters(Node, _{names:Strings}, _{descriptors:Descriptors}) :-
    maplist(parameter_description(Node), Strings, Descriptors).

parameter_description(Node, String, ParamDescriptor) :-
    atom_string(Param, String),
    findall(Facet, parameter_facet(Node, Param, Facet), Facets),
    dict_create(ParamDescriptor, _, Facets).

parameter_facet(_Node, Param, name(Param)).
parameter_facet(Node, Param, Facet) :-
    ros_parameter_property(Node, Param, type(Type)),
    functor(Type, TypeName, _),
    ros:ros_enum_param_type(TypeCode, TypeName),
    (   Facet = type(TypeCode)
    ;   type_details(Type, Facet)
    ).
parameter_facet(Node, Param, description(Text)) :-
    ros_parameter_property(Node, Param, description(Text)).
parameter_facet(Node, Param, additional_constraints(Text)) :-
    ros_parameter_property(Node, Param, additional_constraints(Text)).
parameter_facet(Node, Param, read_only(Bool)) :-
    ros_parameter_property(Node, Param, read_only(Bool)).

type_details(integer(Low, High),
             integer_range([_{from_value:Low, to_value:High, step:1}])).
type_details(integer(Low, High, Step),
             integer_range([_{from_value:Low, to_value:High, step:Step}])).
type_details(double(Low, High),
             floating_point_range([_{from_value:Low, to_value:High, step:0.0}])).
type_details(double(Low, High, Step),
             floating_point_range([_{from_value:Low, to_value:High, step:Step}])).

%!  get_parameters(+Node, +Request, -Response)
%
%   Get the types and values of requested parameters

get_parameters(Node, _{names:Strings}, _{values:Values}) :-
    maplist(get_parameter(Node), Strings, Values).

get_parameter(Node, String, Variant) :-
    atom_string(Param, String),
    (   ros_get_param(Param, Value, [node(Node)]),
        nonvar(Value)
    ->  (   ros_parameter_property(Node, Param, type(Type))
        ->  functor(Type, TypeName, _)
        ;   value_type(Value, TypeName)
        ),
        ros:ros_enum_param_type(TypeCode, TypeName),
        value_variant(TypeName, TypeCode, Value, Variant)
    ;   ros:ros_enum_param_type(TypeCode, not_set),
        Variant = _{type:TypeCode}
    ).

value_variant(TypeName, TypeCode, Value, Variant) :-
    type_field(TypeName, TypeField),
    dict_create(Variant, _, [type=TypeCode, TypeField=Value]).

%!  set_parameters_atomically(+Node, +Request, -Response) is det.
%
%   Set either all or no parameters. If   any parameter failed to be set
%   the successful ones remain unchanged and   the  returned response is
%   `_{successful:false, reason:"Atomically"}`.

set_parameters_atomically(Node, Request, Response) :-
    State = state(true),
    (   transaction(try_set_parameters(Node, Request, Response, State))
    ->  true
    ;   arg(1, State, Response)
    ).

try_set_parameters(Node, Request, Response, State) :-
    set_parameters(Node, Request, Response),
    (   maplist(successful, Response)
    ->  true
    ;   maplist(to_failure, Response, FailedResponse),
        nb_setarg(1, State, FailedResponse),
        fail
    ).

successful(_{successful:true}).

to_failure(_{successful:true}, _{successful:false, reason:"Atomically"}) :- !.
to_failure(Failure, Failure).

%!  set_parameters(+Node, +Request, -Response) is det.
%
%   Set parameters in our node. If multiple  parameters are set some may
%   get set and others not. Whether or   all  parameters are set or not,
%   the changes are atomically.

set_parameters(Node, _{parameters:Params}, _{results:Results}) :-
    transaction(maplist(set_parameter(Node), Params, Results)).

set_parameter(Node, _{name:String, value:Variant}, Result) :-
    atom_string(Param, String),
    variant_value(Variant, Value),
    (   catch(ros_set_param(Param, Value, [node(Node)]), E, true)
    ->  (   var(E)
        ->  Result = _{successful:true}
        ;   message_to_string(E, Message),
            Result = _{successful:false, reason:Message}
        )
    ;   Result = _{successful:false, reason:"Unknown failure"}
    ).

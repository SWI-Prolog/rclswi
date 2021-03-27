/*  This example code is public domain.
*/

:- module(params,
          [ argv_params/0,
            dump/0,
            demo_add_param/0,
            set_x/1
          ]).
:- use_module(library(ansi_term)).
:- use_module(library(broadcast)).
:- use_module(library(lists)).
:- use_module(library(pprint)).
:- use_module(library(ros)).
:- use_module(library(yaml)).
:- use_module(library(ros/param/store)).

:- reexport(library(ros)).
:- reexport(library(ros/param/client)).
:- reexport(library(ros/param/services)).
:- reexport(library(ros/param/store)).

/** <module> Parameter handling demos

Parameter syntax is oneof e.g.

    --param string_param:=test
    -p string_param:=test
    -p node:string_param:=test
    --params-file file
*/

% Declare the default node with a couple of parameters

:- ros_set_defaults(
       [ node(swi_param_demo,
              [ ros_args(['-p', 'x:=2']),
                parameters([ x    - [ type(integer)
                                    ],
                             name - [ type(string)
                                    ]
                           ])
              ])
       ]).

% Start the default node in a thread so we can use the toplevel

:- initialization(ros_spin([thread(spinner)]),
                  program).

% Use SWI-Prolog's broadcast library to listen to changes to parameters,
% either from ros_set_param/2,3 or remotely triggered.

:- listen(ros_parameter_events(Node, Events),
          print_parameter_event(Node, Events)).

print_parameter_event(Node, Events) :-
    ansi_format([bold], '~NParameter events on node ~p~n', [Node]),
    forall(member(Event, Events),
           format('  ~p~n', [Event])).

%!  argv_params
%
%   Dump the parameters as parsed from  commandline options (global) and
%   node arguments (local). This  is  not   intended  for  users of this
%   library, but illustrates the low-level behaviour.

argv_params :-
    ros_default_node(Node),
    argv_params(Node, global),
    argv_params(Node, local).

argv_params(Node, Which) :-
    ros:ros_get_node_parameters(Node, Which, Parms),
    format('~p parameters: ', [Which]),
    print_term(Parms, []),
    nl.

%!  dump
%
%   Illustrates dumping the current set of parameters as YAML.

dump :-
    findall(Name-Value,
            ros_get_param(Name, Value),
            Pairs),
    dict_pairs(Dict, _, Pairs),
    ros_default_node(Node),
    ros_node_property(Node, qname(Name)),
    yaml_write(current_output,
               _{}.put(Name,
                       _{ros__parameters:Dict})).

%!  demo_add_param
%
%   Add a new parameter  with  some   properties.  You  can  delete this
%   parameter again using:
%
%       ?- ros_delete_parameter(valve_open, []).

demo_add_param :-
    ros_parameter(valve_open,
                  [ default(0.5),
                    type(double(0.0, 1.0, 0.01)),
                    description("Amount to open the valve")
                  ]).

%!  set_x(+X)
%
%   Change the X coordinate.

set_x(X) :-
    ros_set_param(x, X).

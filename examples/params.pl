/*  This example code is public domain.
*/

:- module(params,
          [ dump_params/0,
            import_params/0
          ]).
:- reexport(library(ros)).
:- reexport(library(ros/param/client)).
:- reexport(library(ros/param/store)).

/** <module> Parameter handling demos

Parameter syntax is oneof e.g.

    --param string_param:=test
    -p string_param:=test
    -p node:string_param:=test
    --params-file file
*/

:- ros_set_defaults(
       [ node(swi_params,
              [ ros_args(['-p', 'x:=2'])
              ])
       ]).

%!  dump_params
%
%   Dump the parameters as parsed from  commandline options (global) and
%   node arguments (local).

dump_params :-
    ros_default_node(Node),
    dump(Node, global),
    dump(Node, local).

dump(Node, Which) :-
    ros:ros_get_node_parameters(Node, Which, Parms),
    format('~p parameters: '),
    print_term(Parms, []),
    nl.

%!  import_params
%
%   Declare parameters for the default node and dump their value. Should
%   print `x = 2` and `name = Var`. When called with commandline args as
%   below, we should get `name = hello`.
%
%       ... --ros-args -p name:=hello

import_params :-
    ros_parameter(x, [type(integer)]),
    ros_parameter(name, [type(string)]),
    import_parameters([]),
    forall(ros_get_param(Name, Value),
           format('~p = ~p~n', [Name, Value])).

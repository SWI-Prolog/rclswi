/* This code is in the public domain */

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Load all the ROS Prolog libraries. This is   nice  for using all the ROS
predicates interactively. Load using the   command below. Optionally add
``--port=Number``  _at  the  end_.  ``--no-doc``   does  not  start  the
documentation server.

    swipl -p library=install/rclswi/prolog src/rclswi/examples/all.pl

Surf to http://localhost:8080 to view the documentation.
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

:- doc_collect(true).
:- initialization(doc_server_and_toplevel, main).

doc_server_and_toplevel :-
    current_prolog_flag(argv, Argv),
    argv_options(Argv, _, Options),
    (   option(doc(true), Options, true)
    ->  option(port(Port), Options, 8080),
        doc_server(Port)
    ;   true
    ),
    set_prolog_flag(toplevel_goal, prolog).

:- use_module(library(ros)).
:- use_module(library(ros/clocks)).
:- use_module(library(ros/graph)).
:- use_module(library(ros/logging)).
:- use_module(library(ros/qos)).
:- use_module(library(ros/services)).
:- use_module(library(ros/types)).
:- use_module(library(ros/param/store)).
:- use_module(library(ros/param/client)).
:- use_module(library(ros/param/services)).
:- use_module(library(ros/action/client)).
:- use_module(library(ros/action/server)).


/* This code is in the public domain */

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Load all the ROS Prolog libraries. This is   nice  for using all the ROS
predicates interactively or browse the documentation by running

    swipl --pldoc -p library=install/rclswi/prolog src/rclswi/examples/all.pl
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

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


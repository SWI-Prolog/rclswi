% This file is always required to find the rclswi package in the
% ROS environment.
:- use_module(rclswi_setup).
:- use_module(library(ros)).
:- use_module(library(main)).
% Load this to enable --ssh=port, --debug=topic and --spy=name/arity
:- use_module(ros/debug).

% Use this to locate another ROS Prolog package and access Prolog
% files in there.
%:- use_module(ros/pkg).
%:- ros_use_package(mypack).
%:- use_module(mypack(coolstuff)).

:- initialization(main, main).

%!  main(+Argv)
%
%   This is the entry point  called   through  main/0  declared as entry
%   point above, indirectly through library(main).
%
%   library(main)  collects  the  commandline  arguments   and  sets  up
%   handling Control-C to stop the program.   In most cases main/1 shall
%   initialize the ROS node and call ros_spin/0,1.

main(_Argv) :-
    ros_log(info, 'Hello world', []).

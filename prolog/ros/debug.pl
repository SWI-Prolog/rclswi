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

:- module(ros_debug,
          [ start_interactor/0
          ]).
:- use_module(library(error)).
:- use_module(library(main)).
:- use_module(library(option)).
:- autoload(library(threadutil), [tspy/1]).

/** <module> ROS node debugging facilities

This library provides debugging facilities for   ROS  nodes. To use this
file, load it early in the process using

    :- use_module(library(ros/debug)).

It adds these command line options:

    - `--ssh[=port]`
      Start an SSH server (at _port_ or 2020)
    - `--spy=name/arity`
      Set a spy point to trap the Prolog debugger when _name/arity_
      is called.
    - `--debug=topic`
      Print debug messages matching _topic_
*/

%!  ros_setup_debug
%
%   This is by default executed on program startup.  Depending
%   on the process command line arguments this may
%
%     - Create an SSH server
%     - Activate debug/3 channels
%     - Set Prolog spy points using tspy/1

:- initialization(ros_setup_debug, program).

ros_setup_debug :-
    current_prolog_flag(argv, Argv),
    argv_options(Argv, _, Options),
    ros_setup_debug(Options).


%!  ros_setup_debug(+Options)
%
%   Setup debugging for a ROS node.  This supports the following options:
%
%     - ssh(+Port)
%       Create an SSH server at Port.
%     - debug(Topic)
%       Enable Prolog debug messages for Topic
%     - spy(Predicate)
%       Set a spy point on Predicate using tspy/1.

ros_setup_debug(Options) :-
    setup_debug(Options),
    setup_spypoints(Options),
    (   option(sshd(_), Options)
    ->  start_sshd(Options)
    ;   true
    ).


:- if(exists_source(library(ssh_server))).
:- use_module(library(ssh_server)).

%!  start_sshd(+Options)
%
%   Start  a  Prolog  SSH  server  if   Options  contains  ssh(Port)  or
%   ssh(true). The latter creates an SSH server at port 2020.
%
%   This creates a secondary console  that   allows  for  inspecting the
%   dynamic database, setting _spy_ points   and (re)loading the program
%   in an operating ROS node.
%
%   The primary secondary console is realised  by providing SSH login to
%   the                 node                 using                   the
%   [libssh](https://www.swi-prolog.org/pack/list?p=libssh) add on   for
%   SWI-Prolog.
%
%   To enable SSH login to a node,  simply add `--ssh[=port]` to the ROS
%   command line, e.g.
%
%        ros2 run mypkg myprog --ssh=2021
%
%   Now you can login to  the  node   using  the  command below. See the
%   libssh package documentation for   details regarding authentication.
%   Normally it suffices to use `ssh-keygen` to   produce  a key pair in
%   ``~/.ssh``.
%
%        ssh -p 2021 localhost

start_sshd(Options) :-
    (   option(sshd(SSHD), Options)
    ->  (   SSHD == true
        ->  Port = 2020
        ;   integer(SSHD)
        ->  Port = SSHD
        ;   type_error(port, SSHD)
        ),
        ssh_server(Port),
        print_message(informational, ssh_login(Port))
    ;   true
    ).

:- else.

start_sshd(Options) :-
    (   option(sshd(_), Options)
    ->  print_message(warning, install(libssh))
    ;   true
    ).

:- endif.

%!  start_interactor
%
%   Utility predicate to provide  a  secondary   console  to  the Prolog
%   process using either SSH, running an `xterm`  or telling the user to
%   open a tab in the Prolog GUI.

:- if(current_predicate(ssh_server/0)).
start_interactor :-
    ssh_server(2020),
    print_message(informational, ssh_login(2020)).
:- elif(absolute_file_name(path(xterm), _,
                           [access(execute),file_errors(fail)])).
start_interactor :-
    interactor.
:- elif(current_prolog_flag(console_menu,true)).
start_interactor :-
    print_message(informational, open_thread_tab).
:- else.
start_interactor :-
    print_message(error, no_second_console).
:- endif.

%!  setup_debug(+Options)
%
%   Process commandline arguments to setup debugging.

setup_debug(Options) :-
    maplist(setup_debug, Options).

setup_debug(debug(Topic)) :-
    !,
    (   catch(term_string(Term, Topic, []), _, true)
    ->  debug(Term)
    ;   debug(Topic)
    ).
setup_debug(_).


%!  setup_spypoints(+Options)
%
%   Setup spy-points for the graphical debugger.

setup_spypoints(Options) :-
    maplist(setup_spy, Options).

setup_spy(spy(Atom)) :-
    !,
    atom_pi(Atom, PI),
    tspy(user:PI).
setup_spy(_).

atom_pi(Atom, Module:PI) :-
    split(Atom, :, Module, PiAtom),
    !,
    atom_pi(PiAtom, PI).
atom_pi(Atom, Name//Arity) :-
    split(Atom, //, Name, Arity),
    !.
atom_pi(Atom, Name/Arity) :-
    split(Atom, /, Name, Arity),
    !.
atom_pi(Atom, _) :-
    format(user_error, 'Invalid predicate indicator: "~w"~n', [Atom]),
    halt(1).

split(Atom, Sep, Before, After) :-
    sub_atom(Atom, BL, _, AL, Sep),
    !,
    sub_atom(Atom, 0, BL, _, Before),
    sub_atom(Atom, _, AL, 0, AfterAtom),
    (   atom_number(AfterAtom, After)
    ->  true
    ;   After = AfterAtom
    ).


		 /*******************************
		 *            MESSAGES		*
		 *******************************/

prolog:message(install(libssh)) -->
    [ 'Could not find library(ssh_server).  You can install this using'-[],nl,
      '  ?- pack_install(libssh).'-[],
      'Make sure to have the libssh development libraries installed.'-[],nl
    ].
prolog:message(ssh_login(Port)) -->
    [ 'Started an SSH server on port ~p.  Please login from a second'-[Port],nl,
      'console using the command below and (when requested) the passphrase'-[],nl,
      'from your SSH key in ~~/.ssh'-[], nl,nl,
      '    ssh -p ~w localhost'-[Port],
      nl,nl
    ].
prolog:message(open_thread_tab) -->
    [ 'Please use the "Run/New thread" menu to create a second console'-[] ].
prolog:message(no_second_console) -->
    [ 'Could not find a way to create a second console on the same Prolog \c
       process.  That is needed for these demos.'-[]
    ].

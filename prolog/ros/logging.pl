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

:- module(ros_logging,
          [ ros_log/3,             % +Level, +Format, +Args
            ros_log_pos/4,         % +Level, +Format, +Args, +Pos

            ros_set_logger/1,      % +Level
            ros_logger_property/1  % -Level
          ]).
:- use_module(library(ros)).
:- use_module(library(error)).
:- use_module(library(prolog_code)).

/** <module> ROS logging interface

This library allows controlling the ROS   logging  system. The predicate
ros_log/3 is used to send  ROS   log  messages immediately. This library
also  hooks  SWI-Prolog  print_message/2.   If    the   session  is  not
interactive, SWI-Prolog messages  are  redirected   to  the  ROS logging
system.
*/

%!  ros_log(+Severity, +Format, +Args) is det.
%
%   @arg Severity is an integer or a   severity name. Severity names are
%   case insensitive. Examples are  `unset`,   `debug`,  `info`, `warn`,
%   `error`, `fatal`

ros_log(Severity, Format, Args) :-
    format(string(Message), Format, Args),
    ros_logger(Logger),
    ros_log(Logger, Severity, Message, _{}).

%!  ros_log_pos(+Severity, +Format, +Args, +Pos:dict) is det.
%
%   Normally called as rewritten call  to   ros_log/3.  Pos provides the
%   predicate, file and line that is the origin of the call.

ros_log_pos(Severity, Format, Args, Pos) :-
    format(string(Message), Format, Args),
    ros_logger(Logger),
    ros_log(Logger, Severity, Message, Pos).

ros_log(Logger, Severity, Message, Pos) :-
    !,
    Pos :< _{predicate:PI, file:File, line:Line},
    format(string(Pred), "~q", PI),
    ros:'$ros_log'(Logger, Severity, Message,
                   Pred, File, Line).
ros_log(Logger, Severity, Message, _) :-
    ros:'$ros_log'(Logger, Severity, Message,
                   "ros_log/3", "logging.pl", 0).


%!  ros_set_logger(+Property) is det.
%
%   Set properties for the default logger.  Defined properties are:
%
%     - level(Severity)
%       Log actions with severity >= severity.  Default is `info`.

%!  ros_logger_property(?Property) is nondet.
%
%   Query properties for the default logger.  Defined properties are
%
%     - name(Name)
%       Name of the logger.  This defaults to the name of the default
%       ROS node.
%     - level(Severity)
%       See ros_set_logger/1
%     - rosout(Bool)
%       Whether or not logging is forwarded to ``/rosout``.

ros_set_logger(level(Severity)) =>
    ros_logger(Name),
    ros:'$ros_set_logger_level'(Name, Severity).
ros_set_logger(Property) =>
    domain_error(ros_logger_property, Property).

ros_logger_property(name(Name)) :-
    ros_logger(Name).
ros_logger_property(level(Severity)) :-
    ros_logger(Name),
    ros:'$ros_get_logger_level'(Name, Severity).
ros_logger_property(rosout(Enabled)) :-
    (   ros:'$ros_logging_rosout_enabled'
    ->  Enabled = true
    ;   Enabled = false
    ).

ros_logger(Name) :-
    ros_default_node(Node),
    ros_node_property(Node, logger_name(Name)).


		 /*******************************
		 *            MESSAGES		*
		 *******************************/

% forward messages from SWI-Prolog's message system   to ROS. Ideally we
% should only send messages to ``/rosout``.

:- multifile
    user:message_hook/3.

user:message_hook(Term, Level, _Lines) :-
    \+ current_prolog_flag(break_level, _), % Sesion is not interactive
    swi_ros_message_proxy(Level, ROSLevel),
    message_to_string(Term, String),
    (   source_location(File, Line)
    ->  true
    ;   File = "",
        Line = 0
    ),
    ros_log_pos(ROSLevel, "~s", [String],
                _{predicate:print_message/2, file:File, line:Line}),
    fail.

swi_ros_message_proxy(debug(_),    debug).
swi_ros_message_proxy(information, info).
swi_ros_message_proxy(warning,     warn).
swi_ros_message_proxy(error,       error).


		 /*******************************
		 *           POSITION		*
		 *******************************/

:- multifile
    user:goal_expansion/4.

user:goal_expansion(ros_log(Severity, Format, Args), Pos,
                    ros_log_pos(Severity, Format, Args, Source), Pos) :-
    Source = pos{file:File, predicate:PI, line:Line},
    prolog_load_context(file, File),
    prolog_load_context(term, Term),
    term_predicate(Term, PI),
    file_line(Pos, Line).

file_line(Pos, Line) :-
    compound(Pos),
    arg(1, Pos, Offset),
    prolog_load_context(term_position, TermPos),
    stream_position_data(char_count, TermPos, Start),
    Skip is Offset-Start,
    prolog_load_context(stream, Stream),
    stream_property(Stream, position(Here)),
    setup_call_cleanup(
        catch(set_stream_position(Stream, TermPos), _, fail),
        ( forall(between(1,Skip,_), get_char(Stream, _)),
          line_count(Stream, Line)
        ),
        set_stream_position(Stream, Here)),
    !.
file_line(_Offset, Line) :-
    prolog_load_context(term_position, TermPos),
    stream_position_data(line_count, TermPos, Line).

term_predicate((Head :- _), PI) =>
    term_predicate(Head, PI).
term_predicate(Head, PI) =>
    prolog_load_context(module, Module),
    pi_head(PI, Module:Head).

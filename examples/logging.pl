:- module(logging,
          [ go/0
          ]).
:- reexport(library(ros)).
:- reexport(library(ros/logging)).

:- ros_set_defaults(
       [ node(swi_logging, [])
       ]).

go :-
    ros_default_node(Node),
    ros_node_property(Node, logger_name(Logger)),
    ros_logger_property(level(Level)),
    format('Log ~p processes log events >= ~p~n', [Logger, Level]),
    ros_logger_property(rosout(Enabled)),
    format('Logging to /rosout is ~p~n', [Enabled]),
    (   ros_severity(Severity),
        ros_log(Severity, "Message at log severity ~p", [Severity]),
        fail
    ;   true
    ).

ros_severity(unset).
ros_severity(debug).
ros_severity(info).
ros_severity(warn).
ros_severity(error).
ros_severity(fatal).

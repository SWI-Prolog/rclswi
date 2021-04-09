/* This code is in the public domain */

:- module(constants,
          [ is_executing/1
          ]).
:- use_module(library(ros/types)).

/** <module> Demonstrate access to ROS constants

ROS message files define  constants   as  below.  The library(ros/types)
provides access to these constants.

    <type> <name>=<value>
*/

% Import a type

:- ros_import_type('action_msgs/msg/GoalStatus').

%!  is_executing(?Status).
%
%   True when Status is the integer  status associated with the constant
%   ``STATUS_EXECUTING``. Provided the constant name is instantiated and
%   the Status is a variable, ros_constant/3   is  evaluated at _compile
%   time_ and thus introduces no  overhead.   You  can verify this using
%   listing/1:
%
%   ```
%   ?- listing(constants:is_executing/1).
%   is_executing(2).
%   ```
%
%   If the constant is unknown an existence_error   is raised and if the
%   constant is associated with multiple types  and has different values
%   an ambiguity_error is raised.  In  the   latter  case  the user must
%   specify the type in the first argument using of the formats below.
%
%     - 'action_msgs/msg/GoalStatus'
%     - 'GoalStatus'
%     - goal_status

is_executing(Status) :-
    ros_constant(_, 'STATUS_EXECUTING', Status).

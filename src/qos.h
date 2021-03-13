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

#ifndef RCLSWI_QOS_H_INCLUDED
#define RCLSWI_QOS_H_INCLUDED

extern void	install_ros_qos(void);
extern int	get_qos_profile(term_t t, rmw_qos_profile_t **profile);

#endif /*RCLSWI_QOS_H_INCLUDED*/

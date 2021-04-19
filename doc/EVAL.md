# Service

## /add_two_ints

Python implementation as per [1].  Logger   call  removed. Timing on AMD
3950X CPU running Ubuntu 20.04. SWI-Prolog compiled using gcc 9, -O3 (no
PGO optimization which makes it about 30% faster). Rclswi compiled using
gcc 7 -O0 for debugging.

  - Server and client SWI-Prolog
    - Both run at 75% CPU
    - 1,000,000 requests take 13 sec CPU (client) and 51 sec wall time

  - Server Python, client SWI-Prolog
    - Server at 125% CPU, client at 35 %CPU
    - 1,000,000 requests take 15 sec CPU (client) and 109 sec wall time

### Valgrind analysis

  - 20.3% time is spent in Prolog
  - 8.6% in ros_take_request, divided in 2.3% for RCL and 4.7% translating
    the message from C to Prolog.  2.9% is converting type and field names
    to Prolog.
  - 8.9% in ros_wait.  6.4% is for the RCL layer.
  - 2% in ros_send_response.  1.3% in translating message from Prolog to C
    of which 0.66% is, again, field name translation.


[1] https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Service-And-Client.html

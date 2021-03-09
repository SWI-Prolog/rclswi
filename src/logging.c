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

#include <SWI-Stream.h>
#include <SWI-Prolog.h>

#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rcutils/logging.h>
#include <rcutils/time.h>
#include <rcl/logging.h>

#ifdef HAVE_RCL_LOGGING_INTERFACE	/* not yet in foxy */
#include <rcl_logging_interface/rcl_logging_interface.h>
#endif

#include "common.h"

static atom_t ATOM_unset;
static atom_t ATOM_debug;
static atom_t ATOM_info;
static atom_t ATOM_warn;
static atom_t ATOM_error;
static atom_t ATOM_fatal;

static foreign_t
ros_logging_initialize(void)
{ int rc = TRUE;

  TRY(rcutils_logging_initialize());
  TRY(rcl_logging_configure_with_output_handler(
	  &rclswi_default_context()->global_arguments,
	  &rclswi_default_allocator,
	  rcl_logging_multiple_output_handler)); /* TBD: Must be wrapped to be thread-safe */

  return rc;
}

static foreign_t
ros_logging_shutdown(void)
{ int rc = TRUE;

  TRY(rcl_logging_fini());
  TRY(rcutils_logging_shutdown());

  return rc;
}

static atom_t log_severity_atoms[RCUTILS_LOG_SEVERITY_FATAL+1] = {0};

static int
severity_name_to_level(term_t Name, int *levelp)
{ int rc = TRUE;
  int level;
  rcutils_ret_t ret;
  atom_t name;

  if ( !PL_get_atom(Name, &name) )
    return FALSE;

  for(int i=0; i<=RCUTILS_LOG_SEVERITY_FATAL; i++)
  { if ( log_severity_atoms[i] == name )
    { *levelp = i;
      return TRUE;
    }
  }

  ret = rcutils_logging_severity_level_from_string(PL_atom_chars(name),
						   rclswi_default_allocator, &level);
  if ( ret == RCUTILS_RET_LOGGING_SEVERITY_STRING_INVALID )
    return PL_domain_error("ros_log_level", Name);
  TRY(ret);

  if ( rc )
  { if ( level <= RCUTILS_LOG_SEVERITY_FATAL )
    { log_severity_atoms[level] = name;
      PL_register_atom(name);
    }
    *levelp = level;
  }

  return rc;
}

static int
get_severity_level(term_t t, int *levelp)
{ if ( severity_name_to_level(t, levelp) )
    return TRUE;
  if ( PL_exception(0) )
    return FALSE;

  return PL_get_integer(t, levelp);
}


static foreign_t
ros_set_logger_level(term_t Name, term_t Level)
{ char *name;
  int level;
  int rc = TRUE;

  if ( (rc=(get_utf8_name_ex(Name, &name) &&
	    get_severity_level(Level, &level))) )
    TRY(rcutils_logging_set_logger_level(name, level));

  return rc;
}

static foreign_t
ros_get_logger_level(term_t Name, term_t Level)
{ char *name;
  int rc = TRUE;

  if ( (rc=get_utf8_name_ex(Name, &name)) )
  { int level = rcutils_logging_get_logger_effective_level(name);
    if ( level > 0 )
    { switch(level)
      { case RCUTILS_LOG_SEVERITY_UNSET:
	  return PL_unify_atom(Level, ATOM_unset);
	case RCUTILS_LOG_SEVERITY_DEBUG:
	  return PL_unify_atom(Level, ATOM_debug);
	case RCUTILS_LOG_SEVERITY_INFO:
	  return PL_unify_atom(Level, ATOM_info);
	case RCUTILS_LOG_SEVERITY_WARN:
	  return PL_unify_atom(Level, ATOM_warn);
	case RCUTILS_LOG_SEVERITY_ERROR:
	  return PL_unify_atom(Level, ATOM_error);
	case RCUTILS_LOG_SEVERITY_FATAL:
	  return PL_unify_atom(Level, ATOM_fatal);
      }

      rc = PL_unify_integer(Level, level);
    } else
      return PL_existence_error("ros_log_name", Name);
  }

  return rc;
}


static foreign_t
ros_log(term_t Name, term_t Severity, term_t Message,
	term_t Predicate, term_t File, term_t Line)
{ char *name;
  int severity;
  char *message;
  char *predicate, *file;
  int64_t line;

  if ( get_utf8_name_ex(Name, &name) &&
       get_severity_level(Severity, &severity) &&
       get_utf8_name_ex(Message, &message) &&
       get_utf8_name_ex(Predicate, &predicate) &&
       get_utf8_name_ex(File, &file) &&
       PL_get_int64(Line, &line) )
  { rcutils_log_location_t logging_location = {predicate, file, line};

    RCUTILS_LOGGING_AUTOINIT;
    rcutils_log(&logging_location, severity, name, "%s", message);

    return TRUE;
  }

  return FALSE;
}


static foreign_t
ros_logging_rosout_enabled(void)
{ return !!rcl_logging_rosout_enabled();
}


#ifdef HAVE_RCL_LOGGING_INTERFACE
static foreign_t
ros_log_directory(term_t dir)
{ int rc = TRUE;
  char *log_dir = NULL;

  TRY(rcl_logging_get_logging_directory(rclswi_default_allocator, &log_dir));
  rc = rc && PL_unify_chars(dir, PL_ATOM|REP_MB, (size_t)-1, log_dir);
  if ( log_dir )
    rcl_free(log_dir);

  return rc;
}
#endif


		 /*******************************
		 *	      REGISTER		*
		 *******************************/

void
install_ros_logging(void)
{ MKATOM(unset);
  MKATOM(debug);
  MKATOM(info);
  MKATOM(warn);
  MKATOM(error);
  MKATOM(fatal);

  PL_register_foreign("$ros_logging_initialize",     0,	ros_logging_initialize,	    0);
  PL_register_foreign("$ros_logging_shutdown",	     0,	ros_logging_shutdown,	    0);
  PL_register_foreign("$ros_set_logger_level",	     2,	ros_set_logger_level,	    0);
  PL_register_foreign("$ros_get_logger_level",	     2,	ros_get_logger_level,	    0);
  PL_register_foreign("$ros_logging_rosout_enabled", 0,	ros_logging_rosout_enabled, 0);
  PL_register_foreign("$ros_log",		     6,	ros_log,		    0);
#ifdef HAVE_RCL_LOGGING_INTERFACE
  PL_register_foreign("$ros_log_directory",	     1, ros_log_directory,          0);
#endif
}

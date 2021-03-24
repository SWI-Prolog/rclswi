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

#ifndef RCLSWI_COMMON_H_INCLUDED
#define RCLSWI_COMMON_H_INCLUDED

#include <rcl/types.h>
#include <rcl/allocator.h>
#include <rcl/context.h>


		 /*******************************
		 *	     DEBUGGING		*
		 *******************************/

int	ros_debug_level(void);

#define DEBUG(level, goal) \
	do				\
	{ if ( ros_debug_level() >= level )	\
	    goal;			\
	} while(0)

		 /*******************************
		 *	       ERRORS		*
		 *******************************/

int	set_error(rcl_ret_t ret);
void	print_error(rcl_ret_t ret, const char *file, int line, const char *goal);

#define TRY(goal) \
	do				\
	{ if ( rc )			\
	  { DEBUG(9, Sdprintf("Running %s -> ", #goal)); \
	    rcl_ret_t __ret = (goal);	\
	    DEBUG(9, Sdprintf("%d\n", __ret)); \
	    if ( __ret != RCL_RET_OK )	\
	      rc = set_error(__ret);	\
	  }				\
	} while(0)

#define TRY_SILENT(goal) \
	do				\
	{ if ( rc )			\
	  { DEBUG(11, Sdprintf("Running %s -> ", #goal)); \
	    rcl_ret_t __ret = (goal);	\
	    DEBUG(11, Sdprintf("%d\n", __ret)); \
	    if ( __ret != RCL_RET_OK )	\
	      rc = set_error(__ret);	\
	  }				\
	} while(0)

#define TRY_ANYWAY(goal) \
	do				\
	{ DEBUG(9, Sdprintf("Running (rc=%d) %s -> ", rc, #goal)); \
	  rcl_ret_t __ret = (goal);	\
          DEBUG(9, Sdprintf("%d\n", __ret)); \
	  if ( __ret != RCL_RET_OK )	\
	    rc = set_error(__ret);	\
	} while(0)

#define OUTFAIL \
	do				\
	{ rc = FALSE;			\
	  goto out;			\
	} while(0)

#define TRYVOID(goal) \
	do				\
	{ rcl_ret_t __ret = (goal);	\
	  if ( __ret != RCL_RET_OK )	\
	    print_error(__ret, __FILE__, __LINE__, #goal); \
	} while(0)


		 /*******************************
		 *	        UTIL		*
		 *******************************/

extern rcl_allocator_t rclswi_default_allocator;

extern rcl_context_t *rclswi_default_context(void);
extern void *	rcl_alloc(size_t bytes, rcl_allocator_t *allocator);
extern void	rcl_free(void *ptr, rcl_allocator_t *allocator);
extern int	get_utf8_name_ex(term_t t, char **name);


		 /*******************************
		 *	    ENUM SUPPORT	*
		 *******************************/

typedef struct enum_decl
{ int		value;
  const char   *name;
  atom_t	atom;
} enum_decl;

#define EN_DECL(value, name) { (int)value, #name, 0 }
#define EN_END()	     { 0, NULL, 0 }

int	get_enum(term_t t, const char *domain, enum_decl *decl, int *value);
int	put_enum(term_t t, enum_decl *decl, int value);
int	unify_enum(term_t t, enum_decl *decl, int value);


		 /*******************************
		 *	   REGISTRATION		*
		 *******************************/

#define MKFUNCTOR(n,a) \
        FUNCTOR_ ## n ## a = PL_new_functor(PL_new_atom(#n), a)
#define MKATOM(n) \
	ATOM_ ## n = PL_new_atom(#n);

#endif /*RCLSWI_COMMON_H_INCLUDED*/

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

#include <SWI-Prolog.h>
#include "common.h"

int
get_enum(term_t t, const char *domain, enum_decl *decl, int *value)
{ atom_t a;

  if ( PL_get_atom_ex(t, &a) )
  { for(; decl->name; decl++)
    { if ( !decl->atom )
	decl->atom = PL_new_atom(decl->name);

      if ( decl->atom == a )
      { *value = decl->value;
	return TRUE;
      }
    }

    return PL_domain_error(domain,  t);
  } else
    return FALSE;
}

int
put_enum(term_t t, enum_decl *decl, int value)
{ for(; decl->name; decl++)
  { if ( decl->value == value )
    { if ( !decl->atom )
	decl->atom = PL_new_atom(decl->name);
      return PL_put_atom(t, decl->atom);
    }
  }

  return PL_unify_integer(t, value);
}

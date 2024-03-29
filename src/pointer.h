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

#ifndef SWI_POINTER_H_INCLUDED
#define SWI_POINTER_H_INCLUDED

typedef struct c_pointer_type
{ const char   *type;			/* Type of the pointer */
  const void  (*free)(void *);		/* Free pointer */
  atom_t	name;			/* Type as atom */
} c_pointer_type;

int	unify_pointer(term_t t, void *ptr, const c_pointer_type *type);
int	get_pointer(term_t t, void **ptr, const c_pointer_type *type);
int	get_pointer_and_symbol(term_t t, void **ptr, atom_t *symbol,
			       const c_pointer_type *type);
int	get_pointer_ex(term_t t, void **ptr, atom_t *symbol,
		       const c_pointer_type **type);
int	get_pointer_type(term_t t, void **ptr, const c_pointer_type **type);
void   *get_pointer_from_symbol(atom_t symbol);

#endif /*SWI_POINTER_H_INCLUDED*/

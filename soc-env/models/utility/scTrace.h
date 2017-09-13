/*
 * Copyright (c) 2016 Baptiste Roux.
 * email: <baptiste.roux AT inria.fr>.
 */

/*
 * Modules facilities to trace systemC simulation signal.
 */

#ifndef _SC_TRACE_H
#define _SC_TRACE_H

#include <inttypes.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "systemc.h"

/*---
 * CLASS DEFINITION
 *----------------------------------------------------------------------------*/
class scTrace: public sc_core::sc_module
{
	sc_trace_file *trace_fd = NULL;
  template < typename T > void scTrace_template(sc_object *obj);
public:
  scTrace(sc_core::sc_module_name name);
  ~scTrace();
  void traceModule(const sc_module& mod, const char *name);
};
#endif /*_SC_TRACE_H*/

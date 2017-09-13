#include "scTrace.h"

template < typename T > void scTrace::scTrace_template(sc_object *obj){
	T* object ;
	if ((object  = dynamic_cast < T* >(obj)))
		sc_trace(trace_fd, *object , object->name());
}

scTrace::scTrace(sc_core::sc_module_name name){
    trace_fd = sc_create_vcd_trace_file(name);
}

scTrace::~scTrace(){
    sc_close_vcd_trace_file(trace_fd);
}

void scTrace::traceModule(const sc_module& mod, const char *name){

	std::vector < sc_object* > child = mod.get_child_objects();

	for ( unsigned i = 0; i < child.size(); i++ ) {
		sc_module* subMod;
		sc_object* trgt = child[i];

		/* List of available monitored signal type */
		scTrace_template < sc_core::sc_signal < bool > > (trgt);
		scTrace_template < sc_core::sc_signal < bool, SC_MANY_WRITERS > > (trgt);
		scTrace_template < sc_core::sc_signal < sc_bv<16> > > (trgt);
		scTrace_template < sc_core::sc_signal < sc_bv<32> > > (trgt);

		if ((subMod = dynamic_cast < sc_module* > (trgt)))
			traceModule(*subMod, subMod->name());
	}
}

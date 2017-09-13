#include "iconnect.h"

/*---
 * METHODS IMPLEMENTATION
 *----------------------------------------------------------------------------*/
iconnect::iconnect(sc_core::sc_module_name name, uint8_t nbSsk, uint8_t nbMsk)
	: sc_module(name), nb_Ssk(nbSsk), nb_Msk(nbMsk)
{
  //init map struct
  map = new memmap_entry[nb_Msk*4];
  // init syctemC socket
	char txt[32];
  cpu_Ssk = new tlm_utils::simple_target_socket_tagged<iconnect>*[nbSsk];
	for (uint sk = 0; sk < nbSsk; sk++) {
		sprintf(txt, "target_socket_%d", sk);

		cpu_Ssk[sk] = new tlm_utils::simple_target_socket_tagged<iconnect>(txt);

		cpu_Ssk[sk]->register_b_transport(this, &iconnect::b_transport, sk);
		cpu_Ssk[sk]->register_transport_dbg(this, &iconnect::transport_dbg, sk);
		cpu_Ssk[sk]->register_get_direct_mem_ptr(this,
				&iconnect::get_direct_mem_ptr, sk);
	}

  ic_Msk = new tlm_utils::simple_initiator_socket_tagged<iconnect>*[nbMsk];
	for (uint sk = 0; sk < nbMsk; sk++) {
		sprintf(txt, "init_socket_%d", sk);
		ic_Msk[sk] = new tlm_utils::simple_initiator_socket_tagged<iconnect>(txt);

		ic_Msk[sk]->register_invalidate_direct_mem_ptr(this,
				&iconnect::invalidate_direct_mem_ptr, sk);
		map[sk].size = 0;
	}
}

iconnect::~iconnect(){
  delete[] map;
  //Ssk
  for (uint sk=0 ; sk< nb_Ssk; sk++){
    delete cpu_Ssk[sk];
  }
  delete[] ic_Msk;
  //Msk
  for (uint sk=0 ; sk< nb_Msk; sk++){
    delete ic_Msk[sk];
  }
  delete[] ic_Msk;
}

int iconnect::memmap(
		sc_dt::uint64 addr, sc_dt::uint64 size,
		enum addrmode addrmode, int idx,
		tlm::tlm_target_socket<> &s)
{
	for (uint i = 0; i < nb_Msk * 4; i++) {
		if (map[i].size == 0) {
			/* Found a free entry.  */
			map[i].addr = addr;
			map[i].size = size;
			map[i].addrmode = addrmode;
			map[i].sk_idx = i;
			if (idx == -1)
				ic_Msk[i]->bind(s);
			else
				map[i].sk_idx = idx;
			return i;
		}
	}
	printf("FATAL! mapping onto full interconnect!\n");
	abort();
	return -1;
}

unsigned int iconnect::map_address(
			sc_dt::uint64 addr,
			sc_dt::uint64& offset)
{

	for (uint i = 0; i < nb_Msk * 4; i++) {
		if (map[i].size
		    && addr >= map[i].addr
		    && addr <= (map[i].addr + map[i].size)) {
			if (map[i].addrmode == ADDRMODE_RELATIVE) {
				offset = addr - map[i].addr;
			} else {
				offset = addr;
			}
			return map[i].sk_idx;
		}
	}
	/* Did not find any slave !?!?  */
        printf("DECODE ERROR! %lx\n", (unsigned long) addr);
#ifdef ICONNECT_DEBUG
        for(int k=0; k<nb_Msk*4;k++){
          printf("map[%d] = addr:0x%lx, size:0x%x, sk_idx:%d\n", k, map[k].addr, map[k].size, map[k].sk_idx);
        }
#endif
	return 0;
}

void iconnect::unmap_offset(
			unsigned int target_nr,
			sc_dt::uint64 offset,
			sc_dt::uint64& addr)
{
	if (target_nr >= nb_Msk) {
		SC_REPORT_FATAL("TLM-2", "Invalid target_nr in iconnect\n");
	}

	if (map[target_nr].addrmode == ADDRMODE_RELATIVE) {
		if (offset >= map[target_nr].size) {
			SC_REPORT_FATAL("TLM-2", "Invalid range in iconnect\n");
		}

		addr = map[target_nr].addr + offset;
	} else {
		addr = offset;
	}
	return;
}

void iconnect::b_transport(int id,
			tlm::tlm_generic_payload& trans, sc_time& delay)
{
	sc_dt::uint64 addr;
	sc_dt::uint64 offset;
	unsigned int target_nr;

	if (id >= (int) nb_Ssk) {
		SC_REPORT_FATAL("TLM-2", "Invalid socket tag in iconnect\n");
	}


	addr = trans.get_address();
	target_nr = map_address(addr, offset);

  #ifdef ICONNECT_DEBUG
  printf("Received Addr 0x%lx; dec(%ld)\n", trans.get_address(), trans.get_address());
  printf("Split in addr: 0x%lx offset: 0x%lx\n", addr, offset);
  printf("Output on socket %d\n", target_nr);
  printf("Transcation length %d\n", trans.get_data_length());
  #endif

	trans.set_address(offset);
	/* Forward the transaction.  */
	(*ic_Msk[target_nr])->b_transport(trans, delay);
	/* Restore the addresss.  */
	trans.set_address(addr);
}

bool iconnect::get_direct_mem_ptr(int id,
					tlm::tlm_generic_payload& trans,
					tlm::tlm_dmi& dmi_data)
{
	sc_dt::uint64 addr;
	sc_dt::uint64 offset;
	unsigned int target_nr;
	bool r;

	if (id >= (int) nb_Ssk) {
		SC_REPORT_FATAL("TLM-2", "Invalid socket tag in iconnect\n");
	}

	addr = trans.get_address();
	target_nr = map_address(addr, offset);

	trans.set_address(offset);
	/* Forward the transaction.  */
	r = (*ic_Msk[target_nr])->get_direct_mem_ptr(trans, dmi_data);

	unmap_offset(target_nr, dmi_data.get_start_address(), addr);
	dmi_data.set_start_address(addr);
	unmap_offset(target_nr, dmi_data.get_end_address(), addr);
	dmi_data.set_end_address(addr);
	return r;
}

unsigned int iconnect::transport_dbg(int id,
				tlm::tlm_generic_payload& trans)
{
	sc_dt::uint64 addr;
	sc_dt::uint64 offset;
	unsigned int target_nr;

	if (id >= (int) nb_Ssk) {
		SC_REPORT_FATAL("TLM-2", "Invalid socket tag in iconnect\n");
	}

	addr = trans.get_address();
	target_nr = map_address(addr, offset);

	trans.set_address(offset);
	/* Forward the transaction.  */
	(*ic_Msk[target_nr])->transport_dbg(trans);
	/* Restore the addresss.  */
	trans.set_address(addr);
	return 0;
}

void iconnect::invalidate_direct_mem_ptr(int id,
                                         sc_dt::uint64 start_range,
                                         sc_dt::uint64 end_range)
{
	sc_dt::uint64 start, end;

	unmap_offset(id, start_range, start);
	unmap_offset(id, end_range, end);

	for (uint i = 0; i < nb_Ssk; i++) {
		(*cpu_Ssk[i])->invalidate_direct_mem_ptr(start, end);
	}
}

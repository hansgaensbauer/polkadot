
//Header file with data types and other stuff
#include "polkadot.h"
#include "packet_formats.h"
#include "sx1276.h"
#include <stdint.h>
#include "main.h"
#include "stm32f4xx.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

//initialize this as a mesh node
uint8_t polkadot_init(uint32_t id){
	DEBUG_PRINT("Initializing Polkadot Node\n\r");
	my_id = id;
	rreq_id = rand();
	my_sequence_number = 0;
	DEBUG_PRINT("Node ID: %d, Sequence Number: %d, RREQ ID: %d\n\r", my_id, my_sequence_number, rreq_id);
	return SUCCESS;
}

//This is the function that the user calls when they want to send data somewhere. This function hides all of the
//mesh networking.
uint8_t mesh_transmit(uint32_t destination_id, uint8_t data[], uint8_t data_length){
	DEBUG_PRINT("Sending Data Over the Mesh\n\r");
	int8_t route_idx = route_exists(destination_id); 	//check the routing table to see if I have a route
	if(route_idx != -1){
		DEBUG_PRINT("Route is known. Unicasting Data\n\r");
		struct unicast_route_table_entry route;
		route = unicast_route_table[route_idx];
		mesh_send_data(route.destination_id, route.destination_sequence_number, data,
				route.next_hop_destination_id, 0, my_id, data_length);  //uincast the packet
	} else{
		DEBUG_PRINT("Route is not known. Requesting route\n\r");

		rreq_id ++;
		mesh_send_rreq(destination_id, my_id, my_sequence_number, 0, rreq_id);  //build/send an RREQ

		//add the packet to the noroute table
		DEBUG_PRINT("Adding the request to the noroute table\n\r");
		noroute_table[noroute_table_entries].destination_id = destination_id;
		noroute_table[noroute_table_entries].data = data;
		noroute_table[noroute_table_entries].data_length = data_length;
		noroute_table_entries ++;
	}
	SX1276_Start_Receive();		//wait for a new packet
	return SUCCESS;
}

//Send a data packet
uint8_t mesh_send_data(uint32_t destination_id, uint32_t dest_seq_num, uint8_t * data, uint32_t receiver, uint8_t num_hops, uint32_t source_id, uint8_t data_length){
	DEBUG_PRINT("Building a Data Packet\n\r");
	struct data_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = receiver;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.packet_data = data;
	tosend.data_length = data_length;
	tosend.destination_sequence_number = dest_seq_num;

	//transmit the data packet
	DEBUG_PRINT("Transmitting a Data Packet to Node %d\n\r", destination_id);
	uint8_t packet_arr[DATA_BASE_PKT_LEN + data_length];
	uint8_t result = format_packet_data(tosend, packet_arr);
	result &= SX1276_Transmit_Blocking(packet_arr, DATA_BASE_PKT_LEN + data_length);
	return result;
}

//Send an RREQ packet
uint8_t mesh_send_rreq(uint32_t destination_id, uint32_t source_id, uint32_t source_sequence_number, uint8_t num_hops, uint32_t rreq_id){
	DEBUG_PRINT("Building an RREQ Packet\n\r");

	struct rreq_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.source_sequence_number = source_sequence_number;
	tosend.rreq_id = rreq_id;

	rreq_table_append(rreq_id); //don't want to see this again

	DEBUG_PRINT("Broadcasting the RREQ\n\r");

	uint8_t packet_arr[RREQ_BASE_PKT_LEN];
	uint8_t result = format_packet_rreq(tosend, packet_arr);
	result &= SX1276_Transmit_Blocking(packet_arr, RREQ_BASE_PKT_LEN);
	return result;
}

//Send an RREP packet
uint8_t mesh_send_rrep(uint32_t receiver_id, uint32_t destination_id, uint32_t source_id, uint8_t num_hops, uint32_t dest_seq_num){
	DEBUG_PRINT("Building an RREP Packet\n\r");

	struct rrep_packet tosend;
	tosend.transmitter_id = my_id;
	tosend.receiver_id = receiver_id;
	tosend.destination_id = destination_id;
	tosend.source_id = source_id;
	tosend.num_hops = num_hops;
	tosend.destination_sequence_number = dest_seq_num;

	uint8_t packet_arr[RREP_BASE_PKT_LEN];
	uint8_t result = format_packet_rrep(tosend, packet_arr);
	result &= SX1276_Transmit_Blocking(packet_arr, RREP_BASE_PKT_LEN);
	return result;
}

//Send a hello (RREQ) packet
uint8_t mesh_send_hello(){
	DEBUG_PRINT("Broadcasting a hello message\n\r");
	uint8_t result = mesh_send_rreq(0, my_id, 0, 0, rreq_id);
	return result;
}

//Unpack a data packet struct into an array for transmitting
uint8_t format_packet_data(struct data_packet packet, uint8_t packet_arr[]){
	packet_arr[0] = DATA_PACKET;
	packet_arr[1] = 0;
	packet_arr[2] = 0;
	packet_arr[3] = packet.num_hops;
	packet_arr[7] = (my_id >> 24);
	packet_arr[6] = (my_id >> 16) & 0xFF;
	packet_arr[5] = (my_id >> 8) & 0xFF;
	packet_arr[4] = (my_id) & 0xFF;

	packet_arr[11] = (packet.receiver_id >> 24);
	packet_arr[10] = (packet.receiver_id >> 16) & 0xFF;
	packet_arr[9] = (packet.receiver_id >> 8) & 0xFF;
	packet_arr[8] = (packet.receiver_id) & 0xFF;

	packet_arr[15] = (packet.destination_id >> 24);
	packet_arr[14] = (packet.destination_id >> 16) & 0xFF;
	packet_arr[13] = (packet.destination_id >> 8) & 0xFF;
	packet_arr[12] = (packet.destination_id) & 0xFF;

	packet_arr[19] = (packet.source_id >> 24);
	packet_arr[18] = (packet.source_id >> 16) & 0xFF;
	packet_arr[17] = (packet.source_id >> 8) & 0xFF;
	packet_arr[16] = (packet.source_id) & 0xFF;

	packet_arr[23] = (packet.destination_sequence_number >> 24);
	packet_arr[22] = (packet.destination_sequence_number >> 16) & 0xFF;
	packet_arr[21] = (packet.destination_sequence_number >> 8) & 0xFF;
	packet_arr[20] = (packet.destination_sequence_number) & 0xFF;

	for(int i = 0; i < packet.data_length; i++){
		packet_arr[DATA_BASE_PKT_LEN + i] = packet.packet_data[i];
	}

	return SUCCESS;
}

//unpack an rreq packet struct into an array for transmitting.
uint8_t format_packet_rreq(struct rreq_packet packet, uint8_t packet_arr[]){
	packet_arr[0] = RREQ_PACKET;
	packet_arr[1] = 0;
	packet_arr[2] = 0;
	packet_arr[3] = packet.num_hops;

	packet_arr[7] = (my_id >> 24);
	packet_arr[6] = (my_id >> 16) & 0xFF;
	packet_arr[5] = (my_id >> 8) & 0xFF;
	packet_arr[4] = (my_id) & 0xFF;

	packet_arr[11] = (packet.rreq_id >> 24);
	packet_arr[10] = (packet.rreq_id >> 16) & 0xFF;
	packet_arr[9] = (packet.rreq_id >> 8) & 0xFF;
	packet_arr[8] = (packet.rreq_id) & 0xFF;

	packet_arr[15] = (packet.destination_id >> 24);
	packet_arr[14] = (packet.destination_id >> 16) & 0xFF;
	packet_arr[13] = (packet.destination_id >> 8) & 0xFF;
	packet_arr[12] = (packet.destination_id) & 0xFF;

	packet_arr[19] = (packet.source_id >> 24);
	packet_arr[18] = (packet.source_id >> 16) & 0xFF;
	packet_arr[17] = (packet.source_id >> 8) & 0xFF;
	packet_arr[16] = (packet.source_id) & 0xFF;

	packet_arr[23] = (packet.source_sequence_number >> 24);
	packet_arr[22] = (packet.source_sequence_number >> 16) & 0xFF;
	packet_arr[21] = (packet.source_sequence_number >> 8) & 0xFF;
	packet_arr[20] = (packet.source_sequence_number) & 0xFF;

	return SUCCESS;
}

//Unpacks an rrep packet into an array for transmitting
uint8_t format_packet_rrep(struct rrep_packet packet, uint8_t packet_arr[]){
	packet_arr[0] = RREP_PACKET;
	packet_arr[1] = 0;
	packet_arr[2] = 0;
	packet_arr[3] = packet.num_hops;

	packet_arr[7] = (my_id >> 24);
	packet_arr[6] = (my_id >> 16) & 0xFF;
	packet_arr[5] = (my_id >> 8) & 0xFF;
	packet_arr[4] = (my_id) & 0xFF;

	packet_arr[11] = (packet.receiver_id >> 24);
	packet_arr[10] = (packet.receiver_id >> 16) & 0xFF;
	packet_arr[9] = (packet.receiver_id >> 8) & 0xFF;
	packet_arr[8] = (packet.receiver_id) & 0xFF;

	packet_arr[15] = (packet.destination_id >> 24);
	packet_arr[14] = (packet.destination_id >> 16) & 0xFF;
	packet_arr[13] = (packet.destination_id >> 8) & 0xFF;
	packet_arr[12] = (packet.destination_id) & 0xFF;

	packet_arr[19] = (packet.destination_sequence_number >> 24);
	packet_arr[18] = (packet.destination_sequence_number >> 16) & 0xFF;
	packet_arr[17] = (packet.destination_sequence_number >> 8) & 0xFF;
	packet_arr[16] = (packet.destination_sequence_number) & 0xFF;

	packet_arr[23] = (packet.source_id >> 24);
	packet_arr[22] = (packet.source_id >> 16) & 0xFF;
	packet_arr[21] = (packet.source_id >> 8) & 0xFF;
	packet_arr[20] = (packet.source_id) & 0xFF;

	return SUCCESS;
}

//fills a data packet struct from an array
struct data_packet unpack_packet_data(uint8_t parr[], uint8_t data_length, uint8_t data[]){
	struct data_packet packet;
	packet.num_hops = parr[3];
	packet.transmitter_id = (parr[7] << 24) | (parr[6] << 16) | (parr[5] << 8 ) | parr[4];
	packet.receiver_id = (parr[11] << 24) | (parr[10] << 16) | (parr[9] << 8 ) | parr[8];
	packet.destination_id = (parr[15] << 24) | (parr[14] << 16) | (parr[13] << 8 ) | parr[12];
	packet.source_id = (parr[19] << 24) | (parr[18] << 16) | (parr[17] << 8 ) | parr[16];
	packet.destination_sequence_number = (parr[23] << 24) | (parr[22] << 16) | (parr[21] << 8 ) | parr[20];
	packet.packet_data = &parr[DATA_BASE_PKT_LEN];
	packet.data_length = data_length;
	for(int i = 0; i < data_length; i++){
		data[i] = parr[DATA_BASE_PKT_LEN+i];
	}

	DEBUG_PRINT("\tData packet with:\n\r");
	DEBUG_PRINT("\tnum_hops = %d\n\r\ttransmitter_id=%d\n\r\treceiver_id=%d\n\r\tdestination_id=%d\n\r\tsource_id=%d\n\r", packet.num_hops, packet.transmitter_id, packet.receiver_id, packet.destination_id, packet.source_id);
	return packet;
}

//fills an rreq packet struct from an array
struct rreq_packet unpack_packet_rreq(uint8_t parr[]){
	struct rreq_packet packet;
	packet.num_hops = parr[3];
	packet.transmitter_id = (parr[7] << 24) | (parr[6] << 16) | (parr[5] << 8 ) | parr[4];
	packet.rreq_id = (parr[11] << 24) | (parr[10] << 16) | (parr[9] << 8 ) | parr[8];
	packet.destination_id = (parr[15] << 24) | (parr[14] << 16) | (parr[13] << 8 ) | parr[12];
	packet.source_id = (parr[19] << 24) | (parr[18] << 16) | (parr[17] << 8 ) | parr[16];
	packet.source_sequence_number = (parr[23] << 24) | (parr[22] << 16) | (parr[21] << 8 ) | parr[20];

	DEBUG_PRINT("\tRREQ packet with:\n\r");
	DEBUG_PRINT("\tnum_hops = %d\n\r\ttransmitter_id=%d\n\r\trreq_id=%d\n\r\tdestination_id=%d\n\r\tsource_id=%d\n\r", packet.num_hops, packet.transmitter_id, packet.rreq_id, packet.destination_id, packet.source_id);

	return packet;
}

//fills an rrep packet struct from an array
struct rrep_packet unpack_packet_rrep(uint8_t parr[]){
	struct rrep_packet packet;
	packet.num_hops = parr[3];
	packet.transmitter_id = (parr[7] << 24) | (parr[6] << 16) | (parr[5] << 8 ) | parr[4];
	packet.receiver_id = (parr[11] << 24) | (parr[10] << 16) | (parr[9] << 8 ) | parr[8];
	packet.destination_id = (parr[15] << 24) | (parr[14] << 16) | (parr[13] << 8 ) | parr[12];
	packet.destination_sequence_number = (parr[19] << 24) | (parr[18] << 16) | (parr[17] << 8 ) | parr[16];
	packet.source_id = (parr[23] << 24) | (parr[22] << 16) | (parr[21] << 8 ) | parr[20];

	DEBUG_PRINT("\tRREP packet with:\n\r");
	DEBUG_PRINT("\tnum_hops = %d\n\r\ttransmitter_id=%d\n\r\treceiver_id=%d\n\r\tdestination_id=%d\n\r\tsource_id=%d\n\r", packet.num_hops, packet.transmitter_id, packet.receiver_id, packet.destination_id, packet.source_id);

	return packet;
}

//This is THE function which handles all received packets, and implements the AODV algorithm
uint8_t receive_packet_handler(uint8_t packet_data[], uint8_t plength){
	//rand_delay();	//wait a (random) little bit to avoid collisions
	DEBUG_PRINT("Entering Packet Handler\n\r");
	uint8_t ptype = packet_type(packet_data);  //extract the packet type
	DEBUG_PRINT("\tpacket type is %d\n\r", ptype);
	//declare all potential packet types because we might need them
	if(ptype == RREQ_PACKET){
		struct rreq_packet pkt;
		pkt = unpack_packet_rreq(packet_data);
		if(!rreq_table_contains(pkt.rreq_id)) {//check if it's been seen before or if it's stale
			DEBUG_PRINT("Valid RREQ\n\r");
			rreq_table_append(pkt.rreq_id);		//add it to the rreq_table
			update_route_table(pkt.source_id, pkt.source_sequence_number, pkt.num_hops, pkt.transmitter_id);			//update my personal routing table with information about the transmitter and source
			update_route_table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
			if(pkt.destination_id == my_id){		//it's me, unicast RREP back to source
				mesh_send_rrep(pkt.transmitter_id, pkt.source_id, pkt.destination_id, 0, 0); //send an RREP packet
			} else {
				int8_t route_idx = route_exists(pkt.destination_id);
				if(route_idx != -1){		//route exists, unicast RREP back to source
					mesh_send_rrep(pkt.transmitter_id, pkt.source_id, pkt.destination_id, unicast_route_table[route_idx].hop_count, 0); //send an RREP packet
				} else{
					mesh_send_rreq(pkt.destination_id, pkt.source_id, pkt.source_sequence_number, pkt.num_hops + 1, pkt.rreq_id); //increment hop count, broadcast an RREQ
				}
			}
			if(pkt.destination_id == 0) {
				DEBUG_PRINT("This is a hello packet from node %d\n\r", pkt.source_id);
			}
		}
		//Dump the packet (do nothing)
	}
	else if(ptype == RREP_PACKET){
		struct rrep_packet pkt;
		pkt = unpack_packet_rrep(packet_data);
		if(pkt.receiver_id == my_id){		//this is actually bad, but necessary for demo
			update_route_table(pkt.source_id, 0, pkt.num_hops, pkt.transmitter_id);			//update my personal routing table with information about the transmitter and source
			update_route_table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
		}
		if(pkt.destination_id == my_id && pkt.receiver_id == my_id){  //it's intended for me! (second part necessary for demo)
			DEBUG_PRINT("RREP Packet is intended for me\n\r");
			//create a new unicast route table entry
			unicast_route_table[unicast_entries].destination_id = pkt.source_id;
			unicast_route_table[unicast_entries].destination_sequence_number = pkt.destination_sequence_number;
			unicast_route_table[unicast_entries].hop_count = pkt.num_hops;
			unicast_route_table[unicast_entries].next_hop_destination_id = pkt.transmitter_id;
			unicast_route_table[unicast_entries].precursor_nodes_destination_id_array = NULL;
			unicast_route_table[unicast_entries].expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
			unicast_entries ++;


			//look for things to send from the noroute table and send them if they match the route I just added
			DEBUG_PRINT("Searching the noroute table for blocked requests, with %d entries\n\r", noroute_table_entries);
			for(int i = 0; i < noroute_table_entries; i++){
				if(noroute_table[i].destination_id == pkt.source_id){
					DEBUG_PRINT("Found a matching entry in the noroute table\n\r");
					mesh_transmit(noroute_table[i].destination_id, noroute_table[i].data, noroute_table[i].data_length);
				}
			}
		} else if(pkt.receiver_id == my_id){	//forward it onward
			int8_t route_idx = route_exists(pkt.destination_id);
			mesh_send_rrep(unicast_route_table[route_idx].next_hop_destination_id, pkt.destination_id, pkt.source_id, pkt.num_hops + 1, 0);  //pass it along
		}
	}
	else if (ptype == DATA_PACKET){
		DEBUG_PRINT("\tReceived Data Packet\n\r");
		struct data_packet pkt;
		pkt = unpack_packet_data(packet_data, (plength-DATA_BASE_PKT_LEN), rx_data);
		if(pkt.receiver_id == my_id){		//this is actually bad, but necessary for demo
			update_route_table(pkt.source_id, 0, pkt.num_hops, pkt.transmitter_id);			//update my personal routing table with information about the transmitter and source
			update_route_table(pkt.transmitter_id, 0, 0, pkt.transmitter_id);
		}
		if(pkt.destination_id == my_id && pkt.receiver_id == my_id){
			DATA_RX_HANDLER(pkt);  				//Do something with the data
		} else if(pkt.receiver_id == my_id){
			int8_t route_idx = route_exists(pkt.destination_id);
			if(route_idx != -1){
				struct unicast_route_table_entry route;
				route = unicast_route_table[route_idx];
				mesh_send_data(route.destination_id, route.destination_sequence_number, pkt.packet_data,
						route.next_hop_destination_id, pkt.num_hops + 1, pkt.source_id, pkt.data_length);  //pass it along
			}
		} //DO NOTHING, DROP PACKET
	}
	else {
		DEBUG_PRINT("Invalid Packet Type (data[0] = %d)\n\r", packet_data[0]);
		return FAIL;
	}
	return SUCCESS;
}

//detects packet types
uint8_t packet_type(uint8_t packet_data[]){
	switch(packet_data[0] & 0xFF){
		case 0: return DATA_PACKET;
		case 1: return RREQ_PACKET;
		case 2: return RREP_PACKET;
		case 3: return RERR_PACKET;
		default: return INVALID_PACKET;
	}
	return INVALID_PACKET;
}

//adds an entry to the rreq table, to track rreqs that have already been seen
uint8_t rreq_table_append(uint32_t rreq_id){
	rreq_table[rreq_pointer] = rreq_id;
	rreq_pointer++;
	if(rreq_pointer >= RREQ_TABLE_MAX_ENTRIES) rreq_pointer = 0;
	return SUCCESS;
}

//check if the rreq table contains an element
uint8_t rreq_table_contains(uint32_t rreq_id){
	for(uint8_t i = 0; i < RREQ_TABLE_MAX_ENTRIES; i++){
		if(rreq_table[i] == rreq_id) {
			DEBUG_PRINT("RREQ Already Seen\n\r");
			return 1;
		}
	}
	return 0;
}

//check the route table to see if it contains a route to id
int8_t route_exists(uint32_t id){
	DEBUG_PRINT("\tlooking for id=%d in unicast route table\n\r", id);
	for(int i = 0; i < unicast_entries; i++){
		if(unicast_route_table[i].destination_id == id) {
			DEBUG_PRINT("\tFound entry for id=%d, next node is %d\n\r", unicast_route_table[i].destination_id, unicast_route_table[i].next_hop_destination_id);
			return i;
		}
	}
	return -1;
}

//check the route table to see if it should be updated, and add an entry if it should be
uint8_t update_route_table(uint32_t dest_id, uint32_t dest_seq_num, uint8_t num_hops, uint32_t next_hop){
	DEBUG_PRINT("checking to see if unicast table should be updated\n\r");

	//check the source
	int8_t source_idx = route_exists(dest_id);
	if(source_idx == -1){
		DEBUG_PRINT("\tSource node not found in table. Adding.\n\r");
		//create a new unicast route table entry
		unicast_route_table[unicast_entries].destination_id = dest_id;
		unicast_route_table[unicast_entries].destination_sequence_number = dest_seq_num;
		unicast_route_table[unicast_entries].hop_count = num_hops;
		unicast_route_table[unicast_entries].next_hop_destination_id = next_hop;
		unicast_route_table[unicast_entries].precursor_nodes_destination_id_array = NULL;
		unicast_route_table[unicast_entries].expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
		unicast_entries ++;
	} else {
		if(unicast_route_table[source_idx].hop_count > num_hops){
			DEBUG_PRINT("\tSource route exists, but the new route is more efficient. Replacing.\n\r");
			//create a new unicast route table entry
			unicast_route_table[source_idx].destination_id = dest_id;
			unicast_route_table[source_idx].destination_sequence_number = dest_seq_num;
			unicast_route_table[source_idx].hop_count = num_hops;
			unicast_route_table[source_idx].next_hop_destination_id = next_hop;
			unicast_route_table[source_idx].precursor_nodes_destination_id_array = NULL;
			unicast_route_table[source_idx].expiration_time = DEFAULT_ROUTE_EXPIRATION_TIME;
		}
	}

	return SUCCESS;
}

//0 - 49 ms delay
void rand_delay(){
	HAL_Delay(1);
}

//reads the device UID from the STM32, and hashes the three 32 bit words to generade one 32 bit unique id
uint32_t get_UID(){		//generates a unique 32 bit integer by hashing the unique device id
	uint32_t id1 = *((uint32_t*) 0x1FFF7A10);
	uint32_t id2 = *((uint32_t*) 0x1FFF7A14);
	uint32_t id3 = *((uint32_t*) 0x1FFF7A18);
	return id1 ^ id2 ^ id3;
}

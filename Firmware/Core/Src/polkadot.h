/*
 * header file for polkadot driver. Includes major data fields for the AODV implementation
 */
#include <stdint.h>
#include "stm32f4xx.h"
#include "packet_formats.h"

#ifndef SRC_POLKADOT_H_
#define SRC_POLKADOT_H_

	//NODE SPECIFIC DATA
	uint32_t my_id; 	//this node's ID
	uint32_t my_sequence_number; 	//this node's sequence number
	uint32_t rreq_id; 				//id for RREQ

	//Setup Data
	#define UNICAST_TABLE_LENGTH 16
	#define NOROUTE_QUEUE_MAX_ENTRIES 5
	#define ROUTE_LIFETIME 100;
	#define DEFAULT_ROUTE_EXPIRATION_TIME 1000

	//Packet types
	#define DATA_PACKET 0
	#define RREQ_PACKET 1
	#define RREP_PACKET 2
	#define RERR_PACKET 3
	#define INVALID_PACKET 0xF0;

	//Route destinations
	#define NO_ROUTE 0

	//locations for holding data
	uint8_t rx_data[256];			//Buffer for holding received data

	//route table unicast
	struct unicast_route_table_entry
	{
		uint32_t destination_id;
		uint32_t destination_sequence_number;
		uint32_t hop_count;
		uint32_t next_hop_destination_id;
		uint32_t * precursor_nodes_destination_id_array;
		uint32_t expiration_time;
	};
	uint32_t unicast_entries;

	struct unicast_route_table_entry unicast_route_table[UNICAST_TABLE_LENGTH];

	//noroute table
	struct noroute_table_entry
	{
		uint32_t destination_id;
		uint8_t * data;
		uint8_t data_length;
	};

	struct noroute_table_entry noroute_table[NOROUTE_QUEUE_MAX_ENTRIES];						//TODO, this needs to be a linkedlist
	uint8_t noroute_table_entries;

	#define RREQ_TABLE_MAX_ENTRIES 10
	uint32_t rreq_table[RREQ_TABLE_MAX_ENTRIES];
	uint8_t rreq_pointer;

	//FUNCTION PROTOTYPES
	uint8_t mesh_transmit(uint32_t destination_id, uint8_t data[], uint8_t data_length);
	uint8_t mesh_send_data(uint32_t destination_id, uint32_t dest_seq_num, uint8_t * data, uint32_t receiver, uint8_t num_hops, uint32_t source_id, uint8_t data_length);
	uint8_t mesh_send_rreq(uint32_t destination_id, uint32_t source_id, uint32_t source_sequence_number, uint8_t num_hops, uint32_t rreq_id);
	uint8_t mesh_send_rrep(uint32_t receiver_id, uint32_t destination_id, uint32_t source_id, uint8_t num_hops, uint32_t dest_seq_num);
	uint8_t mesh_unicast(uint32_t destination_id, uint32_t dest_seq_num, uint8_t * data, uint32_t receiver, uint8_t num_hops, uint32_t source_id, uint8_t data_length);
	uint8_t format_packet_data(struct data_packet packet, uint8_t packet_arr[]);
	uint8_t format_packet_rreq(struct rreq_packet packet, uint8_t packet_arr[]);
	uint8_t format_packet_rrep(struct rrep_packet packet, uint8_t packet_arr[]);
	struct unicast_packet unpack_packet_unicast(uint8_t parr[], uint8_t data_length, uint8_t data[]);
	uint8_t receive_packet_handler(uint8_t packet_data[], uint8_t plength);
	uint8_t packet_type(uint8_t packet_data[]);
	int8_t route_exists(uint32_t id);
	uint8_t DATA_RX_HANDLER(struct data_packet rx_pkt);
	uint8_t polkadot_init(uint32_t id);
	uint8_t rreq_table_contains(uint32_t rreq_id);
	uint8_t rreq_table_append(uint32_t rreq_id);
	uint8_t update_route_table(uint32_t dest_id, uint32_t dest_seq_num, uint8_t num_hops, uint32_t next_hop);
	uint8_t mesh_send_hello();
	void rand_delay();
	uint32_t get_UID();


#endif /* SRC_POLKADOT_H_ */

#ifndef CANBUS_COMPUTATIONS_H
#define CANBUS_COMPUTATIONS_H
#include <stdint.h>
#include <mcp2515.h>

uint32_t bytes_to_msg(uint8_t * b);

void msg_to_bytes(uint32_t msg, uint8_t * bytes);

uint32_t can_frame_to_msg(can_frame * frm);

uint32_t error_flags_to_msg(uint8_t canintf, uint8_t eflg);

void print_message(int number, int node, int id, int val);

void print_can_errors(uint8_t canintf, uint8_t eflg);

enum inter_core_cmds {
  //From core1 to core0: contains data read (16 bit)
  ICC_READ_DATA = 1,
  // From core0 to core1: contains data to write (16 bit)
  ICC_WRITE_DATA = 2,
  // From core1 to core0: contains regs CANINTF, EFLG
  ICC_ERROR_DATA = 3
};

#endif

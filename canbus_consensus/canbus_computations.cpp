#include "canbus_computations.h"
#include "rp2040.h"
#include <Arduino.h>

uint32_t bytes_to_msg(uint8_t * b) {
  uint32_t b0 {b[0]}, b1 {b[1]},b2 {b[2]},b3 {b[3]};
  return b0 + (b1 << 8) + (b2 << 16) + (b3 << 24);
}


void msg_to_bytes(uint32_t msg, uint8_t * bytes) {
  bytes[0] = msg; bytes[1] = (msg >> 8);
  bytes[2] = (msg >> 16); bytes[3] = (msg >> 24);
}


uint32_t can_frame_to_msg(can_frame * frm) {
  uint8_t b[4];
  b[3] = ICC_READ_DATA; b[2] = frm->can_id;
  b[1] = frm->data[1]; b[0] = frm->data[0];
  return bytes_to_msg(b);
}


uint32_t error_flags_to_msg(uint8_t canintf, uint8_t eflg) {
  uint8_t b[4];
  b[3] = ICC_ERROR_DATA; b[2] = 0;
  b[1] = canintf; b[0] = eflg;
  return bytes_to_msg(b);
}



void print_message(int number, int node, int id, int val)
{
  Serial.print(" number ");
  Serial.print( number );
  Serial.print(" at node " );
  Serial.print( node, HEX );
  Serial.print(" with id ");
  Serial.print( id, HEX );
  Serial.print(" : ");
  Serial.println( val );
}


char canintf_str[] {"MERRF | WAKIF | ERRIF | TX2IF | TX0IF | TX1IF | RX1IF | RX0IF | "};
char eflg_str [] {"RX1OV | RX0OV | TXBO | TXEP | RXEP | TXWAR | RXWAR | EWARN | "};

void print_can_errors(uint8_t canintf, uint8_t eflg) {
  Serial.println( canintf_str );
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print(" ");
    Serial.write( bitRead( canintf, bit ) ? '1' : '0' );
    Serial.print(" | ");
    }
  Serial.println(".");
  Serial.println(eflg_str);
  for (int bit = 7; bit >= 0; bit--) {
    Serial.print(" ");
    Serial.write(bitRead(eflg, bit) ? '1' : '0');
    Serial.print(" | ");
  }
  Serial.println(".");
}

#ifndef CONSENSUS_H
#define CONSENSUS_H

#include "models.h"
#include <Arduino.h>
#include "mcp2515.h"


float evaluate_cost(struct Consenso_no c_node, float d[], float rho, int vector_size);

bool check_feasibility(struct Consenso_no c_node, float d[], int vector_size);

float consensus_iterate(struct Consenso_no c_node, float rho, int size_array, float d[]);

//float consensus(struct Node nodep, float rho, int maxiter, bool data_available, uint8_t ad1, uint8_t ad2, unsigned long time_to_write, unsigned long write_delay, MCP2515::ERROR err, struct can_frame canMsgTx, struct can_frame canMsgRx, int states[3]);

#endif

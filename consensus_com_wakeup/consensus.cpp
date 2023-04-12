#include "consensus.h"

float evaluate_cost(struct Node node, float d[], float rho, int vector_size){
    float cost = 0;
    float norm = 0;
    float prod_int = 0;
    float prod_int2=0;
    for(int i = 0; i< vector_size; i++){
        prod_int += node.c[i]*d[i];
        prod_int2 += node.y[i]*(d[i]-node.d_av[i]);
        norm += (d[i]-node.d_av[i])*(d[i]-node.d_av[i]);
    }
    cost = prod_int + prod_int2+rho/2*norm;
    return cost;
}

bool check_feasibility(struct Node node, float d[], int vector_size){
    float tol = 0.001; //tolerance for rounding errors
    if (d[node.index] < 0.-tol) return false;
    if (d[node.index] > 100.0 + tol) return false;
    float d_k{0};
    float L_o_t{0};
    for(int i=0; i<vector_size;i++){
        d_k += d[i]*node.k[i];
    }
    L_o_t += node.L - node.o - tol; 
    if(d_k < L_o_t) return false;
    return true;
}


float consensus_iterate(struct Node node, float rho, int size_array, float d[]){
    float d_best[size_array];
    float cost_best{1000000};
    bool sol_unconstrained{true};
    bool sol_boundary_linear{true};
    bool sol_boundary_0{true};
    bool sol_boundary_100{true};
    bool sol_linear_0{true};
    bool sol_linear_100{true};
    float z[size_array];
    float d_u[size_array];
    float cost_unconstrained{10000};
    float cost_boundary_linear{10000};
    float cost_boundary_0{10000};
    float cost_boundary_100{1000};
    float cost_linear_0{1000};
    float d_b0[size_array]; 
    float d_bl[size_array];
    float d_b1[size_array];
    float d_l0[size_array];
    float d_l1[size_array];

    for(int i=0; i<size_array;i++){
        z[i]= rho*node.d_av[i]-node.y[i] - node.c[i];
    }
    for(int i=0; i<size_array;i++){
        d_u[i]=  (1/rho)*z[i];
    }
    sol_unconstrained = check_feasibility(node, d_u, size_array);
    if(sol_unconstrained){
        //REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
        //NO NEED TO COMPUTE THE OTHER
            cost_unconstrained = evaluate_cost(node, d_u, rho, size_array);
            if (cost_unconstrained < cost_best){
                for(int i =0; i<size_array; i++){
                   d_best[i] = d_u[i];
                }
                cost_best = cost_unconstrained;
            }
    }
    //compute minimum constrained to linear boundary
    float prod_int{0};
    for(int i =0; i<size_array; i++){
        prod_int+= z[i]*node.k[i];
    }
    for(int i =0; i<size_array;i++){
        d_bl[i] = 1/rho*z[i] - node.k[i]/node.n*(node.o - node.L + (1/rho)*prod_int);
    }
    //check feasibility of minimum constrained to linear boundary
    sol_boundary_linear = check_feasibility(node, d_bl, size_array);
    //compute cost and if best store new optimum
    if(sol_boundary_linear){
        cost_boundary_linear = evaluate_cost(node, d_bl, rho, size_array);
        if(cost_boundary_linear < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_bl[i];
            }
            cost_best = cost_boundary_linear;
        }
    }
    //compute minimum constrained to 0 boundary

    for(int i =0; i<size_array; i++){
        d_b0[i] = (1/rho)*z[i];
    }
    d_b0[node.index] = 0;
    //check feasibility of minimum constrained to 0 boundary
    sol_boundary_0 = check_feasibility(node, d_b0, size_array);
    if(sol_boundary_0){
        cost_boundary_0 = evaluate_cost(node, d_b0, rho, size_array);
        if(cost_boundary_0 < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_b0[i];
            }
            cost_best = cost_boundary_0;
        }
    }
    //compute minimum constrained to 100 boundary
    for(int i =0; i<size_array; i++){
        d_b1[i] = (1/rho)*z[i];
    }
    d_b1[node.index] = 100;
     //check feasibility of minimum constrained to 100 boundary
    sol_boundary_100 = check_feasibility(node, d_b1, size_array);
    if(sol_boundary_100){
        cost_boundary_100 = evaluate_cost(node, d_b1, rho, size_array);
        if(cost_boundary_100 < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_b1[i];
            }            
            cost_best = cost_boundary_100;
        }
    }
    //compute minimum constrained to linear and 0 boundary
    float sub{0};
    for(int i=0;i<size_array;i++){
        sub+=z[i]*node.k[i];
    }
    for(int i =0; i<size_array; i++){
        d_l0[i] = (1/rho)*z[i] - (1/node.m)*node.k[i]*(node.o - node.L) + (1/rho/node.m)*node.k[i]*(node.k[node.index]*z[node.index]- sub);
    }
    d_l0[node.index] = 0;
    //check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(node, d_l0, size_array);
    //compute cost and if best store new optimum
    if(sol_linear_0){
        cost_linear_0 = evaluate_cost(node,d_l0,rho,size_array);
        if(cost_linear_0 < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_l0[i];
            }
            cost_best = cost_linear_0;
        }
    }
    // compute minimum constrained to linear and 100 boundary
    for(int i =0; i<size_array; i++){
        d_l1[i] = (1/rho)*z[i] - (1/node.m)*node.k[i]*(node.o - node.L + 100*node.k[node.index]) + (1/rho/node.m)*node.k[i]*(node.k[node.index]*z[node.index]- sub);
    }
    d_l1[node.index] = 100;
    //check feasibility of minunum constrained to linear and 0 boundary
    sol_linear_100 = check_feasibility(node, d_l1, size_array);
    //compute cost and if best store new optimum
    if(sol_linear_100){
        cost_linear_0 = evaluate_cost(node,d_l1,rho, size_array);
        if(cost_linear_0 < cost_best){
            for(int i =0; i<size_array; i++){
              d_best[i] = d_l1[i];
            }
            cost_best = cost_linear_0;
        }
    }
    for(int i =0; i<size_array; i++){
       d[i] = d_best[i];
    }
    return cost_best;
}


// float consensus(struct Node nodep, float rho, int maxiter, bool data_available, uint8_t ad1, uint8_t ad2, unsigned long time_to_write, unsigned long write_delay, MCP2515::ERROR err, struct can_frame canMsgTx, struct can_frame canMsgRx, int states[3]){
//   float d1[3]{0,0,0};
//   float d2[3]{0,0,0};
//   float d3[3]{0,0,0};
//   for(int i =0; i<maxiter; i++){
//     //minimize augmented lagragian with respect to di for the current values of yi and d_av
//     float cost1 = consensus_iterate(nodep, rho, 3, d1);
//     for(int j=0; j<3; j++){
//       nodep.d[j] = d1[j];
//     }
//     //exchanges solution 

//     for(int track_state = 0; track_state<3;track_state++){
//       int msg_sent{0};
//       int msg_received1{0};
//       int msg_received2{0};
//       while(msg_sent < 3 && msg_received1 <3 && msg_received2 <3){
//         if(millis() >= time_to_write && msg_sent < 3 && states[track_state] == 1 ) {// && (send_ack1 == false || send_ack2 == false) && msg_sent < 3 ){
//             //creates the message
//             unsigned long message = nodep.d[msg_sent];
//             canMsgTx.can_id = node_address;
//             canMsgTx.can_dlc = 5;
//             uint8_t command = 0b11100;
//             uint8_t node_pos;
//             if(msg_sent == 0) node_pos= 0x0;
//             else if(msg_sent == 1) node_pos = 0x1;
//             else if (msg_sent == 2) node_pos = 0x2;
//             uint8_t response = 0x0;
//             uint8_t iteration = (byte)i;
//             int int_b_value = nodep.d[msg_sent]*1000;
//             //divide into the proper bytes to transmit
//             uint8_t b_value_1 = int_b_value >> 9; 
//             uint8_t b_value_2 = int_b_value >> 1;
//             uint8_t b_value_3 = (int_b_value & 0x1) << 7;
//             canMsgTx.data[0] = ( (node_pos << 6))|((response << 5)) |(command);
//             canMsgTx.data[1] = iteration;
//             canMsgTx.data[2] = b_value_1;
//             canMsgTx.data[3] = b_value_2;
//             canMsgTx.data[4] = b_value_3;
//             //sends the message
//             noInterrupts();
//             err = can0.sendMessage(&canMsgTx);
//             interrupts();
//             // Serial.print("Sending message ");
//             // Serial.print( msg_sent );
//             // Serial.print(" from node ");
//             // Serial.print( node_address, HEX );
//             // Serial.print(" for iteration ");
//             // Serial.println(i);
//             // for (int k=0 ; k < canMsgTx.can_dlc ; k++)
//             //   Serial.print( canMsgTx.data[ k ], BIN), Serial.print(" ");
//             // Serial.println();
//             msg_sent++;
//             time_to_write = millis() + write_delay;
//         }

//         if(data_available && states[track_state] == 0){ //recebeu mensagem e esta no estado de receber
//           noInterrupts();
//           can_frame frm {canMsgRx}; //local copy
//           interrupts();
//           //receives data from different node
//           if((frm.data[0] & 0b00111111) == 0b00011100 && frm.can_dlc == 5){
//             if((int)frm.data[1] == i){//same iteration
//                 float value = ((uint32_t) (frm.data[2] <<  9) | (uint32_t) (frm.data[3]  << 1) | (uint32_t) (frm.data[4]  >> 7))/1000.0; 
//                 int idx = frm.data[0] >> 6;
//                             //    Serial.println();Serial.print("Iteracao = "); Serial.println(idx); Serial.println();
//                 if(frm.can_id == ad1){
//               //    Serial.println("Recebeu dado no 1");
//                   d2[idx] = value;
//                   msg_received1++;
//                 }
//                 else if(frm.can_id == ad2){
//                 //  Serial.println("Recebeu dado no 2");
//                   d3[idx] = value;
//                   msg_received2++;
//                 }
//             }
//           }
//         data_available = false;
//         }
//       }
//     }
//   //compute the average of each Node's solution
//   for(int j=0; j<3; j++){
//       nodep.d_av[j] = (nodep.d[j]+d2[j]+d3[j])/3;
//     }
//   //update the Lagrange multipliers
//     for(int j=0; j<3; j++){
//       nodep.y[j] = nodep.y[j] + rho*(nodep.d[j]-nodep.d_av[j]);
//     }
//   }
//   //solucao dos duty cycles
//   Serial.println("Solucao final dos duty cycles: ");
//     for(int j=0;j<3;j++){
//       Serial.println(nodep.d_av[j]);
//     }

//   //solucao que indica a referencia que cada no precisa seguir
//   float reference = 0;
//   //mulitiplicar K*d+o
//   reference = nodep.k[0]*nodep.d_av[0] + nodep.k[1]*nodep.d_av[1] + nodep.k[2]*nodep.d_av[2] + nodep.o; 
//   Serial.println();
//   Serial.print("Referencia final do no:  ");
//   Serial.println(reference);
//   return reference;  
// }

struct nodes{
    int index;
    float d[2];
    float d_av[2];
    float y[2];
    float k[2];
    float n;
    float m;
    float c[2];
    float o;
    float L;
};

float evaluate_cost(struct nodes node, float d[], float rho, int vector_size){
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

bool check_feasibility(struct nodes node, float d[], int vector_size){
    float tol = 0.001; //tolerance for rounding errors
    if (d[node.index] < 0-tol) return false;
    if (d[node.index] > 100 + tol) return false;
    float d_k{0};
    float L_o_t{0};
    for(int i=0; i<vector_size;i++){
        d_k += d[i]*node.k[i];
    }
    L_o_t += node.L - node.o - tol; 
    if(d_k < L_o_t) return false;
    return true;
}

float consensus_iterate(struct nodes node, float rho, int size_array, float d[]){
    float d_best[2]{-1,-1};
    float cost_best{1000000};
    bool sol_unconstrained{true};
    bool sol_boundary_linear{true};
    bool sol_boundary_0{true};
    bool sol_boundary_100{true};
    bool sol_linear_0{true};
    bool sol_linear_100{true};
    float z[2] {0,0};
    float d_u[2] {0,0};
    float cost_unconstrained{10000};
    float cost_boundary_linear{10000};
    float cost_boundary_0{10000};
    float cost_boundary_100{1000};
    float cost_linear_0{1000};
    float d_b0[2]; 
    float d_bl[2];
    float d_b1[2];
    float d_l0[2];
    float d_l1[2];

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

//case of test
float L1 = 150;
float o1 = 30;
float L2 = 80;
float o2 = 0;

float c1 = 1; float c2=1;

float rho = 0.07;
const int maxiter = 50;

//system calibration parameters
float k11{2}, k12{0.5}, k21{0.5}, k22{2};

//ignore centralized solution

//history of distrbuting solution
float d11[maxiter]{0}, d12[maxiter]{0}, d21[maxiter]{0}, d22[maxiter]{0}, av1[maxiter]{0}, av2[maxiter]{0};

//distributed node initialization
struct nodes node1 = {{0},{0,0},{0,0},{0,0},{k11,k12},{4.25},{0.25},{c1,0},{o1},{L1}};
struct nodes node2 = {{1},{0,0},{0,0},{0,0},{k21,k22},{4.25},{0.25},{0,c2},{o2},{L2}};

float l[2]{0,0};
float d[2]{0,0};


//variaveis de teste
float teste_arr[2]{0,0};
//bool testand = consensus_iterate(node1,rho,2,teste_arr);
float d1[2]{0,0};

void setup(){
  d11[0] = node1.d[0];
  d12[0] = node1.d[1];
  d21[0] = node2.d[0];
  d22[0] = node2.d[1];
  av1[0] = (d11[0]+d21[0])/2;
  av2[0] = (d12[0]+d22[0])/2;
  for(int i =1; i<maxiter; i++){
    //computation of the primal solutions
    //node 1
    float cost1 = consensus_iterate(node1, rho, 2, d1);
    for(int j=0; j<2; j++) node1.d[j] = d1[j];

    //node 2
    float d2[2]{0,0};
    float cost2 = consensus_iterate(node2, rho, 2, d2);
    for(int j=0; j<2; j++){ node2.d[j] = d2[j];}
    
    for(int j=0; j<2; j++){
      node1.d_av[j] = (node1.d[j]+node2.d[j])/2;
      node2.d_av[j] = (node1.d[j]+node2.d[j])/2;
    }
    //compuation of the lagrangian updates
    for(int j=0; j<2; j++){
      node1.y[j] = node1.y[j] + rho*(node1.d[j]-node1.d_av[j]);
      node2.y[j] = node2.y[j] + rho*(node2.d[j]-node1.d_av[j]);
    }

  d11[i] = node1.d[0];
  d12[i] = node1.d[1];
  d21[i] = node1.d[0];
  d22[i] = node1.d[1];
  av1[i] = (d11[i]+d21[i])/2;
  av2[i] = (d12[i]+d22[i])/2;
  }

  for(int i=0;i<2;i++){
    d[i] = node1.d_av[i];
  }
  //mulitiplicar K*d+o
  l[0] = k11*node1.d_av[0] + k12*node1.d_av[1] + o1;
  l[1] = k21*node1.d_av[0] + k22*node1.d_av[1] + o2;
}

void loop(){
   Serial.print("l = ");Serial.print(l[0]); Serial.print(" "); Serial.println(l[1]);
  Serial.print("d = "); Serial.print(d[0]);Serial.print(" ");Serial.println(d[1]);
}

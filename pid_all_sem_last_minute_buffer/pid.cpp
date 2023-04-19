#include "pid.h"

pid::pid( int select_)
  // member variable initialization list
  : select{select_}, h {1}, N {10}, Tt{5}, I {0.0}, D {0.0}, b{0.0}, y_old{0.0}, bi{0.0}, ad{0.0}, bd{0.0}, ao{0.5}, v{0.0}, u{0.0}, Ti {4.8}, Td {0}, K{0.0}, in_transition{false}
{ 
  if (select == 1){
    K_baixo = 0.43;
    b_baixo = 30;
    K_medio = 0.38;
    b_medio = 8;
    K_alto = 0.33;
    b_alto = 10;
  }

  else if (select == 2){
    K_baixo = 0.45;
    b_baixo = 30;
    K_medio = 0.40;
    b_medio = 6;
    K_alto = 0.32;
    b_alto = 10;
  }
  
  else{ //select = 3
    K_baixo = 0.55;
    b_baixo = 30;
    K_medio = 0.43;
    b_medio = 8;
    K_alto = 0.35;
    b_alto = 9;
  }

  K_old = K_baixo;
  b_old = b_baixo;
} // should check arguments validity

float pid::compute_control( float r, float y) {
  //control
  if(in_transition){
    in_transition = false;
    I = I + K_old*(b_old*r-y)-K*(b*r-y); 
  }
  float P = K*(b*r-y);
  D = ad*D-bd*(y-y_old);
  v = P+I+D;
  if( v < 0 ) u = 0;
  else if( v > 4095 ) u = 4095;
  else u = v;
  return u;
}

void pid::compute_coefficients(float r, bool r_change){
    if(Ti!=0) bi = K*h/Ti;
    else bi=0;
    
    if((Td+N*h) == 0){
      ad = 0;
      bd = 0;}
    else{
      ad = Td/(Td+N*h);
      bd = K*N*Td/(Td+N*h);
    }
    
    if(Tt != 0 ) ao = h/Tt;
    else ao = 0;

    if(r_change){
      in_transition = true;
      K_old = K;
      b_old = b;}
    
    if(r <=45){K = K_baixo; b = b_baixo;}
    else if((r>45) && (r<=120)){K = K_medio; b = b_medio;}
    else{K = K_alto; b = b_alto;} 
}

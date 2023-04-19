#ifndef PID_H
#define PID_H

class pid {
    float I, D, Ti, Td, h, y_old, N, Tt, K_old, b_old, bi, ad, bd, ao, K_baixo, b_baixo, K_medio, b_medio, K_alto, b_alto, y, u, v, b, K;
    int select;
    bool in_transition;
    public:
        explicit pid(int select_ = 1);
        ~pid() {};
        float compute_control( float r, float y);
        void housekeep( float r, float y, bool antiwindup_on);
        void compute_coefficients(float r, bool r_change);

        void resetPID(){
            I = 0;
        };
};

inline void pid::housekeep( float r, float y, bool antiwindup_on) {
    float e = r - y;
    y_old = y;
    b_old = b;
    if (antiwindup_on) I += bi*(r-y)+ao*(u-v);
    else I += bi*(r-y);
}


#endif //PID_H 

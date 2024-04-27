#ifndef COORDINATE_TRANSFORM_HPP
#define COORDINATE_TRANSFORM_HPP

#include "type_definition.hpp"

class CoordTf{

    public:

        CoordTf();

        void setPos(mat31& q_pan, mat31& q_lift);

        void getWheelCenter(int i, mat43& b_p_wc_i);

        void TfSE3(mat33& R, mat31& t, mat44& T);

        void rotPan(double q_pan_i, mat33& b_R_p);

        void rotLift(double q_lift_i, mat33& p_R_l);

    private:
        mat14 vec_aug;
        mat33 b_t_pan;
        mat33 Eye;
        mat41 lift_p_leg;
        mat31 b_t_c;
        mat31 t0;

        mat31 q_pan_;
        mat31 q_pan_init;
        mat31 q_lift_;


};



#endif
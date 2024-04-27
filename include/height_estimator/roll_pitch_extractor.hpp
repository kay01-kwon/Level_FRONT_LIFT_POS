#ifndef ROLL_PITCH_EXTRACTOR_HPP
#define ROLL_PITCH_EXTRACTOR_HPP

#include <iostream>
#include "type_definition.hpp"

using std::cout;
using std::endl;

class RollPitchExtr{

    public:

        // Constructor
        RollPitchExtr();

        // Convert quaternion to rotation matrix
        void quat2rotm(quat &q);

        // Get roll and pitch except yaw
        void get_roll_pitch(double* roll, double* pitch);

    private:


        void quat2q_vec(quat& q, mat31& q_vec);

        void q_vec2skiew_sym(mat31& q_vec, mat33& skiew_sym);
        

        double roll_;
        double pitch_;

        mat33 rotm_;

        mat33 eyem_;

};



#endif
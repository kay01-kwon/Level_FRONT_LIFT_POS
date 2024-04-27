#include "roll_pitch_extractor.hpp"

RollPitchExtr::RollPitchExtr()
{
    roll_ = 0.0;
    pitch_ = 0.0;

    eyem_ << 1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0;
    cout<<"Constructor"<<endl;
}

void RollPitchExtr::quat2rotm(quat &q)
{
    mat31 q_v;
    mat33 skiew_sym;

    quat2q_vec(q, q_v);
    q_vec2skiew_sym(q_v, skiew_sym);

    rotm_ = (q.w()*q.w()-q_v.transpose()*q_v)*eyem_ 
    + 2*q_v*q_v.transpose()
    + 2*q.w()*skiew_sym;

}

void RollPitchExtr::get_roll_pitch(double* roll, double* pitch)
{
    *pitch = -asin(rotm_(2,0));
    *roll = atan2(rotm_(2,1)/cos(*pitch),
    rotm_(2,2)/cos(*pitch));
}

void RollPitchExtr::quat2q_vec(quat &q, mat31& q_vec)
{
    q_vec << q.x(), q.y(), q.z();
}

void RollPitchExtr::q_vec2skiew_sym(mat31& q_vec, mat33& skiew_sym)
{
    skiew_sym << 0.0, -q_vec(2), q_vec(1),
                q_vec(2), 0.0, -q_vec(0),
                -q_vec(1), q_vec(0), 0.0;
}
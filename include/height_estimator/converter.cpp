#include "converter.hpp"

Converter::Converter()
{
    offset_ << 0, 0, 0;
}

Converter::Converter(mat31& offset):offset_(offset)
{
    cout<<offset_<<endl;
}

double Converter::convert_actaul2q_pan(int32_t inc)
{
    return inc*inc2deg_PAN;
}

double Converter::convert_actual2q_lift(int32_t inc, double offset_i)
{
    return -(inc*inc2deg_LIFT) + offset_i;
}

int32_t Converter::convert_q_lift_des2inc(double q_lift_des, double offset)
{
    return -(q_lift_des - offset)*deg2target_LIFT;
}
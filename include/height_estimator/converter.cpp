#include "converter.hpp"

Converter::Converter()
{

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
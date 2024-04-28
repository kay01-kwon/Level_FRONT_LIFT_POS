#ifndef CONVERTER_HPP
#define CONVERTER_HPP

#include "type_definition.hpp"
#include "motor_info.hpp"

class Converter{

    public:

        Converter();
        Converter(mat31& offset);

        double convert_actaul2q_pan(int32_t inc);
        double convert_actual2q_lift(int32_t inc, double offset_i);
        int32_t convert_q_lift_des2inc(double q_lift_des, double offset_i);
    private:

        mat31 offset_;


};


#endif
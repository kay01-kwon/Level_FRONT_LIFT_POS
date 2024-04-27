#include "coordinate_transform.hpp"

CoordTf::CoordTf()
{
    double th1, th2,l,c;
    th1 = 2.0*M_PI/3.0;
    th2 = -2.0*M_PI/3.0;
    l = 0.400;
    c = 0.170;

    Eye<<1, 0, 0,
    0, 1, 0,
    0, 0, 1;

    vec_aug << 0, 0, 0, 1;

    b_t_pan << 0.2, 0.2*cos(th1), 0.2*cos(th2),
    0, 0.2*sin(th1), 0.2*sin(th2),
    0, 0, 0;

    lift_p_leg << 0, 0, -l, 1;
    b_t_c << 0, 0, -c;
    t0 << 0, 0, 0;

    q_pan_init << 0, -2*M_PI/3.0, 2*M_PI/3.0;

}

void CoordTf::setPos(mat31& q_pan, mat31& q_lift)
{
    q_pan_ = q_pan*M_PI/180.0 + q_pan_init;
    q_lift_ = q_lift*M_PI/180.0;

    cout<<"Pan pos: ";
    for(int i = 0; i < 3; i++)
        cout<<(q_pan_(i)-q_pan_init(i))*180.0/M_PI<<", ";

    cout<<"\n";

    cout<<"Lift pos: ";
    for(int i = 0; i < 3; i++)
        cout<<q_lift_(i)*180.0/M_PI<<", ";

    cout<<"\n";

}

void CoordTf::getWheelCenter(int i, mat43& b_p_wc)
{
    mat44 b_T_c, b_T_p, p_T_l;
    mat44 b_T_l;
    mat33 b_R_p, p_R_l;
    mat31 b_t_pan_i;
    mat41 b_p_wl_i;

    b_t_pan_i = b_t_pan.col(i);

    rotLift(q_lift_(i), p_R_l);
    rotPan(q_pan_(i), b_R_p);

    TfSE3(p_R_l, t0, p_T_l);
    TfSE3(b_R_p, b_t_pan_i, b_T_p);
    TfSE3(Eye, b_t_c, b_T_c);

    b_p_wl_i = b_T_p*p_T_l*lift_p_leg;
    b_p_wc.col(i) = b_T_c*b_p_wl_i;
}

void CoordTf::TfSE3(mat33& R, mat31& t, mat44& T)
{
    T << R, t, vec_aug;

    // cout << "T: " << T << "\n"; 
}

void CoordTf::rotPan(double q_pan_i, mat33& b_R_p)
{
    double cp, sp; 
    cp = cos(q_pan_i);
    sp = sin(q_pan_i);
    
    b_R_p << cp, sp, 0,
        -sp, cp, 0,
        0, 0, 1;

}

void CoordTf::rotLift(double q_lift_i, mat33& p_R_l)
{
    double cl, sl;
    cl = cos(q_lift_i);
    sl = sin(q_lift_i);

    p_R_l << cl, 0, -sl,
            0, 1, 0,
            sl, 0, cl;

}

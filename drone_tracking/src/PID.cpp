#include <iostream>
#include <PID.h>

PID::PID(float kp, float ki, float kd, float dt, float target, float current)
{
    float pre_error = 0.0;
    float pre_ierror = 0.0;

    setVals(kp, ki, kd, dt);
    
    calcPID(target, current, pre_error, pre_ierror);
}

void PID::setVals(float kp, float ki, float kd, float dt)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _dt = dt;
}

void PID::calcPID(float target, float current, float pre_error, float pre_ierror)
{
    //ROS_INFO("calculating");
    //target x and target y is the position relative to quad 
    float error = target;
    
    // //stop adjusting if error is less than tolerance
    // if (error <= 0.25)
    // {   
    //     _PID = 0.0;          
    // }

    float Pgain = _kp * error;
    float int_error = pre_ierror + ((error + pre_error)/ 2) * _dt;
    float Igain = _ki * int_error;
    
    float der_error = (error - pre_error) / _dt;
    float Dgain = _kd * der_error;

    float PID = Pgain + Igain + Dgain;
    
    //set gain constraint to prevent the guy from going to crazy
    const float gain_constraint = 3.5;

    if (PID >= gain_constraint)
        {
            PID = gain_constraint;
        }

    if (PID <= -gain_constraint)
        {
            PID = -gain_constraint;
        }
        
    //save error
    pre_error = error;
    pre_ierror = int_error;

    _PID = PID;
}
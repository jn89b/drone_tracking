#include <iostream>

#include <math.h>
#include <moving_avg.h>

MovingAverage::MovingAverage(float x, float y)
{
    init_vals(x,y);
    //kfx_avg = compute_avg(x);
    //std::cout<< "avg_x" << kfx_avg << std::endl;
    //kfy_avg = compute_avg(y);
    //_avg_mag = sqrt(pow(kfx_avg,2) + pow(kfy_avg,2));
}

void MovingAverage::init_vals(float x, float y)
{
    kf_x = x;
    kf_y = y;

    kfx_avg = 0.0;
    kfy_avg = 0.0;
    
    running_total = 0.0;
}


float MovingAverage::compute_avg(float input)
{   
    //check if buffer queue is full
    if (buffer.size() == window_size)
    {   
        //remove front value from total summation
        running_total -= buffer.front();
        //pop off the front 
        buffer.pop();
    }
    //add new value in
    buffer.push(input);
    //recalcuate sum 
    running_total += input;
    //computer average
    return static_cast<double>(running_total/ buffer.size());
}

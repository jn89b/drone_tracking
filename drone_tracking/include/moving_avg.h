#ifndef NULL_MOVING_AVG_H
#define MOVING_AVG_H

#include <queue>

class MovingAverage
{
    private:
        
        float kf_x;
        float kf_y;
        float kf_mag;
        
        float kfx_avg;
        float kfy_avg;
                
        float running_total;
        
        const int window_size = 100;
        std::queue<int> buffer;
        float _avg_mag;

    public:
        MovingAverage(float x, float y); 

        void init_vals(float x, float y);
        float compute_avg(float input);

        //float compute_avg(float x);
        //float compute_avg(float y);        

        float get_kfx(){return kfx_avg;}
        float get_kfy(){return kfy_avg;}

        float compute_mag(float x, float y);

        float get_avg_mag(){return _avg_mag;}   
};

#endif
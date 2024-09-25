#ifndef SEED_Lab_PID_controller.h
#define SEED_Lab_PID_controller.h

class PID_control{

    private:
        float desired_left_vel;
        float desired_right_vel;

    public:
        float get_right_vel();
        void set_right_vel(long *desired_pos, long *measured_pos, float *desired_velocity);
        
};


#endif
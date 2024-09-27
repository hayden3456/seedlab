#ifndef SEED_Lab_PID_controller.h
#define SEED_Lab_PID_controller.h

class PID_control{

    private:
        float desired_left_vel;
        float desired_right_vel;
        

    public:
        float get_des_vel(int motor_num);
        void set_right_vel(long *desired_pos, float measured_pos = get_wheel_pos(0), float *desired_right_vel);
        void set_right_vel(long *desired_pos, float measured_pos = get_wheel_pos(1), float *desired_left_vel);

};


#endif

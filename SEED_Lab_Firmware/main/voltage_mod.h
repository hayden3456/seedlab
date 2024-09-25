#ifndef voltage_mod_h
#define voltage_mod_h


class velocity_control{
    private:
        float measured_left_vel;
        float left_voltage;
        float left_dir;
        float left_prev_pos;

        float measured_right_vel;
        float right_voltage;
        float right_dir;
        float right_prev_pos;

    public:
        void left_voltage_set(float desired_left_vel, float measured_left_vel);
        void left_velocity_set(float left_pos, float left_prev_pos);
        float left_voltage_get();
        float measured_left_vel_get();
        float left_dir_get();
        
        void right_voltage_set(float desired_right_vel, float measured_right_vel);
        void right_velocity_set(float right_pos, float right_prev_pos);
        float right_voltage_get();
        float measured_right_vel_get();
        float right_dir_get();
};

#endif
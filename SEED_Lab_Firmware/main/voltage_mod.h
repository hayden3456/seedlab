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
        void set_left_voltage(float desired_left_vel, float measured_left_vel);
        void set_measured_left_vel(float left_pos, float left_prev_pos);
        void set_right_voltage(float desired_right_vel, float measured_right_vel);
        void set_measured_right_vel(float right_pos, float right_prev_pos);

        float get_voltage(int wheel);
        float get_measured_vel(int wheel);
        float get_dir(int wheel);
}
#endif

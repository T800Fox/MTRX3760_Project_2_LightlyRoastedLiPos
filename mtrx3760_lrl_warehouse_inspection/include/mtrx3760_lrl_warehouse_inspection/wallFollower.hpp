#ifndef MTRX3760_LRL_WALL_FOLLOWER_HPP_
#define MTRX3760_LRL_WALL_FOLLOWER_HPP_

//Include actuator goal type
#include <array>
#include <deque>
#include <fstream>
#include <cmath>
#include <fstream>
#include <cstdint>

#include "mtrx3760_lrl_actuator/actuator_client.hpp"
#include "utils.hpp"


const double POSE_EQUAL_THRESH = 0.15;


enum DIRECTION {
    BACKWARD,
    RIGHT,
    FORWARD,
    LEFT,
};

enum STATE {
    CONTACT_WALL,
    AWAIT_LOGIC,
    ROT_RIGHT_CORNER,
};

const double LIDAR_DT = 1.0/5; // LIDAR update-rate is 5HZ (from .sdf file)
const double CORNER_OFFSET = 0.15; //Just below threshold distance to ensure wall is registered
const double STABLE_VEL = 1.0; //Linear velocity at which odom is stable (no slip)




class wallFollower {
    
    public:
        wallFollower();
        ~wallFollower();

        ActuatorCmd determine_cmd(std::array<bool,4> wall_pres, std::array<float,4> distance, std::string& debug_buf);
        ActuatorCmd calc_refinement(std::array<float,4> distance, std::string& debug_buf);

        double query_roc();

    private:
        // Member variables
        bool prev_at_init_pose;
        bool awaiting_loop_jump;
        double curr_angle;
        std::array<bool, 4> prev_wall_pres; //Presence of wall in each 90deg direction (at previous reading)

        Point prev_seg_end_point;
        std::vector<LineSeg> traversed_segs; //Array of line-segments that have been travesered already (for identifying foreign loops)
        double prev_right_dist; //Distance to right at previous update (for determining wall_postiion)
        
        bool apply_refinement;
        uint64_t moving_avr_window;
        std::deque<double> moving_avr_buf; //Buffer of distances for angular refinement (deqeue allows efficient itteration and push/pop)
        double old_avr; //Moving average at previous update for calculating roc

        STATE state; //High-level navigation state
        int move_index;

        Pose2D curr_pose;
        Pose2D loop_init_pose;

        double avr_roc;

};


#endif
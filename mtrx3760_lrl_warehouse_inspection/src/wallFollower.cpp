#include "mtrx3760_lrl_warehouse_inspection/wallFollower.hpp"



wallFollower::wallFollower(){
    // Initialise member variables
    awaiting_loop_jump = false; //State flag for whether robot is searching for foreign loop to jump to (after loop completed)

    prev_at_init_pose = true;
    prev_right_dist = 0.0;

    state = CONTACT_WALL; //Initial state where robot gains contact with wall

    //Assign previous wall_pres
    prev_wall_pres.fill(false);

    //Moving average of wall-dist to refine angle (odom innaccuracy)
    moving_avr_window = 3;
    old_avr = 0.0;

    avr_roc = 0.0;

}

wallFollower::~wallFollower(){}


ActuatorCmd wallFollower::determine_cmd(std::array<bool,4> wall_pres, std::array<float,4> distance, std::string& debug_buf){
    apply_refinement = false;

    ActuatorCmd actuator_cmd;

    //Lambda actuator functions
    auto set_vel = [&actuator_cmd] (double vel){
        actuator_cmd.mode = actuator_cmd.MODE_VEL_LINEAR;
        actuator_cmd.magnitude = vel;
    };

    auto move_dist = [&actuator_cmd] (double dist){
        actuator_cmd.mode = actuator_cmd.MODE_ABS_LINEAR;
        actuator_cmd.magnitude = dist;
    };

    auto rotate_ang = [&actuator_cmd] (double angle){
        actuator_cmd.mode = actuator_cmd.MODE_ABS_ANGULAR;
        actuator_cmd.magnitude = -angle * 3.1415926/180.0;
    };


    switch (state) {
        case CONTACT_WALL: {
            if (wall_pres[FORWARD]){
                //Wall is in contact - rotate to align with wall then enter main logic state
                debug_buf.append("Contacted wall - turning!!\n");

                prev_at_init_pose = true; //Flag to avoid instant loop-completion
                loop_init_pose = curr_pose;
                loop_init_pose.theta += 90.0; //True starting pose is aligned with wall (90deg offset)
                //RCLCPP_INFO(this->get_logger(), "First contact with loop! starting pose x: %.2f y: %.2f theta: %.2f", 
                //                                   loop_init_pose.pos.x, loop_init_pose.pos.y, loop_init_pose.theta);

                //Project forward distance in robot's direction to get global wall_pos
                prev_seg_end_point = curr_pose.pos + project_point(curr_pose.theta, distance[FORWARD]);


                rotate_ang(90.0); //Rotate left 90deg (stops auto)

                
                state = AWAIT_LOGIC;

            } else{
                debug_buf.append("Driving forwards... no wall detected\n");

                set_vel(0.15);
            
            }
            break;
        }

        case AWAIT_LOGIC: {
            bool corner_flag = true; //Flag for whether a corner-turn command has occured
            Point seg_end_point;

            if (!wall_pres[RIGHT]){
                //RCLCPP_INFO(this->get_logger(), "Right corner!!");
                set_vel(0.0);

                //Turn right-corner
                //Enter movement sequence
                state = ROT_RIGHT_CORNER;
                move_index = 0;

                //Calculate global pos of traveresed-wall's end (current right-dist is invalid on right turn so prev-update must be used)
                seg_end_point = curr_pose.pos + project_point(curr_pose.theta + M_PI/2, prev_right_dist);


            } else if (wall_pres[FORWARD]){
                //RCLCPP_INFO(this->get_logger(), "Left corner!!");
                //RCLCPP_INFO(this->get_logger(), "wall_dist callback fired: forward: %.2f, right: %.2f, backward: %.2f, left: %.2f ", msg->distance[FORWARD], msg->distance[RIGHT], msg->distance[BACKWARD], msg->distance[LEFT]);  // STUB
                //RCLCPP_INFO(this->get_logger(), "pres: forward: %d, right: %d, backward: %d, left: %d ", msg->wall_pres[FORWARD], msg->wall_pres[RIGHT], msg->wall_pres[BACKWARD], msg->wall_pres[LEFT]);  // STUB

                rotate_ang(90.0); //Rotate left 90deg (stops auto)

                //Calculate global pos of traversed-wall's end
                seg_end_point = curr_pose.pos + project_point(curr_pose.theta, distance[FORWARD]) + 
                                            project_point(curr_pose.theta + M_PI/2, distance[RIGHT]);


            } else{
                //Moving forwards:
                //RCLCPP_INFO(this->get_logger(), "Moving forwards!!");
                set_vel(0.15); 

                //Flag for later called refinement method
                apply_refinement = true;

                //Check if loop has been completed
                bool return_prev_flag = false;
                bool at_init_pose = dist2D(curr_pose.pos, loop_init_pose.pos) < POSE_EQUAL_THRESH;
                if (at_init_pose && !prev_at_init_pose){
                    //Loop has been completed - wait for foreign loop to jump to (disregard first flag)
                    //RCLCPP_INFO(this->get_logger(), "Loop completed!!");
                    
                    //If already awaiting jump at loop completion, no foreign loop is in view of current loop
                    //Return to previous loop (CONTACT_WALL)
                    if (awaiting_loop_jump){
                        return_prev_flag = true;
                    } else{
                        awaiting_loop_jump = true; 
                    }
                }

                corner_flag = false;
                prev_at_init_pose = at_init_pose;

                
                //Search left LIDAR probe for foreign loop
                //If projected point can't be found in existing segment array, loop is foreign
                if (awaiting_loop_jump){
                    //Calculate global point of left facing lidar-probe
                    Point left_proj = project_point(curr_pose.theta - M_PI/2, distance[LEFT]);

                    
                    bool is_foreign_loop;
                    double temp, min_dist = INFINITY;

                    //Ignore if projected point is inf - out of lidar range 
                    if (left_proj.x == INFINITY || left_proj.y == INFINITY){
                        is_foreign_loop = false;

                    } else{
                        is_foreign_loop = true;

                        //Itterate through segments and determine if point is on foreign loop
                        for (const LineSeg &seg : traversed_segs){
                            temp = seg.point_seg_col(left_proj);
                            min_dist = std::min(min_dist, temp);

                            if (temp < 0.1){   //Replace constant!!
                                is_foreign_loop = false;
                                break;
                            }
                        }
                    }
                    

                    if ((is_foreign_loop && min_dist != INFINITY) || return_prev_flag){

                        //Foreign-loop has been detected!! Jump across
                        //Rotate to face away from wall and enter wall-contact state (forwards till wall is reached)
                        rotate_ang(M_PI_2); 

                        state = CONTACT_WALL;
                        awaiting_loop_jump = return_prev_flag; //If returning to previous loop, continue awaiting loop jump
                        prev_at_init_pose = true;
                        
                        seg_end_point = curr_pose.pos + project_point(curr_pose.theta + M_PI/2, prev_right_dist);

                        corner_flag = true; //Although a corner move isn't occuring, the segment should be stored

                        //RCLCPP_INFO(this->get_logger(), "Jumping walls: (distance = %.2f)", min_dist);
                    }
                }

            }

            //Corner-turn command has occured, add traversed-seg to list
            if (corner_flag){
                //RCLCPP_INFO(this->get_logger(), "Segment completed: (%.2f, %.2f) to (%.2f, %.2f)", prev_seg_end_point.x,prev_seg_end_point.y,
                //                                                                                   seg_end_point.x, seg_end_point.y);

                LineSeg temp_seg(prev_seg_end_point, seg_end_point);                                                                                  
                traversed_segs.push_back(temp_seg);
                prev_seg_end_point = seg_end_point; //End of current seg is beggining of next.

                //NEED A WAY OF PUBLISHING SEGMENT - MAIN CAN QUERY MOST RECENT TRAVERSED SEG

                moving_avr_buf.clear(); //Clear angular-refinement buffer 
                old_avr = 0.0;
            }

            break;
        }

        //Mini state machine for 3-part move around right corner
        case ROT_RIGHT_CORNER: {
            switch (move_index){
            case 0:
                move_dist(CORNER_OFFSET); //Move far enough to clear wall (plus comfort threshold)
                break;

            case 1:
                rotate_ang(-90.0); //Rotate right 90deg;
                break;

            case 2:
                move_dist(CORNER_OFFSET*3); //Move forwards to regain right-contact with wall

                //Return to normal state
                state = AWAIT_LOGIC;
                break;
            }

            move_index++;
            break;
        }
    }

    prev_right_dist = distance[RIGHT];

    
    //Return command
    return actuator_cmd;
}


double wallFollower::query_roc(){
    return avr_roc;
}


ActuatorCmd wallFollower::calc_refinement(std::array<float,4> distance, std::string& debug_buf){
    ActuatorCmd actuator_cmd{actuator_cmd.MODE_VEL_ANGULAR, 0.0};

    //Only apply refinement if flagged from main actuation cmd
    if (!apply_refinement){
        debug_buf.append("NO REFINEMENT!\n");
        return actuator_cmd;}
    
    moving_avr_buf.push_back(distance[RIGHT]);

    //Refinement in rad/s
    double refinement = 0.0;

    //Only perform refinements once window is full
    if (moving_avr_buf.size() == moving_avr_window){
        //Remove old entry if desired window size has been reached
        moving_avr_buf.pop_front();

        double new_avr = std::accumulate(moving_avr_buf.begin(), moving_avr_buf.end(), 0.0) / moving_avr_buf.size();

        //Avoid refining on first pass - old_avr=0 (garbage value)
        if (old_avr){
            //Calculate rate of change (lidar dt = time between updates)
            avr_roc = (new_avr - old_avr) / LIDAR_DT;

            //Small-angle aprox of angular error
            double error = avr_roc / STABLE_VEL;

            //Publish correctional vel prop to error
            refinement = -error * 4.5;
        }

        old_avr = new_avr;
    }

    //Construct actuator goal
    actuator_cmd.magnitude = refinement;

    return actuator_cmd;
}
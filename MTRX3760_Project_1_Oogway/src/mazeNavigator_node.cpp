#include "mtrx3760_oogway_mazeSolver/mazeNavigator_node.hpp"



using namespace std::chrono_literals;



mazeNavigator::mazeNavigator()
: Node("oogway_mazeSolver_node")
{
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos); 
  angular_cmd_pub_ = this->create_publisher<mtrx3760_oogway_mazesolver::msg::AngularCmd>("angular_cmd", qos); 
  linear_cmd_pub_ = this->create_publisher<mtrx3760_oogway_mazesolver::msg::LinearCmd>("linear_cmd", qos); 

  // Initialise subscribers
  at_goal_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "at_goal", 
    qos, 
    std::bind(
      &mazeNavigator::at_goal_callback, 
      this, 
      std::placeholders::_1));

  wall_dist_sub_ = this->create_subscription<mtrx3760_oogway_mazesolver::msg::WallDist>(
    "wall_dist", 
    qos, 
    std::bind(
      &mazeNavigator::wall_dist_callback,   
      this, 
      std::placeholders::_1));

  is_abs_rotating_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "is_abs_rotating", 
    qos, 
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
        is_abs_rotating = msg->data;
    }
  );

  is_abs_moving_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "is_abs_moving", 
    qos, 
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
        is_abs_moving = msg->data;
    }
  );

  curr_pose_sub_ = this->create_subscription<mtrx3760_oogway_mazesolver::msg::Pose>(
    "curr_pose", 
    qos, 
    [this](const mtrx3760_oogway_mazesolver::msg::Pose::SharedPtr msg) {
        curr_pose.pos.x = msg->x;
        curr_pose.pos.y = msg->y;
        curr_pose.theta = msg->theta;
    }
  );

  // Initialise member variables
  is_abs_rotating = false;
  is_abs_moving = false;

  awaiting_loop_jump = false; //State flag for whether robot is searching for foreign loop to jump to (after loop completed)

  prev_at_init_pose = true;
  prev_right_dist = 0.0;

  state = CONTACT_WALL; //Initial state where robot gains contact with wall

  //Assign previous wall_pres
  prev_wall_pres.fill(false);

  //Moving average of wall-dist to refine angle (odom innaccuracy)
  moving_avr_window = 5;
  old_avr = 0.0;

  std::ofstream file("seg_data.txt", std::ios::trunc); // clear segment data
  file.close();

  RCLCPP_INFO(this->get_logger(), "Oogway Maze Navigator node has been initialised");
}

mazeNavigator::~mazeNavigator()
{
  RCLCPP_INFO(this->get_logger(), "Oogway Maze Navigator simulation node has been terminated");
}

void mazeNavigator::at_goal_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::cout << msg->data << std::endl;
  //RCLCPP_INFO(this->get_logger(), "at_goal callback fired");  // STUB
}


void mazeNavigator::wall_dist_callback(const mtrx3760_oogway_mazesolver::msg::WallDist::SharedPtr msg)
{
 
  //RCLCPP_INFO(this->get_logger(), "Pose: x:%.2f, y:%.2f, theta:%.2f", curr_pose.pos.x, curr_pose.pos.y, curr_pose.theta);  // STUB

  /*
  //Check if an update in wall presence has occured
  if (prev_wall_pres != msg->wall_pres){
    // Perform deep copy
    prev_wall_pres = msg->wall_pres; 
    
    if (!msg->wall_pres[FORWARD]){
      set_vel(1.0);
    } else{
      set_vel(0.0);
    }
    
  }*/
/*
  for (double dist : msg->distance){
    if (dist == INFINITY){
      RCLCPP_INFO(this->get_logger(), "INFINITE DISTANCE WTF IS GOING ONNNNNN!!!!");
    }
  }
*/
  


  if (is_abs_moving || is_abs_rotating){
    return;
  }

 

  switch (state) {
    case CONTACT_WALL: {
      if (msg->wall_pres[FORWARD]){
        //Wall is in contact - rotate to align with wall then enter main logic state

        prev_at_init_pose = true; //Flag to avoid instant loop-completion
        loop_init_pose = curr_pose;
        loop_init_pose.theta += 90.0; //True starting pose is aligned with wall (90deg offset)
        RCLCPP_INFO(this->get_logger(), "First contact with loop! starting pose x: %.2f y: %.2f theta: %.2f", 
                                          loop_init_pose.pos.x, loop_init_pose.pos.y, loop_init_pose.theta);

        //Project forward distance in robot's direction to get global wall_pos
        prev_seg_end_point.x = curr_pose.pos.x + cos(curr_pose.theta) * msg->distance[FORWARD];
        prev_seg_end_point.y = curr_pose.pos.y + sin(curr_pose.theta) * msg->distance[FORWARD];


        rotate_offset(90.0); //Rotate left 90deg (stops auto)

      
        state = AWAIT_LOGIC;

      } else{
        set_vel(0.2);
      }
      break;
    }

    case AWAIT_LOGIC: {
      bool corner_flag = true; //Flag for whether a corner-turn command has occured
      Point seg_end_point;

      if (!msg->wall_pres[RIGHT]){
        RCLCPP_INFO(this->get_logger(), "Right corner!!");
        set_vel(0.0);

        //Turn right-corner
        //Enter movement sequence
        state = ROT_RIGHT_CORNER;
        move_index = 0;

        //Calculate global pos of traveresed-wall's end (current right-dist is invalid on right turn so prev-update must be used)
       seg_end_point = Point{curr_pose.pos.x + sin(curr_pose.theta) * prev_right_dist, 
                            curr_pose.pos.y - cos(curr_pose.theta) * prev_right_dist};

      } else if (msg->wall_pres[FORWARD]){
        RCLCPP_INFO(this->get_logger(), "Left corner!!");
        RCLCPP_INFO(this->get_logger(), "wall_dist callback fired: forward: %.2f, right: %.2f, backward: %.2f, left: %.2f ", msg->distance[FORWARD], msg->distance[RIGHT], msg->distance[BACKWARD], msg->distance[LEFT]);  // STUB
        RCLCPP_INFO(this->get_logger(), "pres: forward: %d, right: %d, backward: %d, left: %d ", msg->wall_pres[FORWARD], msg->wall_pres[RIGHT], msg->wall_pres[BACKWARD], msg->wall_pres[LEFT]);  // STUB

        rotate_offset(90.0); //Rotate left 90deg (stops auto)

        //Calculate global pos of traversed-wall's end
        double sin_a = sin(curr_pose.theta);
        double cos_a = cos(curr_pose.theta);

        seg_end_point = Point{curr_pose.pos.x + sin_a * msg->distance[RIGHT] + cos_a * msg->distance[FORWARD], 
                            curr_pose.pos.y - cos_a * msg->distance[RIGHT] + sin_a * msg->distance[FORWARD]};


      } else{
        //Moving forwards:
        RCLCPP_INFO(this->get_logger(), "Moving forwards!!");
        set_vel(0.2); 

        //Check if loop has been completed
        bool return_prev_flag = false;
        bool at_init_pose = dist2D(curr_pose.pos, loop_init_pose.pos) < POSE_EQUAL_THRESH;
        if (at_init_pose && !prev_at_init_pose){
          //Loop has been completed - wait for foreign loop to jump to (disregard first flag)
          RCLCPP_INFO(this->get_logger(), "Loop completed!!");
          
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

        //Refine angle to account for inaccuracies in odom
        refine_angle(msg->distance[RIGHT]);
        
        //Search left LIDAR probe for foreign loop
        //If projected point can't be found in existing segment array, loop is foreign
        if (awaiting_loop_jump){
          //Calculate global point of left facing lidar-probe
          Point left_proj = {curr_pose.pos.x - sin(curr_pose.theta) * msg->distance[LEFT], 
                            curr_pose.pos.y + cos(curr_pose.theta) * msg->distance[LEFT]};

          
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
            set_vel(0.0);
            //Rotate to face away from wall and enter wall-contact state (forwards till wall is reached)
            rotate_offset(90.0); 
            state = CONTACT_WALL;
            awaiting_loop_jump = return_prev_flag; //If returning to previous loop, continue awaiting loop jump
            prev_at_init_pose = true;

            
            double sin_a = sin(curr_pose.theta);
            double cos_a = cos(curr_pose.theta);
            seg_end_point = Point{curr_pose.pos.x + sin_a * msg->distance[RIGHT], 
                            curr_pose.pos.y - cos_a * msg->distance[RIGHT]};

            corner_flag = true; //Although a corner move isn't occuring, the segment should be stored

             RCLCPP_INFO(this->get_logger(), "Jumping walls: (distance = %.2f)", min_dist);
          }
        }

      }

      //Corner-turn command has occured, add traversed-seg to list
      if (corner_flag){
        RCLCPP_INFO(this->get_logger(), "Segment completed: (%.2f, %.2f) to (%.2f, %.2f)", prev_seg_end_point.x,prev_seg_end_point.y,
                                                                                          seg_end_point.x, seg_end_point.y);

        LineSeg temp_seg(prev_seg_end_point, seg_end_point);                                                                                  
        traversed_segs.push_back(temp_seg);
        prev_seg_end_point = seg_end_point; //End of current seg is beggining of next.


        std::ofstream file("seg_data.txt", std::ios::app); // open in append mode

        Point temp = {curr_pose.pos.x - sin(curr_pose.theta) * msg->distance[LEFT], 
                            curr_pose.pos.y + cos(curr_pose.theta) * msg->distance[LEFT]};
        file << temp_seg.to_string() << ("RobotPos: (" + std::to_string(curr_pose.pos.x) + ", " + std::to_string(curr_pose.pos.y) + ")")
                                        << ("ProjectedPos: (" + std::to_string(temp.x) + ", " + std::to_string(temp.y) + ")\n");
        file.close();


        moving_avr_buf.clear(); //Clear angular-refinement buffer 
        old_avr = 0.0;
      }

      break;
    }

    //Mini state machine for 3-part move around right corner
    case ROT_RIGHT_CORNER: {
      switch (move_index){
        case 0:
          move_distance(CORNER_OFFSET); //Move far enough to clear wall (plus comfort threshold)
          break;

        case 1:
          rotate_offset(-90.0); //Rotate right 90deg;
          break;

        case 2:
          move_distance(CORNER_OFFSET*3); //Move forwards to regain right-contact with wall

          //Return to normal state
          state = AWAIT_LOGIC;
          break;
      }

      move_index++;
      break;
    }
  }

  prev_right_dist = msg->distance[RIGHT];

}


void mazeNavigator::rotate_offset(double angle_offset){
  mtrx3760_oogway_mazesolver::msg::AngularCmd cmd;
  cmd.target_angle = std::fmod((curr_angle + angle_offset * (M_PI/180)), M_PI*2) ; //Wrap angle to valid range
  cmd.mode = cmd.MODE_ABSOLUTE;

  angular_cmd_pub_->publish(cmd); //Publish angle to controller

  curr_angle = cmd.target_angle; // Update current angle
}

void mazeNavigator::set_ang_vel(double ang_vel){
  mtrx3760_oogway_mazesolver::msg::AngularCmd cmd;
  cmd.angular_vel = ang_vel;
  cmd.mode = cmd.MODE_VELOCITY;

  angular_cmd_pub_->publish(cmd); //Publish angular velocity to controller
}

void mazeNavigator::move_distance(double distance){
  mtrx3760_oogway_mazesolver::msg::LinearCmd cmd;
  cmd.target_distance = std::max(0.0, distance); //Clamp distance to positive value
  cmd.mode = cmd.MODE_ABSOLUTE;

  linear_cmd_pub_->publish(cmd); //Publish distance to controller
}

void mazeNavigator::set_vel(double vel){
  mtrx3760_oogway_mazesolver::msg::LinearCmd cmd;
  cmd.linear_vel = std::max(0.0, vel); //Clamp velocity to positive value
  cmd.mode = cmd.MODE_VELOCITY;

  linear_cmd_pub_->publish(cmd); //Publish velocity to controller  
}




//Refine angle based on right-distance roc
void mazeNavigator::refine_angle(double new_dist){
  moving_avr_buf.push_back(new_dist);

  //Only perform refinements once window is full
  if (moving_avr_buf.size() == moving_avr_window){
    //Remove old entry if desired window size has been reached
    moving_avr_buf.pop_front();

    double new_avr = std::accumulate(moving_avr_buf.begin(), moving_avr_buf.end(), 0.0) / moving_avr_buf.size();

    //Avoid refining on first pass - old_avr=0 (garbage value)
    if (old_avr){
      //Calculate rate of change (lidar dt = time between updates)
      double avr_roc = (new_avr - old_avr) / LIDAR_DT;

      //Small-angle aprox of angular error
      double error = avr_roc / STABLE_VEL;

      //Publish correctional vel prop to error
      set_ang_vel(-error * 2.5);
    }

    old_avr = new_avr;
  }
}


/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mazeNavigator>());
  rclcpp::shutdown();

  return 0;
}
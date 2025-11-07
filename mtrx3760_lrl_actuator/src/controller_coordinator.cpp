#include "mtrx3760_lrl_actuator/controller_coordinator.hpp"
//------------------------------------------------------------------------------------------
mtrx3760_lrl_warehousebot::controllerCoordinator::controllerCoordinator()
{
    passiveAngularVelocity = 0.0;
    passiveLinearVelocity = 0.0;
    currentControlMethod = PASSIVE;

    return;
}
//---------------------------------------------
mtrx3760_lrl_warehousebot::controllerCoordinator::~controllerCoordinator()
{
    if (currentControlMethod != PASSIVE)
    {
        delete currentController;
    }
}
//---------------------------------------------
bool mtrx3760_lrl_warehousebot::controllerCoordinator::controllerRunning()
{
    bool running;

    if (currentControlMethod == PASSIVE)
    {
        running = false;
    }
    else
    {
        running = true;
    }

    return running;
}
//---------------------------------------------
geometry_msgs::msg::TwistStamped mtrx3760_lrl_warehousebot::controllerCoordinator::updatePassiveVelLinear(const std::shared_ptr<GoalHandleActuator> goal_handle)
{
    geometry_msgs::msg::TwistStamped outputCmd;
    passiveLinearVelocity = goal_handle->get_goal()->magnitude;

    outputCmd.twist.linear.x = passiveLinearVelocity;
    outputCmd.twist.angular.z = passiveAngularVelocity;

    auto result = std::make_shared<Actuator::Result>();
    result->success = true;
    goal_handle->succeed(result);

    return outputCmd;
}
//---------------------------------------------
geometry_msgs::msg::TwistStamped mtrx3760_lrl_warehousebot::controllerCoordinator::updatePassiveVelAngular(const std::shared_ptr<GoalHandleActuator> goal_handle)
{
    geometry_msgs::msg::TwistStamped outputCmd;

    passiveAngularVelocity = goal_handle->get_goal()->magnitude;

    outputCmd.twist.linear.x = passiveLinearVelocity;
    outputCmd.twist.angular.z = passiveAngularVelocity;

    auto result = std::make_shared<Actuator::Result>();
    result->success = true;
    goal_handle->succeed(result);

    return outputCmd;
}
//---------------------------------------------
void mtrx3760_lrl_warehousebot::controllerCoordinator::fireUpController(const std::shared_ptr<GoalHandleActuator> goal_handle)
{
    auto goalData = goal_handle->get_goal();

    std::cout << "[Controller Coordinator] : " << "RECIEVED" << goalData->mode << std::endl;

    if (currentControlMethod == PASSIVE)
    {
        switch (goalData->mode)
        {
            case ABS_ANGULAR:
            {
                std::cout << "[Controller Coordinator] : " << "Firing up ABS_ANGULAR" << std::endl;
                currentController = new mtrx3760_lrl_warehousebot::absAngularController(goal_handle);
                currentControlMethod = ABS_ANGULAR;

                // makes sure it stays still after, just in case the robot is moving when it starts the routine
                passiveAngularVelocity = 0.0;
                passiveLinearVelocity = 0.0;

                break;
            }

            case ABS_LINEAR:
            {
                std::cout << "[Controller Coordinator] : " << "Firing up ABS_LINEAR" << std::endl;
                currentController = new mtrx3760_lrl_warehousebot::absLinearController(goal_handle);
                currentControlMethod = ABS_LINEAR;

                // makes sure it stays still after, just in case the robot is moving when it starts the routine
                passiveAngularVelocity = 0.0;
                passiveLinearVelocity = 0.0;

                break;
            }

            default:
            {
                std::cout << "[Controller Coordinator]: " << "Invalid contoller specifed -> " << goalData->mode << std::endl;
                break;
            }
        }
    }
    else
    {
        // send fail??
        std::cout << "[Controller Coordinator]: " << "Already running -> " << currentControlMethod << std::endl;
    }

    return;
}
//---------------------------------------------
geometry_msgs::msg::TwistStamped mtrx3760_lrl_warehousebot::controllerCoordinator::routeTf(geometry_msgs::msg::TransformStamped msg)
{
    geometry_msgs::msg::TwistStamped outputCmd;

    std::cout << "[Controller Coordinator] : " << "Passive Linear ->" << passiveLinearVelocity << " Passive Angular -> " << passiveAngularVelocity << std::endl;


    switch (currentControlMethod)
    {
        case ABS_ANGULAR:
        {
            std::cout << "[Controller Coordinator] : " << "Feeding TF to Angular..." << std::endl;
            outputCmd = currentController->respondToStimulus(msg);

            // cleanup if complete
            if (outputCmd.twist.linear.x == 0.0 and outputCmd.twist.angular.z == 0.0)
            {
                currentControlMethod = PASSIVE;
                delete currentController;
            }
            break;
        }

        case ABS_LINEAR:
        {
            std::cout << "[Controller Coordinator] : " << "Feeding TF to Linear..." << std::endl;
            outputCmd = currentController->respondToStimulus(msg);

            // cleanup if complete
            if (outputCmd.twist.linear.x == 0.0 and outputCmd.twist.angular.z == 0.0)
            {
                currentControlMethod = PASSIVE;
                delete currentController;
            }
            break;
        }

        case PASSIVE:
        {
            std::cout << "[Controller Coordinator] : " << "Pushing Passive Values" << std::endl;

            outputCmd.twist.linear.x = passiveLinearVelocity;
            outputCmd.twist.angular.z = passiveAngularVelocity;
            break;
        }
    }

    return outputCmd;
}
//------------------------------------------------------------------------------------------
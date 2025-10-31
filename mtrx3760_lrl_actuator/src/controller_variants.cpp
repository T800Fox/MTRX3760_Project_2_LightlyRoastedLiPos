#include "mtrx3760_lrl_actuator/controller_variants.hpp"
//------------------------------------------------------------------------------------------
mtrx3760_lrl_warehousebot::controller::controller(const std::shared_ptr<GoalHandleActuator> goal_handle)
{
    std::cout << "Controller Constructed" << std::endl;
    controllerGoalHandle = goal_handle;
    storedControlState = AWAITING_START_COND;
    targetValue = -0.1;

    return;
}
//---------------------------------------------
mtrx3760_lrl_warehousebot::controller::~controller()
{
    return;
}
//---------------------------------------------
double mtrx3760_lrl_warehousebot::controller::transformToHeading(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    // Convert quaternion to roll, pitch, yaw
    tf2::Quaternion tf_quat(
        aTransformMsg.transform.rotation.x, 
        aTransformMsg.transform.rotation.y,
        aTransformMsg.transform.rotation.z,
        aTransformMsg.transform.rotation.w
    );

    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}
//------------------------------------------------------------------------------------------
mtrx3760_lrl_warehousebot::absAngularController::absAngularController(const std::shared_ptr<GoalHandleActuator> goal_handle): controller(goal_handle)
{
    std::cout << "Angular Controller Running..." << std::endl;
    return;
}
//---------------------------------------------
geometry_msgs::msg::TwistStamped mtrx3760_lrl_warehousebot::absAngularController::respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    std::cout << "Angular State Machine" << std::endl;
    geometry_msgs::msg::TwistStamped outputCmd;

    // check for abort command?

    // check for completion
    if (storedControlState == RUNNING && abs(generateErrorTerm(aTransformMsg, false)) <= completionTol)
    {
        storedControlState = TARGET_REACHED;
    }

    // run state machine
    switch (storedControlState)
    {
        case AWAITING_START_COND:
        {
            std::cout << "ANGULAR -> AWAIT" << std::endl;

            targetValue = generateTargetValue(aTransformMsg);

            double err = generateErrorTerm(aTransformMsg, false);
            lastError = err;
            storedControlState = RUNNING;
        }
        
        case RUNNING:
        {
            std::cout << "ANGULAR -> RUN" << std::endl;
            double currentHeading = transformToHeading(aTransformMsg);
            double err = generateErrorTerm(aTransformMsg, true);
            
            double response = computeResponse(err);

            std::cout << "      LAST ERROR : " << lastError << std::endl;
            std::cout << "      CURRENT : " << currentHeading << std::endl;
            std::cout << "      TARGET : " << targetValue << std::endl; 
            std::cout << "      ERROR : " << err << std::endl;
            std::cout << "      RESPONSE : " << response << std::endl;
            
            // send feedback
            auto feedback = std::make_shared<Actuator::Feedback>();
            auto err_n = feedback->err_n;
            auto err_t = feedback->err_t;
            err_t = err;
            err_n = 0.0;
            controllerGoalHandle->publish_feedback(feedback);

            // set new actuator values
            outputCmd.twist.angular.z = response;

            break;
        }

        case TARGET_REACHED:
        {
            outputCmd.twist.angular.z = 0;

            auto result = std::make_shared<Actuator::Result>();
            result->success = true;
            controllerGoalHandle->succeed(result);
            
            storedControlState = COMPLETE;
            break;
        }

    }

    return outputCmd;
}
//---------------------------------------------
double mtrx3760_lrl_warehousebot::absAngularController::generateTargetValue(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    double offsetToApply =  controllerGoalHandle->get_goal()->magnitude;
    double currentHeading = transformToHeading(aTransformMsg);
    double generatedTarget = currentHeading + offsetToApply;

    std::cout << "      Current Heading -> " << currentHeading << std::endl;
    std::cout << "      Offset To Apply -> " << offsetToApply << std::endl;
    std::cout << "      target -> " << generatedTarget << std::endl;

    // if(generatedTarget < -1.0*pi)
    // {
    //     std::cout << "BELOW BOUND : " << generatedTarget << " +2pi" << std::endl;
    //     generatedTarget = generatedTarget + 2.0*pi;
    // }
    // else if(generatedTarget > pi)
    // {
    //     std::cout << "ABOVE BOUND : " << generatedTarget << " -2pi" << std::endl;
    //     generatedTarget = generatedTarget - 2.0*pi;
    // }
    
    // std::cout << "bounded target -> " << generatedTarget << std::endl;

    return generatedTarget;
}
//---------------------------------------------
double mtrx3760_lrl_warehousebot::absAngularController::generateErrorTerm(geometry_msgs::msg::TransformStamped aTransformMsg, bool doJump)
{

    // double angle_diff = heading2_rad - heading1_rad;

    // // Normalize the angle to be within [-pi, pi]
    // // This handles cases where the difference is > pi or < -pi
    // if (angle_diff > M_PI) {
    //     angle_diff -= 2 * M_PI;
    // } else if (angle_diff < -M_PI) {
    //     angle_diff += 2 * M_PI;
    // }

    double currentHeading = transformToHeading(aTransformMsg);
    double errorTerm = currentHeading - targetValue;

    // normalise
    if(errorTerm > pi)
    {
        errorTerm -=  2.0*pi;
    }
    else if(errorTerm < -pi)
    {
        errorTerm -= 2.0* pi;
    }
    
    // detect jump when moving over -pi and pi boundary
    if (abs(lastError - errorTerm) > pi/2.0 && doJump)
    {
        std::cout << "              JUMP!!!!!!" << std::endl;
        if (targetValue > pi)
        {
            targetValue -= 2.0*pi;
        }
        else
        {
            targetValue += 2.0*pi;
        }
    }



    return errorTerm;
}
//---------------------------------------------
double mtrx3760_lrl_warehousebot::absAngularController::computeResponse(double aErrorVal)
{
    double response = p * aErrorVal; 

    if(response > MAX_ANG_VEL)
    {
        response = MAX_ANG_VEL;
    }

    return response;
}
//------------------------------------------------------------------------------------------
mtrx3760_lrl_warehousebot::absLinearController::absLinearController(const std::shared_ptr<GoalHandleActuator> goal_handle)
: controller(goal_handle)
{
    std::cout << "Absolute Linear Controller Running..." << std::endl;

    return;
}
//---------------------------------------------
geometry_msgs::msg::TwistStamped mtrx3760_lrl_warehousebot::absLinearController::respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    std::cout << "Linear State Machine" << std::endl;
    geometry_msgs::msg::TwistStamped outputCmd;

    // check for abort?

    // check for compeletion
    if (storedControlState == RUNNING && abs(generateErrorTerm(aTransformMsg)) < completionTol)
    {
        storedControlState = TARGET_REACHED;
    }

    // state machine
    switch(storedControlState)
    {
        case AWAITING_START_COND:
        {
            std::cout << "LINEAR -> AWAIT" << std::endl;
            targetCoord = generateTargetCoord(aTransformMsg);
            lastErr = generateErrorTerm(aTransformMsg);
            storedControlState = RUNNING;
        }

        case RUNNING:
        {
            std::cout << "LINEAR -> RUN" << std::endl;
            double err = generateErrorTerm(aTransformMsg);
            double response = computeResponse(err);
            lastErr = err;

            std::cout << "      " << "ERROR : " << err << std::endl;
            std::cout << "      " << "RESPONSE : " << response << std::endl;

            // send feedback
            auto feedback = std::make_shared<Actuator::Feedback>();
            auto err_n = feedback->err_n;
            auto err_t = feedback->err_t;
            err_t = 0.0;
            err_n = err;
            controllerGoalHandle->publish_feedback(feedback);

            // set new actuator values
            outputCmd.twist.linear.x = response;

            break;
        }

        case TARGET_REACHED:
        {
            std::cout << "LINEAR -> TARGET REACHED" << std::endl;
            outputCmd.twist.linear.x = 0;

            auto result = std::make_shared<Actuator::Result>();
            result->success = true;
            controllerGoalHandle->succeed(result);
            
            storedControlState = COMPLETE;
            break;
        }
    }

    return outputCmd;
}
//---------------------------------------------
coord mtrx3760_lrl_warehousebot::absLinearController::transformToCoord(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    coord outputCoord;

    // coords were going in opposite direction that robot was driving; beats me why atm
    outputCoord.x = -1.0 * aTransformMsg.transform.translation.x;
    outputCoord.y = -1.0 * aTransformMsg.transform.translation.y;

    return outputCoord;
}
//---------------------------------------------
coord mtrx3760_lrl_warehousebot::absLinearController::generateTargetCoord(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    coord outputCoord;

    double distanceToTraverse = controllerGoalHandle->get_goal()->magnitude;
    double currentHeading = transformToHeading(aTransformMsg);
    coord currentCoord = transformToCoord(aTransformMsg);

    double dX =  std::cos(currentHeading) * distanceToTraverse;
    double dY =  std::sin(currentHeading) * distanceToTraverse;

    outputCoord.x = currentCoord.x + distanceToTraverse;
    outputCoord.y = currentCoord.y;

    // outputCoord.x = currentCoord.x + dX;
    // outputCoord.y = currentCoord.y + dY;

    std::cout << "      Traverse Distance : " << distanceToTraverse << std::endl;
    std::cout << "      Current Coord : (" << currentCoord.x << " , " << currentCoord.y << ")" << std::endl;
    std::cout << "      Current Heading : " << currentHeading << std::endl;
    std::cout << "      deltaX : " << dX << std::endl; 
    std::cout << "      deltaY : " << dY << std::endl; 
    std::cout << "      Goal @ (" << outputCoord.x << " , " << outputCoord.y << ")" << std::endl;

    
    return outputCoord;
}
//---------------------------------------------
double mtrx3760_lrl_warehousebot::absLinearController::generateErrorTerm(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    double errorTerm;
    double errorMagnitude;
    double errorSign = 1.0;
    coord currentCoord = transformToCoord(aTransformMsg);

    double currentHeading = transformToHeading(aTransformMsg);
    
    std::cout << "      Current Heading : " << currentHeading << std::endl;
    std::cout << "      Computing Error f/ current coord (" << currentCoord.x << " , " << currentCoord.y << ")" << std::endl;
    std::cout << "      With Goal @ coord (" << targetCoord.x << " , " << targetCoord.y << ")" << std::endl;

    errorMagnitude = std::sqrt( pow((currentCoord.x - targetCoord.x) , 2.0 ) + pow((currentCoord.y - targetCoord.y) , 2.0 ));

    // assuming target never exactly equals current
    // if (targetCoord.x > currentCoord.x || targetCoord.y > targetCoord.y)
    // {
    //     errorSign = 1.0;
    // }
    // else
    // {
    //     errorSign = -1.0;
    // }

    errorTerm = errorMagnitude * errorSign;
    return errorTerm;
}
//---------------------------------------------
double mtrx3760_lrl_warehousebot::absLinearController::computeResponse(double aErrorVal)
{
    double errD = (aErrorVal - lastErr) / 0.1;
    double response;
    double proportionalTerm = p * aErrorVal;
    double derivativeTerm = d * errD;

    response = proportionalTerm - derivativeTerm;

    std::cout << "  CONTROLLER" << std::endl;
    std::cout << "          Error : " << aErrorVal << std::endl;
    std::cout << "          Error Delta : " << errD << std::endl;
    std::cout << "          Proportional Value : " << proportionalTerm << std::endl;
    std::cout << "          Derivative Value :" << derivativeTerm << std::endl;
    

    response = std::clamp(response, -1.0*MAX_LIN_VEL, MAX_LIN_VEL);
    std::cout << "          Clamped Response :" << response << std::endl << std::endl;
    return response;
}
//------------------------------------------------------------------------------------------
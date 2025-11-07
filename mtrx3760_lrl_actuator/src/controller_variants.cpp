#include "mtrx3760_lrl_actuator/controller_variants.hpp"
//------------------------------------------------------------------------------------------
mtrx3760_lrl_warehousebot::controller::controller(const std::shared_ptr<GoalHandleActuator> goal_handle)
{
    std::cout << "Controller Constructed" << std::endl;
    controllerGoalHandle = goal_handle;
    storedControlState = AWAITING_START_COND;

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
    // target value not set here as this method of control runs off where should be versus where it is
    std::cout << "Angular Controller Running..." << std::endl;
    targetValue = -0.1;
    return;
}
//---------------------------------------------
geometry_msgs::msg::TwistStamped mtrx3760_lrl_warehousebot::absAngularController::respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    std::cout << "Angular State Machine" << std::endl;
    geometry_msgs::msg::TwistStamped outputCmd;

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
            
            outputCmd.twist.angular.z = response;   // set new actuator values

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

    return generatedTarget;
}
//---------------------------------------------
double mtrx3760_lrl_warehousebot::absAngularController::generateErrorTerm(geometry_msgs::msg::TransformStamped aTransformMsg, bool doJump)
{
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
        std::cout << "              Transitioning over -pi/pi boundary; applying correction to target heading..." << std::endl;
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

    // this controller stores goal distance as it doesn't really care where wants to be, 
    // just how far it should be from the starting position.
    // initially it was setup in the same method as angular, but imperfections in the gearbox
    // meant it would occasionally drifting off course and miss the target point
    goalDistance = controllerGoalHandle->get_goal()->magnitude;

    return;
}
//---------------------------------------------
geometry_msgs::msg::TwistStamped mtrx3760_lrl_warehousebot::absLinearController::respondToStimulus(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    std::cout << "Linear State Machine" << std::endl;
    geometry_msgs::msg::TwistStamped outputCmd;

    // check for abort?

    // checking for completion done in compute response, 
    // when it gets soo small the motors will do nothing output velocity is set to 0  

    // state machine
    switch(storedControlState)
    {
        case AWAITING_START_COND:
        {
            std::cout << "LINEAR -> AWAIT" << std::endl;

            startCoord = transformToCoord(aTransformMsg);

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

            if (storedControlState != TARGET_REACHED)
            {
                outputCmd.twist.linear.x = response;

                break;
            }
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
mtrx3760_lrl_warehousebot::absLinearController::coord mtrx3760_lrl_warehousebot::absLinearController::transformToCoord(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    coord outputCoord;

    // coords were going in opposite direction that robot was driving; beats me why atm
    outputCoord.x = -1.0 * aTransformMsg.transform.translation.x;
    outputCoord.y = -1.0 * aTransformMsg.transform.translation.y;

    return outputCoord;
}
//---------------------------------------------
double mtrx3760_lrl_warehousebot::absLinearController::generateErrorTerm(geometry_msgs::msg::TransformStamped aTransformMsg)
{
    double errorTerm;
    double distCovered;
    double errorMagnitude;
    double errorSign = 1.0;

    coord currentCoord = transformToCoord(aTransformMsg);
    distCovered = std::sqrt( pow((currentCoord.x - startCoord.x) , 2.0 ) + pow((currentCoord.y - startCoord.y) , 2.0 ));
    errorTerm = goalDistance - distCovered;

    std::cout << "      goal distance : " << goalDistance << std::endl;
    std::cout << "      distance covered : " << distCovered << std::endl;

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

    if (abs(response) <= completionTol)
    {
        storedControlState = TARGET_REACHED;
        std::cout << "  CONTROLLER >>>>> BOT HIT TARGET" << std::endl;
    }
    std::cout << "          Clamped Response :" << response << std::endl << std::endl;

    return response;
}
//------------------------------------------------------------------------------------------

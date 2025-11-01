#ifndef MTRX3760_LRL_MAZESOLVER_PATHFOLLOWER_NODE_HPP_
#define MTRX3760_LRL_MAZESOLVER_PATHFOLLOWER_NODE_HPP_

#include <memory>
#include <thread>
#include <vector>
 
#include "rclcpp/rclcpp.hpp"

namespace mtrx3760_package_delivery
{
    struct actionCommand{
        int mode;
        double magnitude;
    };

    class pathFeeder
    {
        public:
            pathFeeder(/* args */);
            ~pathFeeder();

            void pushToCommandList(actionCommand aCommand);


        private:
            actionCommand fetchNextCommand();

            std:vector<actionCommand> commandList;
    };
}; // namespace mtrx3760_package_delivery
    


#endif 

        
//Generic wrapper for action that impliments blocking functionaility through polling (work around)
template<typename ActionT>
class ActionClientWrapper {

    enum ACTION_STATUS {
        UNDEFINED = 0,
        AWAITING_RESPONSE = 1,
        AWAITING_RESULT = 2
    };

    public:
        using GoalHandleAction = rclcpp_action::ClientGoalHandle<ActionT>;

        ActionClientWrapper(rclcpp::Node::SharedPtr node, const std::string &action_name);

        bool wait_for_server(std::chrono::seconds timeout = std::chrono::seconds(3));

        void send_goal(const Goal &goal,
                    std::function<void(const std::shared_ptr<const ActionT::Feedback> feedback)> feedback_cb = nullptr,
                    std::function<void(const GoalHandleAction::WrappedResult& result)> result_cb = nullptr);


        bool is_awaiting_result();


    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp_action::Client<ActionT>::SharedPtr action_client_;

        //Status for blocking functionality
        ACTION_STATUS currentActionStatus;
};
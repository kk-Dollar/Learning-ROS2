#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include "behaviortree_cpp/blackboard.h"
#include "std_msgs/msg/int32.hpp"
#include <random>

using namespace std::chrono_literals;

class PublishResult : public BT::StatefulActionNode
{
public:
    PublishResult(const std::string &action_name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(action_name, config)
    {
        node_=config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    }

    BT::NodeStatus onStart() override
    {
        std::string topic_name;
        getInput<std::string>("topic_name", topic_name);
        pub_=node_->create_publisher<std_msgs::msg::Int32>(topic_name,1);
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override
    {
        int value;
        getInput<int>("generated_number",value);
        std_msgs::msg::Int32 v;
        v.data=value;
        pub_->publish(v);
        std::cout<<"Generated number published: "<<value<<std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {
        return;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("topic_name"), BT::InputPort<int>("generated_number")};
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};
class NumberChecker : public BT::StatefulActionNode
{
public:
    NumberChecker(const std::string &action_name, const BT::NodeConfig &config)
        : BT::StatefulActionNode(action_name, config)
    {
    }

    BT::NodeStatus onStart() override
    {
        if (!getInput<int>("check_value", num_thresold_))
        {
            throw BT::RuntimeError("missing required input [goal]");
        }
        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning() override
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(1, 100);
        int random_number = distrib(gen);

        std::cout << "Generated number:" << random_number << "- Thresold: " << num_thresold_ << " - ";
        if (random_number < num_thresold_)
        {
            setOutput("generated_number", random_number);
            std::cout << "success!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout << "Failure!" << std::endl;
            setOutput("generated_nuber", -1);
            return BT::NodeStatus::FAILURE;
        }
    }

    void onHalted() override
    {
        return;
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<int>("check_value"), BT::OutputPort<int>("generated_number")};
    }

private:
    int num_thresold_;
};

class BTExecutor : public rclcpp::Node
{
public:
    BTExecutor() : Node("bt_executor")
    {
        // init_btree(); can't call in constructor qki ros node ka object cahiye waha toh constructor run hone denge phir
        // call karenge tick_function se , lkn tick funtion to cb hai isliye ek flag variable use karenge --> first
        first_ = true;
        timer_ = this->create_wall_timer(0.5s, std::bind(&BTExecutor::tick_function, this));
        blackboard_ = BT::Blackboard::create();
    }

private:
    void init_btree()
    {
        // shared_from_this() Converts this into a std::shared_ptr
        blackboard_->set<rclcpp::Node::SharedPtr>(
            "node",
            this->shared_from_this()); // it stores the current ROS2 node inside the Behavior Tree blackboard so other BT
                                       // nodes can use it. taki publish or subscribe kar sake jo ros node he kar sakta bt
                                       // node  nahi:Now every BT node can: publish subscribe log access parameters
        factory_.registerNodeType<NumberChecker>("CheckNumber1");
        factory_.registerNodeType<NumberChecker>("CheckNumber2");
        factory_.registerNodeType<NumberChecker>("CheckNumber3");
        factory_.registerNodeType<PublishResult>("PublishResult");

        this->declare_parameter<std::string>("tree_xml_file", "");
        std::string tree_file;
        this->get_parameter("tree_xml_file", tree_file);
        tree_ = factory_.createTreeFromFile(tree_file, blackboard_);
    }
    void tick_function()
    {
        if (first_)
        {
            init_btree();
            first_ = false;
        }
        tree_.tickOnce();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    BT::Tree tree_;
    BT::BehaviorTreeFactory factory_;
    BT::Blackboard::Ptr blackboard_;
    bool first_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BTExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
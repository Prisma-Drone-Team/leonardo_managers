#include "seed.h"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace seed;

class ExploreBehaviour : public Behavior{
public:
    ExploreBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();
    
    void motorSchema();

    void start();

    void exit();

protected:

    void update_weights();

    int sample_element();

    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    // random number generator
    std::mt19937 rnd;

    std::vector<std::string> explorables;
    std::vector<double> weights;

    std::vector<double> w_time;

    int current_target;

    std::string agent;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pb;
};




class PathBehaviour : public Behavior{
public:
    PathBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    std::vector<std::string> path;
    int current_target;

    std::string agent;
    WM_node *me;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pb;
};



// ********** CONCRETE PDT BEHAVIORS **********//


class TakePictureBehaviour : public Behavior { //WMVBehavior{
public:
    TakePictureBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    bool done;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pb_logger;
    //rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pb_all;
};



class WatchToBehaviour : public Behavior { //WMVBehavior{
public:
    WatchToBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    std::string target_frame;
    std::string command;
    std::string topic;
    std::string contended_variable;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};


class FlyToBehaviour : public Behavior { //WMVBehavior{
public:
    FlyToBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    std::string target_frame;
    std::string command;
    std::string topic;
    std::string contended_variable;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    //rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};


class GoToBehaviour : public Behavior { //WMVBehavior{
public:
    GoToBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    std::string target_frame;
    std::string command;
    std::string topic;
    std::string contended_variable;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    //rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};



class FollowBehaviour : public Behavior { //WMVBehavior{
public:
    FollowBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    std::string target_frame;
    std::string command;
    std::string topic;
    std::string contended_variable;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    //rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};


class TakeOffBehaviour : public Behavior { //WMVBehavior{
public:
    TakeOffBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    std::string command;
    std::string topic;
    std::string contended_variable;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};


class LandBehaviour : public Behavior { //WMVBehavior{
public:
    LandBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    std::string command;
    std::string topic;
    std::string contended_variable;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};


class WaitBehaviour : public Behavior { //WMVBehavior{
public:
    WaitBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    std::string command;
    std::string topic;
    std::string contended_variable;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;
};
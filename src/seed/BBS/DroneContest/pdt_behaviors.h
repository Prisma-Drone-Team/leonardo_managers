#include "seed.h"

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace seed;

class PhotoBehaviour : public Behavior { //WMVBehavior{
public:
    PhotoBehaviour(std::string instance);

    static Behavior *create(std::string instance);

    bool perceptualSchema();

    void motorSchema();

    void start();

    void exit();

    void img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);



    bool wait_for_click();

    void initialize_window(std::string window_name);

    cv::Mat create_selection_interface(cv::Mat image);

    static void mouse_callback(int event, int x, int y, int flags, void* userdata);

    void handle_selection_click(int x, int y);

    void save_image(std::string mode, std::string filename);

    std::string format_timestamp_with_date(const builtin_interfaces::msg::Time& timestamp);

    void add_timestamp_to_image(cv::Mat& image, const std::string& timestamp);

    std::string get_current_time_string();

protected:
    //NOTE: this variable is used to self-register the class into the BBS
    //inline static bool registered = BehaviorBasedSystem::add("template",&TemplateBehavior::create); //this should be done in the .cpp
    static std::string behavior_name;
    static bool registered;

    bool was_active;
    bool now_active;
    bool was_accomplished;
    bool now_accomplished;

    bool window_open;

    std::string dir_path;
    
    std::string mode;

    bool have_image;

    cv::Mat current_image;
    cv::Mat old_image;

    std::string current_timestamp;
    
    int video_frame;

    cv::Rect yes_rect;
    cv::Rect no_rect;

    bool clicked;
    bool image_accepted;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer;

    //rclcpp::Node::SharedPtr nh;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pbs;

    // image from agent
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sb_image;
};



class CoverBehaviour : public Behavior { //WMVBehavior{
public:
    CoverBehaviour(std::string instance);

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



class CircleBehaviour : public Behavior { //WMVBehavior{
public:
    CircleBehaviour(std::string instance);

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
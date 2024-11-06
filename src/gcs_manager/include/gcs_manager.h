#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// boost-based wget
//	from: https://cpp-netlib.org/0.12.0/examples/http/simple_wget.html
//#include <boost/network/protocol/http/client.hpp>
//#include <boost/network/uri.hpp>
#include <fstream>

//JSON
#include <jsoncpp/json/json.h>


using namespace std::chrono_literals;

class GCSManager : public rclcpp::Node
{
public:
    //using NavigateToPose = nav2_msgs::action::NavigateToPose;
    //using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GCSManager();

private:

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    
    std::string raw_json_mission;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rover_feedback_pb_;
    
    
    
    
    geometry_msgs::msg::TransformStamped get_tf(std::string source_frame, std::string target_frame);
    
    std::vector<std::string> instance2vector(std::string schemaInstance);

    std::string wget_json(std::string jury_ip, std::string jury_port, std::string filename);
    
    std::string read_json_from_file(std::string filename);

    void parse_json(std::string json_string);

};

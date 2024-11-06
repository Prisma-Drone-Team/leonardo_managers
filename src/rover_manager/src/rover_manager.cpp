#include "rover_manager.h"




RoverManager::RoverManager() : Node("rover_manager")
{
    this->client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    current_command = "";
    new_command = "";
    command_running = false;
    nav2_running = false;

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "seed_pdt_rover/command", 1,
        std::bind(&RoverManager::command_callback, this, std::placeholders::_1));
        
    rover_feedback_pb_ = this->create_publisher<std_msgs::msg::String>("seed_pdt_rover/state", 1);

    this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RoverManager::command_manager_callback, this));
        
    std::cout<<"rover_manager created"<<std::endl;
}


/**
*
* @param schemaInstance
* @return vettore di stringhe rappresentanti i parametri dello schema
* in versione prolog-like:
* EG.
*      vec[0]="nome schema"
*      vec[1]="primo parametro"
*      vec[2]="secondo parametro"
*      etc.
*
* eventuali parametri che siano essi stessi funtori vengono
* restituiti ugualmente come elemento del vettore
* EG.
*      vec[i]="fun1(fun2(x,y),z)"
*
* NOTE: This version also consider the [ ] as a list!
*/
std::vector<std::string> RoverManager::instance2vector(std::string schemaInstance){
    bool isAtom=true, isString=false;
    char c;
    std::string app;
    std::vector<std::string> result;
    std::stringstream ss(schemaInstance);
    int count=0;
    ss >> std::noskipws;
    //leggi il primo carattere della stringa
    ss>>c;
    //mentre non sei a fine stringa
    while(!ss.eof())
    {
        //se il carattere è un doppio apice e non sono in una stringa
        if(c=='"' && !isString){
            //allora sono in una stringa
            isString=true;
            //aggiungo l'apice
            app=app+c;
        }
        //se il carattere è un doppio apice e sono in una stringa
        else if(c=='"' && isString){
            //la stringa è finita
            isString=false;
            //aggiungo l'apice
            app=app+c;
            //aggiungila come elemento del funtore
            //result.push_back(app);
        }
        //mentre sono in una stringa
        else if(isString){
            //aggiungi il carattere senza controllarlo
            app=app+c;
        }
        //se sono un atomo ed il carattere letto è una parentesi aperta
        else if(c=='(' && isAtom){
            //non sono più un atomo
            isAtom=false;
            //inserisco il nome come primo elemento del vettore
            result.push_back(app);
            //pulisco la stringa d'appoggio
            app="";
            //salto la parentesi
//            ss>>c;
        }
        else if(c=='(' || c=='['){
            count++;
            app=app+c;
        }
        else if( ( c==')' || c==']' ) && count!=0){
            count--;
            app=app+c;
        }
        //se il carattere letto non è una virgola
        else if(c!=',' || count!=0)
            //aggiungilo alla stringa d'appoggio
            app=app+c;
        //altrimenti (ie. il carattere è una virgola)
        else {
            //inserisci la stringa d'appoggio nel vettore risultato
            result.push_back(app);
            //pulisci la stringa d'appoggio
            app="";
            //ho saltato la virgola
        }
        //leggi il successivo carattere
        ss>>c;
    }
    //se lo schema non ha parametri aggiungi il solo nome (vec[0])
    if(isAtom) {
        //check the \ character and split by it (added 01/12/2020 in seed 4.0)
        if( app.find('\\') != std::string::npos ){
            std::stringstream ss2(app);
            std::string substr;
            //std::cout<<"INSTANCE TO VECTOR: "<<schemaInstance<<std::endl;
            while(std::getline(ss2, substr, '\\')){
                result.push_back(substr);
                //std::cout<<"split: "<<substr<<std::endl;
            }
        }
        else
            result.push_back(app);
    }
    //altrimenti aggiungi l'ultima stringa rimuovendo l'ultima parentesi
    else{
        app.erase(app.size()-1);
        result.push_back(app);
    }
    //ritorna il vettore calcolato
    return result;
}



geometry_msgs::msg::TransformStamped RoverManager::get_tf(std::string source_frame, std::string target_frame)
{
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
        //transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        transform_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero);
        //RCLCPP_INFO(this->get_logger(), "Got transform: %f, %f, %f",
        //            transform_stamped.transform.translation.x,
        //            transform_stamped.transform.translation.y,
        //            transform_stamped.transform.translation.z);
    }
    catch (tf2::TransformException &ex) {
        std::cout<<"Could not transform"<<std::endl;
    }

    return transform_stamped;
}



void RoverManager::command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  new_command = msg->data.c_str();
}

void RoverManager::command_manager_callback()
{
    if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        return;
    }
    
    //std::cout<<"rover_manager, new_command: "<<new_command<<std::endl;
    //std::cout<<"rover_manager, current_command: "<<current_command<<std::endl;

    if(new_command != current_command){

      if(command_running){
        std::cout<<"rover_manager: switching from "<<current_command<<" to "<<new_command<<std::endl;

        if(nav2_running && current_command != "cancelling"){
            /*
          if(cancel_goal()){
            execute_command(new_command);
            current_command = new_command;
          }
          else{
            std::cout<<"rover_manager: unable to cancel "<<current_command<<std::endl;
          }
            */

	  std::cout<<"cancelling all nav2 goals "<<std::endl;
          this->client_ptr_->async_cancel_all_goals();
          current_command = "cancelling";
          //no the manager will wait for the goal to be cancelled (which sets nav2_running to FALSE)!
        }
        //NOTE: cancel here commands that are not executed through nav2!
      }
      else{
        std::cout<<"rover_manager: starting "<<new_command<<" from scretch"<<std::endl;

        execute_command(new_command);
        current_command = new_command;
        command_running = true;
      }

    }
}
/*
bool RoverManager::cancel_goal()
{
  bool canceled = false;
    if (goal_handle_) {
        RCLCPP_INFO(this->get_logger(), "Attempting to cancel goal");
        auto cancel_result_future = client_ptr_->async_cancel_goal(goal_handle_);
        try {
            const auto cancel_result = cancel_result_future.get();
            //if (cancel_result->return_code == rclcpp_action::CancelResponse::ACCEPTED) {
            if (cancel_result->return_code == rclcpp_action::CancelResponse::ACCEPT) {
                RCLCPP_INFO(this->get_logger(), "Goal successfully canceled");
                canceled = true;
                nav2_running = false;
            } else {
                RCLCPP_WARN(this->get_logger(), "Goal cancellation was rejected");
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Goal cancellation failed: %s", e.what());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "No goal to cancel");
    }

    return canceled;
}
*/

void RoverManager::execute_command(std::string cmd)
{
  std::vector<std::string> cv = instance2vector(cmd);

  if ( cv[0] == "goto" ) {
      //RCLCPP_INFO(this->get_logger(), "Rover going to %s", cv[1]);
      std::cout<<"Rover going to "<< cv[1] <<std::endl;
      auto target_tf = get_tf("rover/map", cv[1]);

      //get only the yaw!
      tf2::Quaternion quat(
        target_tf.transform.rotation.x,
        target_tf.transform.rotation.y,
        target_tf.transform.rotation.z,
        target_tf.transform.rotation.w);

      // Convert quaternion to RPY
      tf2::Matrix3x3 mat(quat);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);

      // Convert back to quaternion
      tf2::Quaternion target_quat;
      target_quat.setRPY(0.0, 0.0, yaw);

      auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();

      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.header.stamp = this->now();
      goal_msg.pose.pose.position.x = target_tf.transform.translation.x;
      goal_msg.pose.pose.position.y = target_tf.transform.translation.y;
      goal_msg.pose.pose.position.z = 0.0; //not used
      goal_msg.pose.pose.orientation.x = target_quat.x();
      goal_msg.pose.pose.orientation.y = target_quat.y();
      goal_msg.pose.pose.orientation.z = target_quat.z();
      goal_msg.pose.pose.orientation.w = target_quat.w();


      RCLCPP_INFO(this->get_logger(), "Sending goal to nav2");
      std::cout<<"\t going to "<<goal_msg.pose.pose.position.x<<", "<<goal_msg.pose.pose.position.y<<", "<<yaw<<std::endl;
      std::cout<<"\t quat "<<goal_msg.pose.pose.orientation.x<<", "<<goal_msg.pose.pose.orientation.y<<", "<<goal_msg.pose.pose.orientation.z<<", "<<goal_msg.pose.pose.orientation.w<<std::endl;

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&RoverManager::nav2_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&RoverManager::nav2_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&RoverManager::nav2_result_callback, this, std::placeholders::_1);

      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

      nav2_running = true;
  }else if (cv[0] == "wait"){
    //do nothing (previous goal already canceld)
    RCLCPP_INFO(this->get_logger(), "Rover waiting");
  }
  //NOTE: add here commands that are not executed through nav2
  else {
      //do nothing
      RCLCPP_INFO(this->get_logger(), "Rover stopped (command unknown)");
  }
  
}

//void RoverManager::nav2_goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future)
void RoverManager::nav2_goal_response_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>> future)
{
    goal_handle_ = future.get();
    if (!goal_handle_) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

//void RoverManager::nav2_feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>,
//                        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
void RoverManager::nav2_feedback_callback(std::shared_ptr<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>,
                        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    //RCLCPP_INFO(this->get_logger(), "Current position: (%.2f, %.2f)",
    //            feedback->current_pose.pose.position.x,
    //            feedback->current_pose.pose.position.y);
    
    //this is invoked continuously during the exxecution, for now we do nothing
}

void RoverManager::nav2_result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
            RCLCPP_INFO(this->get_logger(), "Goal was reached");

            current_command = "";
            command_running = false;
            nav2_running = false;

            break;
        }
        case rclcpp_action::ResultCode::CANCELED: {
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
            
            current_command = "";
            command_running = false;
            nav2_running = false;
            
            break;
        }
        case rclcpp_action::ResultCode::ABORTED: {
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
            
            //advise SEED that goal is not reachable
            std_msgs::msg::String failure_msg;
            failure_msg.data = instance2vector(current_command)[1] + ".unreachable";
            rover_feedback_pb_->publish(failure_msg);
            

            current_command = "";
            command_running = false;
            nav2_running = false;

            break;
        }
        default: {
            RCLCPP_INFO(this->get_logger(), "Unknown result code");
            
            current_command = "";
            command_running = false;
            nav2_running = false;

            break;
        }
    }

    //rclcpp::shutdown();
}







int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoverManager>();
    rclcpp::spin(node);
    
    // not stopping the robot!
    //node->all_stop();
    //sleep(1);
    
    rclcpp::shutdown();
    return 0;
}

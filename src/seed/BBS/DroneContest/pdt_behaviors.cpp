#include "pdt_behaviors.h"


// ***** ***** ***** PHOTO ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string PhotoBehaviour::behavior_name = "photo";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool PhotoBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&PhotoBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
PhotoBehaviour::PhotoBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    std::string topic = "/gcs_result";

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    was_active = false;
    now_active = false;
    
    was_accomplished = false;
    now_accomplished = false;

    window_open = false;

    mode = arg(2);

    have_image = false;

    dir_path = "/home/user/ros2_ws/src/ldc_images";

    video_frame = 0;

    //std::filesystem::create_directories(dir_path);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *PhotoBehaviour::create(std::string instance){
    return new PhotoBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void PhotoBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool PhotoBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    if(dead())
        wm_unlock();

    this->setRate(10);

    //check if this behavior is active now
    std::vector<WM_node *> me = WM->getNodesByInstance(getInstance());
    now_active = WM->isReleased(getInstance());
    now_accomplished = (me.size()>0 && !me[0]->goalStatus());

    //if the behavior was active but it is not anymore, stop subscription
    if(was_active && !now_active){
        sb_image.reset(); //this should stop subscription
    }
    //else, if the behavior was not active but it is now, start subscription
    else if(!was_active && now_active){

        if(SEED_NAME == "seed_pdt_drone"){
            sb_image = nh->create_subscription<sensor_msgs::msg::CompressedImage>("/aruco_detector/result_img/compressed", 
            rclcpp::SensorDataQoS(), std::bind(&PhotoBehaviour::img_callback, this, _1));
        }
        else if(SEED_NAME == "seed_pdt_rover"){

            if(arg(1) == "teddy_bear" || arg(1) == "traffic_light" || arg(1) == "plant" || arg(1) == "fire_hydrant" ){
                sb_image = nh->create_subscription<sensor_msgs::msg::CompressedImage>("/yolo/prediction/image/compressed", 
                rclcpp::SensorDataQoS(), std::bind(&PhotoBehaviour::img_callback, this, _1));
            }
            else{
                sb_image = nh->create_subscription<sensor_msgs::msg::CompressedImage>("/rover_aruco_detector/result_img/compressed", 
                rclcpp::SensorDataQoS(), std::bind(&PhotoBehaviour::img_callback, this, _1));
            }

        }
    }
    was_active = now_active;
    was_accomplished = now_accomplished;

    wm_unlock();
    

    return true;

}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void PhotoBehaviour::motorSchema(){
    // write CUSTOM code here...

    if(!have_image)
        return;

    have_image = false;
    current_image = ros_image.clone();

    std::cout<<"RUNNING PHOTO"<<std::endl;

    //check which mode
    if(mode == "once"){
        initialize_window(getInstance());
        cv::Mat display_image = create_selection_interface(current_image);
        cv::imshow(getInstance(), display_image);
        
        if(wait_for_click() && image_accepted){
            save_image(mode,arg(1));

            wm_lock();
            std::string var = arg(1) + "." + arg(2) + ".confirmed";
            wmv_set<bool>(var,true);
            wm_unlock();

            //publish result to gcs
            //result format: (id, current_timestamp, image_path)
            std_msgs::msg::String msg;
            msg.data = "(" + arg(3) + "," + current_timestamp + "," + dir_path + "/" + arg(1) + "_" + mode + ".png" + ")";
            pbs->publish(msg);
            std::cout<<"SENDING TO GCS: "<<msg.data<<std::endl;
        }
        
        
        cv::destroyWindow(getInstance());
        window_open = false;
        cv::waitKey(1);
        std::cout<<"window destroyed"<<std::endl;
    }
    else if(mode == "first"){
        initialize_window(getInstance());
        cv::Mat display_image = create_selection_interface(current_image);
        cv::imshow(getInstance(), display_image);
        
        if(wait_for_click() && image_accepted){
            wm_lock();
            std::string var = arg(1) + "." + arg(2) + ".confirmed";
            wmv_set<bool>(var,true);
            wm_unlock();

            //old_image = current_image.clone();
            save_image(mode,arg(1));

            //publish result to gcs ?
        }
        
        cv::destroyWindow(getInstance());
        window_open = false;
        cv::waitKey(1);
        std::cout<<"window destroyed"<<std::endl;
    }
    else if(mode == "second"){
        initialize_window(getInstance());
        cv::Mat display_image = create_selection_interface(current_image);
        cv::imshow(getInstance(), display_image);

        if(wait_for_click() && image_accepted){
            save_image(mode,arg(1));

            wm_lock();
            std::string var = arg(1) + "." + arg(2) + ".confirmed";
            wmv_set<bool>(var,true);
            wm_unlock();

            //publish result to gcs
            std_msgs::msg::String msg;
            msg.data = "(" + arg(3) + "," + current_timestamp + "," + dir_path + "/" + arg(1) + "_" + mode + ".png" + ")";
            pbs->publish(msg);
            std::cout<<"SENDING TO GCS: "<<msg.data<<std::endl;
        }
        
        cv::destroyWindow(getInstance());
        window_open = false;
        cv::waitKey(1);
        std::cout<<"window destroyed"<<std::endl;
    }
    //otherwise, we have to take a video!
    else {
        save_image(mode, arg(1));
        sleep(1); // around 1 fps
    }
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void PhotoBehaviour::exit(){
    // write CUSTOM code here...

    if(window_open)
        cv::destroyWindow(getInstance());

    wm_lock();

    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}

void PhotoBehaviour::img_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
    //std::cout<<"IMAGE RECEIVED!!"<<std::endl;
    //cv_bridge::CvImagePtr cv_ptr;
    //try {
    //    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //} catch (cv_bridge::Exception& e) {
    //    RCLCPP_ERROR(nh->get_logger(), "cv_bridge exception: %s", e.what());
    //    return;
    //}
    //cv::imshow("view", cv_ptr->image);
    //cv::waitKey(1);

    try {
        //cv::Mat ros_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        //ros_image.convertTo(current_image, CV_8U, 255.0);
        ros_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        
        if(!ros_image.empty()) {
            // Aggiungi timestamp e data
            current_timestamp = format_timestamp_with_date(msg->header.stamp);
            add_timestamp_to_image(ros_image, current_timestamp);

            have_image = true;
        }
        else
            std::cout<<"EMPTY IMAGE RECEIVED!!"<<std::endl;
    }
    catch (const std::exception& e) {
        std::cout<<"EXECPTION ON IMAGE CALLBACK: "<<*e.what()<<std::endl;
    }
}




// IMAGE RELATED FUNCTIONS

bool PhotoBehaviour::wait_for_click(){
    clicked = false;
    while(!clicked){
        cv::waitKey(1);

        wm_lock();
        if(WM->getNodesByInstance(getInstance()).size() == 0 || !WM->isReleased(getInstance()) ){
            std::cout<<this->getInstance()<<" forgotten or disabled!"<<std::endl;
            wm_unlock();
            return false;
        }
        wm_unlock();
    }
    std::cout<<"CLICKED!"<<std::endl;
    return true;
}

//OK
void PhotoBehaviour::initialize_window(std::string window_name) {
    std::cout<<"window creation:"<<std::endl;
    // Crea finestre con dimensioni fisse per consistenza
    cv::namedWindow(window_name, cv::WINDOW_NORMAL | cv::WINDOW_GUI_EXPANDED);
    std::cout<<"window created"<<std::endl;
    
    // Imposta dimensioni e posizione
    cv::resizeWindow(window_name, 800, 600);
    std::cout<<"window resized"<<std::endl;
    cv::moveWindow(window_name, 100, 100); //to be randomized
    std::cout<<"window moved"<<std::endl;
    
    // Callback separati per ogni finestra
    cv::setMouseCallback(window_name, &PhotoBehaviour::mouse_callback, this);
    std::cout<<"window cb created"<<std::endl;

    window_open = true;
}


cv::Mat PhotoBehaviour::create_selection_interface(cv::Mat image) {
    // Aggiungi area per i pulsanti
    cv::Mat display_image;
    int button_area_height = 80;
    cv::copyMakeBorder(image, display_image, 0, button_area_height, 0, 0, 
                        cv::BORDER_CONSTANT, cv::Scalar(40, 40, 40));
    
    // Crea pulsanti
    int button_width = 120;
    int button_height = 40;
    int button_y = image.rows + 20;
    int button_spacing = 20;
    
    // Pulsante SAVE (verde)
    yes_rect = cv::Rect(
        display_image.cols/2 - button_width - button_spacing/2, 
        button_y, button_width, button_height
    );
    cv::rectangle(display_image, yes_rect, cv::Scalar(0, 180, 0), -1);

    cv::putText(display_image, "SAVE", 
                cv::Point(yes_rect.x + 35, yes_rect.y + 27),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    
    // Pulsante DISCARD (rosso)
    no_rect = cv::Rect(
        display_image.cols/2 + button_spacing/2, 
        button_y, button_width, button_height
    );
    cv::rectangle(display_image, no_rect, cv::Scalar(0, 0, 180), -1);

    cv::putText(display_image, "DISCARD", 
                cv::Point(no_rect.x + 15, no_rect.y + 27),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
    
    return display_image;
}



// Callback per mouse separati
void PhotoBehaviour::mouse_callback(int event, int x, int y, int flags, void* userdata) {
    PhotoBehaviour* self = static_cast<PhotoBehaviour*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN) {
        self->handle_selection_click(x, y);
    }
}

void PhotoBehaviour::handle_selection_click(int x, int y) {
    if (yes_rect.contains(cv::Point(x, y))) {
        image_accepted = true;
    }
    else if (no_rect.contains(cv::Point(x, y))) {
        image_accepted = false;
    }
    clicked = true;
}


void PhotoBehaviour::save_image(std::string mode, std::string filename){
    if(mode == "once" || mode == "first"){
        std::string full_path = dir_path + "/" + filename + "_" + mode + ".png";
        cv::imwrite(full_path, current_image);
    }
    else if(mode == "second"){
        old_image = cv::imread(dir_path + "/" + filename + "_first.png");
        cv::Mat combined_image;
        cv::hconcat(old_image, current_image, combined_image);

        std::string full_path = dir_path + "/" + filename + "_" + mode + ".png";
        cv::imwrite(full_path, combined_image);
    }
    else{
        //is a video
        std::string frame = std::to_string(video_frame);
        std::string full_path = dir_path + "/sequence/" + filename + "_" + frame + ".png";
        cv::imwrite(full_path, current_image);

        video_frame++;
    }

}



// Utility functions
std::string PhotoBehaviour::format_timestamp_with_date(const builtin_interfaces::msg::Time& timestamp) {
    auto seconds = timestamp.sec;
    std::time_t time_t_seconds = seconds;
    std::tm tm = *std::localtime(&time_t_seconds);
    
    std::stringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

void PhotoBehaviour::add_timestamp_to_image(cv::Mat& image, const std::string& timestamp) {
    cv::putText(image, timestamp, cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
}

std::string PhotoBehaviour::get_current_time_string() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}

























// ***** ***** ***** COVER ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string CoverBehaviour::behavior_name = "cover";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool CoverBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&CoverBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
CoverBehaviour::CoverBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    contended_variable = SEED_NAME + ".command";
    topic = SEED_NAME + "/command";

    command = getInstance();

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *CoverBehaviour::create(std::string instance){
    return new CoverBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void CoverBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool CoverBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    /*
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer->lookupTransform("drone_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout<<"EXCEPTION: unable to lookup transform to "<<target_frame<<std::endl;
    }
    */

    wmv_compete<std::string>("none", contended_variable, command);

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void CoverBehaviour::motorSchema(){
    // write CUSTOM code here...

    wm_lock();
    
   if(wmv_solve_once<std::string>(contended_variable) == command){
        std_msgs::msg::String msg;
        msg.data = command;
        pbs->publish(msg);
    }
    //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void CoverBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_withdraw<std::string>(contended_variable);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}





// ***** ***** ***** EXPLORE ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string ExploreBehaviour::behavior_name = "explore";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool ExploreBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&ExploreBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
ExploreBehaviour::ExploreBehaviour(std::string instance): rnd(std::random_device{}()){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    //static elements
    //explorables.push_back("exp1");
    //explorables.push_back("exp2");
    //explorables.push_back("exp3");
    //explorables.push_back("exp4");

    //get from ROS2 param
    std::vector<std::string> default_exp = { "exp00", "exp01", "exp10", "exp11" };
    nh->declare_parameter("frames_to_explore", default_exp);
    nh->get_parameter("frames_to_explore",explorables);

    std::fill_n(std::back_inserter(weights), explorables.size(), 0.0);
    std::fill_n(std::back_inserter(w_time), explorables.size(), 0.0);
    current_target = -1;

    std::cout<<ansi::red<<"I' AM "<<SEED_NAME<<ansi::end<<std::endl;

    if(SEED_NAME == "seed_pdt_drone")
        pb = nh->create_publisher<std_msgs::msg::String>("seed_pdt_rover/state", 1);
    else if(SEED_NAME == "seed_pdt_rover")
        pb = nh->create_publisher<std_msgs::msg::String>("seed_pdt_drone/state", 1);
    else if(SEED_NAME == "seed_pdt_ptzcam")
        pb = nh->create_publisher<std_msgs::msg::String>("seed_pdt_ptzcam/state", 1);
    else if(SEED_NAME == "seed_inspect_drone")
        pb = nh->create_publisher<std_msgs::msg::String>("seed_inspect_drone/state", 1);

    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *ExploreBehaviour::create(std::string instance){
    return new ExploreBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void ExploreBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool ExploreBehaviour::perceptualSchema(){
    // write CUSTOM code here...
    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void ExploreBehaviour::motorSchema(){
    // write CUSTOM code here...

    //start new exploration if none is running
    wm_lock();

    //update weights
    update_weights();

    std::vector<WM_node*> children = wm_get_child_nodes()[0];

    if(children.size()==0){
        //start a new exploration

        current_target = sample_element();
        std::cout<<"SELECTED: "<<current_target<<std::endl;

        std::stringstream ss;
        //fly if drone, go otherwise
        if(SEED_NAME == "seed_pdt_drone")
            ss<<"flyto("<<explorables[current_target]<<")";
        else if(SEED_NAME == "seed_pdt_rover")
            ss<<"goto("<<explorables[current_target]<<")";
        else if(SEED_NAME == "seed_pdt_ptzcam")
            ss<<"watchto("<<explorables[current_target]<<")";
        else if(SEED_NAME == "seed_inspect_drone")
            ss<<"flyto("<<explorables[current_target]<<")";

        //add new son
        wm_add_child_node(ss.str());

        std::cout<<"new explorative action "<<ss.str()<<" SELECTED with weight "<<weights[current_target]<<std::endl;
    }
    // if action is accomplished
    else if(children[0]->goalStatus()){
        std::cout<<"current explorative action "<<children[0]->instance<<" ACCOMPLISHED"<<std::endl;
        wm_remove_child_node(children[0]->instance);

        //weights[current_target] = 0.0;
        w_time[current_target] = 0.0; //reset time-based component
    }
    // if action is failed
    else if(wmv_get<bool>(explorables[current_target] + ".unreachable")){
        std::cout<<"current explorative action "<<children[0]->instance<<" FAILED"<<std::endl;
        wm_remove_child_node(children[0]->instance);

        //tell other agent to prioritize the target
        std_msgs::msg::String msg;
        msg.data = explorables[current_target] + ".prioritized";
        pb->publish(msg);
    }
    //oth. let the current node works..

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void ExploreBehaviour::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}


void ExploreBehaviour::update_weights(){
    double k_reach = 0.3; //increase by 30%
    double k_time = 0.1; //0.1 every tic (it should be every second)

    for(auto i=0; i<explorables.size(); i++){
        //if target is still reachable
        if(!wmv_get<bool>(explorables[i]+".unreachable")){
            
                //NOTE: we can also use the distance (explorables[i]+".distance") to weight the targets
        
            //update the time-based component of the weight
            w_time[i] = w_time[i] + k_time;
            //if target is requessted (i.e. other agents are unable to reach it)
            if(wmv_get<bool>(explorables[i]+".prioritized"))
                //apply the reach bonus
                weights[i] = w_time[i] + k_reach*w_time[i];
            else
                weights[i] = w_time[i];

        }
        //otherwise, target is unreachable
        else
            //it must not be selected
            weights[i] = 0.0;
    }
}

int ExploreBehaviour::sample_element(){
    //weights no need to sum to 1!
    std::cout<<"SAMPLING FROM: "<<std::endl;
    for(auto i=0; i<weights.size(); i++){
        std::cout<<i<<": "<<weights[i]<<std::endl;
    }

    // size_t is suitable for indexing.
    std::discrete_distribution<std::size_t> d{weights.begin(), weights.end()};
    
    //randomize seed
    rnd.seed(std::time(nullptr));

    return d(rnd);
}



// ***** ***** ***** PATH ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string PathBehaviour::behavior_name = "path";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool PathBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&PathBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
PathBehaviour::PathBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...
        
    //get from ROS2 param
    nh->declare_parameter("path", std::vector<std::string>());
    nh->get_parameter("path",path);
    current_target = 0;

    std::cout<<ansi::red<<"I' AM "<<SEED_NAME<<ansi::end<<std::endl;

    if(SEED_NAME == "seed_pdt_drone")
        pb = nh->create_publisher<std_msgs::msg::String>("seed_pdt_rover/state", 1);
    else if(SEED_NAME == "seed_pdt_rover")
        pb = nh->create_publisher<std_msgs::msg::String>("seed_pdt_drone/state", 1);
    else if(SEED_NAME == "seed_pdt_ptzcam")
        pb = nh->create_publisher<std_msgs::msg::String>("seed_pdt_ptzcam/state", 1);
    else if(SEED_NAME == "seed_inspect_drone")
        pb = nh->create_publisher<std_msgs::msg::String>("seed_inspect_drone/state", 1);

    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *PathBehaviour::create(std::string instance){
    return new PathBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void PathBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool PathBehaviour::perceptualSchema(){
    // write CUSTOM code here...
    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void PathBehaviour::motorSchema(){
    // write CUSTOM code here...
    
    //start new exploration if none is running
    wm_lock();

    std::vector<WM_node*> children = wm_get_child_nodes()[0];
    
    if(current_target >= path.size()){
        wm_unlock();
        return;
    }

    if(children.size()==0){
        //start a new waypoint
        std::cout<<"CURRENT WP: "<<current_target<<std::endl;

        std::stringstream ss;
        //fly if drone, go otherwise
        if(SEED_NAME == "seed_pdt_drone")
            ss<<"flyto("<<path[current_target]<<")";
        else if(SEED_NAME == "seed_pdt_rover")
            ss<<"goto("<<path[current_target]<<")";
        else if(SEED_NAME == "seed_pdt_ptzcam")
            ss<<"watchto("<<path[current_target]<<")";
        else if(SEED_NAME == "seed_inspect_drone")
            ss<<"flyto("<<path[current_target]<<")";

        //add new son
        wm_add_child_node(ss.str());

        std::cout<<"new navigation action "<<ss.str()<<" STARTED"<<std::endl;
    }
    // if action is accomplished
    else if(children[0]->goalStatus()){
        std::cout<<"current navigation action "<<children[0]->instance<<" ACCOMPLISHED"<<std::endl;
        wm_remove_child_node(children[0]->instance);
        current_target++;
    }
    // if action is failed
    else if(wmv_get<bool>(path[current_target] + ".unreachable")){
        std::cout<<"current explorative action "<<children[0]->instance<<" FAILED"<<std::endl;
        wm_remove_child_node(children[0]->instance);

        //tell other agent to prioritize the target
        std_msgs::msg::String msg;
        msg.data = path[current_target] + ".prioritized";
        pb->publish(msg);
        current_target++;
    }
    //oth. let the current node works..

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void PathBehaviour::exit(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}


// ********** ********************** **********//
// ********** CONCRETE PDT BEHAVIORS **********//
// ********** ********************** **********//



// ***** ***** ***** TAKE PICTURE ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string TakePictureBehaviour::behavior_name = "takePicture";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool TakePictureBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&TakePictureBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
TakePictureBehaviour::TakePictureBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    done = false;

    pb_logger = nh->create_publisher<std_msgs::msg::String>("logger/command", 1);

    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *TakePictureBehaviour::create(std::string instance){
    return new TakePictureBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void TakePictureBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool TakePictureBehaviour::perceptualSchema(){
    // write CUSTOM code here...
    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void TakePictureBehaviour::motorSchema(){
    // write CUSTOM code here...

    if(done) return;

    wm_lock();
    
    std_msgs::msg::String msg;
    msg.data = "shoot";
    pb_logger->publish(msg);

    //THIS SHOULD BE PUBLISHED BY THE LOGGER
    //std_msgs::msg::String msg2;
    //msg2.data = "picture.done";
    //pb_all->publish(msg2);
    
    done = true;

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void TakePictureBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}




// ***** ***** ***** WATCH TO ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string WatchToBehaviour::behavior_name = "watchto";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool WatchToBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&WatchToBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
WatchToBehaviour::WatchToBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    target_frame = arg(1);
    contended_variable = SEED_NAME + ".command";
    topic = SEED_NAME + "/command";
    command = getInstance();

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *WatchToBehaviour::create(std::string instance){
    return new WatchToBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void WatchToBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool WatchToBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    /*
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer->lookupTransform("drone_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout<<"EXCEPTION: unable to lookup transform to "<<target_frame<<std::endl;
    }
    */

    wmv_compete<std::string>("none", contended_variable, command);

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void WatchToBehaviour::motorSchema(){
    // write CUSTOM code here...

    wm_lock();
    
   if(wmv_solve_once<std::string>(contended_variable) == command){
        std_msgs::msg::String msg;
        msg.data = command;
        pbs->publish(msg);
    }
    //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void WatchToBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_withdraw<std::string>(contended_variable);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}




// ***** ***** ***** FLY TO ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string FlyToBehaviour::behavior_name = "flyto";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool FlyToBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&FlyToBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
FlyToBehaviour::FlyToBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    target_frame = arg(1);
    contended_variable = SEED_NAME + ".command";
    topic = SEED_NAME + "/command";
    command = "flyto(" + arg(1) + ")";

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *FlyToBehaviour::create(std::string instance){
    return new FlyToBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void FlyToBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool FlyToBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    /*
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer->lookupTransform("drone_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout<<"EXCEPTION: unable to lookup transform to "<<target_frame<<std::endl;
    }
    */

    wmv_compete<std::string>("none", contended_variable, command);

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void FlyToBehaviour::motorSchema(){
    // write CUSTOM code here...

    wm_lock();
    
   if(wmv_solve_once<std::string>(contended_variable) == command){
        std_msgs::msg::String msg;
        msg.data = command;
        pbs->publish(msg);
    }
    //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void FlyToBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_withdraw<std::string>(contended_variable);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}




// ***** ***** ***** GO TO ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string GoToBehaviour::behavior_name = "goto";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool GoToBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&GoToBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
GoToBehaviour::GoToBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    target_frame = arg(1);
    contended_variable = SEED_NAME + ".command";
    topic = SEED_NAME + "/command";
    command = "goto(" + arg(1) + ")";

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *GoToBehaviour::create(std::string instance){
    return new GoToBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void GoToBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool GoToBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    /*
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer->lookupTransform("drone_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout<<"EXCEPTION: unable to lookup transform to "<<target_frame<<std::endl;
    }
    */

    wmv_compete<std::string>("none", contended_variable, command);

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void GoToBehaviour::motorSchema(){
    // write CUSTOM code here...

    wm_lock();
    
   if(wmv_solve_once<std::string>(contended_variable) == command){
        std_msgs::msg::String msg;
        msg.data = command;
        pbs->publish(msg);
    }
    //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void GoToBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_withdraw<std::string>(contended_variable);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}




// ***** ***** ***** CIRCLE ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string CircleBehaviour::behavior_name = "circle";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool CircleBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&CircleBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
CircleBehaviour::CircleBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    target_frame = arg(1);
    contended_variable = SEED_NAME + ".command";
    topic = SEED_NAME + "/command";
    command = "flyto(circle(" + target_frame + "))";

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *CircleBehaviour::create(std::string instance){
    return new CircleBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void CircleBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool CircleBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    /*
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer->lookupTransform("drone_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout<<"EXCEPTION: unable to lookup transform to "<<target_frame<<std::endl;
    }
    */

    wmv_compete<std::string>("none", contended_variable, command);

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void CircleBehaviour::motorSchema(){
    // write CUSTOM code here...

    wm_lock();
    
   if(wmv_solve_once<std::string>(contended_variable) == command){
        std_msgs::msg::String msg;
        msg.data = command;
        pbs->publish(msg);
    }
    //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void CircleBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_withdraw<std::string>(contended_variable);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}




// ***** ***** ***** TAKE OFF ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string TakeOffBehaviour::behavior_name = "takeoff";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool TakeOffBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&TakeOffBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
TakeOffBehaviour::TakeOffBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    contended_variable = SEED_NAME + ".command";
    topic = SEED_NAME + "/command";
    command = getInstance();

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *TakeOffBehaviour::create(std::string instance){
    return new TakeOffBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void TakeOffBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool TakeOffBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    /*
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer->lookupTransform("drone_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout<<"EXCEPTION: unable to lookup transform to "<<target_frame<<std::endl;
    }
    */

    wmv_compete<std::string>("none", contended_variable, command);

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void TakeOffBehaviour::motorSchema(){
    // write CUSTOM code here...

    wm_lock();
    
   if(wmv_solve_once<std::string>(contended_variable) == command){
        std_msgs::msg::String msg;
        msg.data = command;
        pbs->publish(msg);
    }
    //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void TakeOffBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_withdraw<std::string>(contended_variable);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}




// ***** ***** ***** LAND ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string LandBehaviour::behavior_name = "land";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool LandBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&LandBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
LandBehaviour::LandBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    contended_variable = SEED_NAME + ".command";
    topic = SEED_NAME + "/command";
    command = getInstance();

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *LandBehaviour::create(std::string instance){
    return new LandBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void LandBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool LandBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    /*
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer->lookupTransform("drone_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout<<"EXCEPTION: unable to lookup transform to "<<target_frame<<std::endl;
    }
    */

    wmv_compete<std::string>("none", contended_variable, command);

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void LandBehaviour::motorSchema(){
    // write CUSTOM code here...

    wm_lock();
    
   if(wmv_solve_once<std::string>(contended_variable) == command){
        std_msgs::msg::String msg;
        msg.data = command;
        pbs->publish(msg);
    }
    //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void LandBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_withdraw<std::string>(contended_variable);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}




// ***** ***** ***** WAIT ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string WaitBehaviour::behavior_name = "wait";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool WaitBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&WaitBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
WaitBehaviour::WaitBehaviour(std::string instance){
    // please set the instance before anything else
    setInstance(instance);
    // write CUSTOM construction code here...

    contended_variable = SEED_NAME + ".command";
    topic = SEED_NAME + "/command";
    command = getInstance();

    pbs = nh->create_publisher<std_msgs::msg::String>(topic, 1);

    
    std::cout<<arg(0)<<": Constructor() executed "<<std::endl;
}

// 2. DEFINE the create function.
//  this function MUST return the reference to the currente behavior.
Behavior *WaitBehaviour::create(std::string instance){
    return new WaitBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void WaitBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool WaitBehaviour::perceptualSchema(){
    // write CUSTOM code here...

    wm_lock();

    /*
    tf_buffer = std::make_unique<tf2_ros::Buffer>(nh->get_clock());

    try {
        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer->lookupTransform("drone_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
        std::cout<<"EXCEPTION: unable to lookup transform to "<<target_frame<<std::endl;
    }
    */

    wmv_compete<std::string>("none", contended_variable, command);

    wm_unlock();

    return true;
}

// 5. DEFINE the motorSchema.
//  this function is executed at the specified frequency (+ delay of perceptualSchema), 
//  but only if: 
//      - the perceptualSchema returns true
//      - all releasers along the WM branch are true
//      - all goals along the WM branch are false
//  if these conditions are not satisfied, only perceptualSchema is executed.
void WaitBehaviour::motorSchema(){
    // write CUSTOM code here...

    wm_lock();
    
   if(wmv_solve_once<std::string>(contended_variable) == command){
        std_msgs::msg::String msg;
        msg.data = command;
        pbs->publish(msg);
    }
    //NOTE: if no one wins (e.g., ex-aequo) nothing is published!

    wm_unlock();
}

// 6. DEFINE the exit function.
//  this function executed only once at the end of the execution
void WaitBehaviour::exit(){
    // write CUSTOM code here...

    wm_lock();
    wmv_withdraw<std::string>(contended_variable);
    wmv_set<bool>(this->getInstance() + ".done", false);
    wm_unlock();

    std::cout<<arg(0)<<": exit() executed "<<std::endl;
}

#include "pdt_behaviors.h"


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
    nh->declare_parameter("frames_to_explore", std::vector<std::string>());
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
    command = getInstance();

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
    command = getInstance();

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




// ***** ***** ***** FOLLOW ***** ***** ***** //

// REGISTER this behavior into the Behavior-based System (BBS).
//  This is a 2 steps process...

// 1. PROVIDE a unique name to this behavior
//  the behavior will be identified by this name
std::string FollowBehaviour::behavior_name = "follow";

// 2. ADD this behavior (identified by the behavior_name) to the BBS for possible activation
bool FollowBehaviour::registered = BehaviorBasedSystem::add(behavior_name,&FollowBehaviour::create);

// ...now your behavior has been registered, you may recall it by loading
//  a predicate in the form "<behavior_name>(<Arg1>, <Arg2>, ..., <ArgN>)" to the WM



// DEFINE the code of this behavior.
//  This is a 6 steps process...

// 1. DEFINE the constructor for this beahvior, it will be executed when this behavior is recalled.
//
//      instance: is the string containing the predicate that has been used to invoke ths behavior.
//                NOTE: if no paramaters (i.e., further args) are given then instance == behavior_name
FollowBehaviour::FollowBehaviour(std::string instance){
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
Behavior *FollowBehaviour::create(std::string instance){
    return new FollowBehaviour(instance);
}


// now let's implement the CUSTOM core of our behavior (virtual functions).

// 3. DEFINE the start function.
//  this function executed only once at the beginning of the execution
void FollowBehaviour::start(){
    // write CUSTOM code here...
    std::cout<<arg(0)<<": start() executed "<<std::endl;
}

// 4. DEFINE the perceptualSchema.
//  this function is executed at the specified frequency.
//      If TRUE is returned, the motorSchema may be executed
bool FollowBehaviour::perceptualSchema(){
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
void FollowBehaviour::motorSchema(){
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
void FollowBehaviour::exit(){
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

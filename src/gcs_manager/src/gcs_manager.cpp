#include "gcs_manager.h"




GCSManager::GCSManager() : Node("gcs_manager")
{
/*
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
*/

    std::string json_string = read_json_from_file("DC2024");
    parse_json(json_string);
    std::cout<<"bye"<<std::endl;
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
std::vector<std::string> GCSManager::instance2vector(std::string schemaInstance){
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



geometry_msgs::msg::TransformStamped GCSManager::get_tf(std::string source_frame, std::string target_frame)
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


std::string GCSManager::wget_json(std::string jury_ip, std::string jury_port, std::string filename){
    //let's try with a simple system call for wget
    std::stringstream call;
    call<<"wget http://"<<jury_ip<<":"<<jury_port<<" -o /home/user/ros2_ws/src/gcs_manager/missions/"<<filename<<".json";
    std::cout<<"GCS: executing wget"<<std::endl;
    system(call.str().c_str());
    std::cout<<"GCS: wget executed"<<std::endl;
    
    return read_json_from_file(filename);
}

std::string GCSManager::read_json_from_file(std::string filename){
    std::cout<<"GCS: reading file"<<std::endl;
    std::ifstream file("/home/user/ros2_ws/src/gcs_manager/missions/" + filename + ".Json"); 
    std::string str;
    std::string file_contents;
    while (std::getline(file, str))
    {
      file_contents += str;
      file_contents.push_back('\n');
    }  

    file.close();
    
    std::cout<<"GCS: JSON FILE: "<<std::endl;
    std::cout<<file_contents<<std::endl;

    return file_contents;
}

void GCSManager::parse_json(std::string json_string){
    //TODO
    
    Json::Reader reader;
    Json::Value js;
    
    reader.parse(json_string, js);
    std::cout<<"manche: "<<js["manche"].asString()<<std::endl;
    std::cout<<"markers: "<<js["markers"].asString()<<std::endl;
}






int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GCSManager>();
    rclcpp::spin(node);
    
    // not stopping the robot!
    //node->all_stop();
    //sleep(1);
    
    rclcpp::shutdown();
    return 0;
}

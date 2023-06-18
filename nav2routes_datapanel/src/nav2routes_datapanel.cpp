#include "nav2routes_datapanel/nav2routes_datapanel.h"
#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "custom_interfaces/srv/navroutes_service_message.hpp"
#include "custom_interfaces/srv/hospitals_service_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QInputDialog>
#include <QComboBox>
#include <QStringList>
#include <sstream>
//#include <String>


namespace nav2routes_datapanel 
{
        // *****************************************//
        // ******** constructor definition *********//
        // *****************************************//

    Nav2routesDataPanel::Nav2routesDataPanel(QWidget *parent) //
            : rviz_common::Panel(parent)
    {
        // *****************************************//
        // *********** panel definition ************//
        // *****************************************//

        auto layout = new QGridLayout(this);
        layout->setContentsMargins(10, 10, 10, 10);

        // hospitals dropdown box definition
        hospital_dropdown_ = new QComboBox(this);
        hospital_dropdown_->setFixedWidth(250);
        hospital_dropdown_->setGeometry(10, 70, 150, 30);
   
        // rooms dropdown box definition
        room_dropdown_ = new QComboBox(this);
        room_dropdown_->setGeometry(10, 110, 150, 30);
     
        // routes dropdown box definition
        route_dropdown_ = new QComboBox(this);
        route_dropdown_->setGeometry(10, 150, 150, 30);       

        // labels definition
        auto title = new QLabel("Akara Nav2Routes Data Panel");
        auto subtitle = new QLabel("Choose: hospital, room and route.");
        auto hospital = new QLabel("Hospital:");
        auto room = new QLabel("Room:");
        auto route = new QLabel("Route:");

        //auto lineEdit_room = new QLineEdit(); // to be deleted
       // auto lineEdit_route = new QLineEdit(); // to be deleted
        
        // button definition
        auto pushbutton = new QPushButton("display route");   

        // definition of the menu structure //
        layout->addWidget(title, 0, 0, 1, 2);
        layout->addWidget(subtitle, 1, 0, 1, 2);
        layout->addWidget(hospital, 2, 0);
        layout->addWidget(hospital_dropdown_, 2, 1);
        layout->addWidget(room, 3, 0);
        layout->addWidget(room_dropdown_, 3, 1);
        layout->addWidget(route, 4, 0);
        layout->addWidget(route_dropdown_, 4, 1);
        layout->addWidget(pushbutton, 5, 0, 1, 2);

        setLayout(layout);
        
        // *****************************************//
        // *********** client definition ***********//
        // ***********  display routes *************//
        // *****************************************//

        pushbutton->connect(pushbutton, &QPushButton::clicked, [=]() 
        {
       
            // clicking the button, the client calls the server //
            auto request_routes = std::make_shared<custom_interfaces::srv::NavroutesServiceMessage::Request>();               
            request_routes->room = set_room;        
            request_routes->route = set_route_id;
            auto client_route_display = rclcpp::Node::make_shared("nav2routes_datadisplay_client");
            auto service_client_route_display = client_route_display->create_client<custom_interfaces::srv::NavroutesServiceMessage>(
                "nav2routes_datadisplay_server");
        
            // When the server responds the following function is executed //
            auto response_received = [](rclcpp::Client<custom_interfaces::srv::NavroutesServiceMessage>::SharedFuture future) 
            {
                auto response = future.get();
                if (response->success) 
                {
                    qDebug() << "Route successfully displayed";
                } 
                else 
                {
                    qDebug() << "Route not found";
                }
            };

            // this block sends the request to server and waits the answer //
            auto future = service_client_route_display->async_send_request(request_routes);
            rclcpp::spin_until_future_complete(client_route_display, future, std::chrono::seconds(5));

            // Response received managing
            if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) 
            {
                response_received(std::move(future));
            } 
            else 
            {
                qDebug() << "Service call timed out";
            }  
                qDebug() << "Button pushed!";
                //qDebug() << lineEdit_route->text().toInt();
        });


    }

    void Nav2routesDataPanel::onInitialize()
  {
        // *****************************************//
        // *********** client definition ***********//
        // *********  hospitals dictionary *********//
        // *****************************************//
        // starting from keys and values of a flatten dictionary constructs the hospitals original dictionary
        // The hospitals_dict to make work drop-down menu

        auto client_hospitals_dict = rclcpp::Node::make_shared("navroutes_datareader_client");
        auto service_client_hospitals_dict = client_hospitals_dict->create_client<custom_interfaces::srv::HospitalsServiceMessage>(
            "hospitals_datareader_service");    

        auto response_received = [this](rclcpp::Client<custom_interfaces::srv::HospitalsServiceMessage>::SharedFuture future) {
            auto response = future.get();
            if (response->success) 
            {
                std::string encoded_data_string = response->encoded_dict;
                nlohmann::json decoded_data = nlohmann::json::parse(encoded_data_string);
                qDebug() << "hospitals dictionary successfuly received";
               
                // It extracts the dictionary data and populates the drop-down menus.
                // It builds also the QStringList and maps of hospitals, rooms and routes rispectively.
                // level 1 (first key of dict: ex. st. james) populates the hospitals 
                // to undestand the levels see json dictionary structure below.               
                int i = 0; // hospital order number
                for (auto& level1 : decoded_data.items()) {
                //qDebug() << "calling for cycle...";
                std::string key = level1.key();
                nlohmann::json value = level1.value();
                hospitals.append(QString::fromStdString(key));   
                //qDebug() << "Hospital: " << QString::fromStdString(key);            

                    // level 2 (second key of dict: ex. delivery room) populates the rooms 
                    int j = 0; // room order number
                    rooms.clear();
                    for (auto& level2 : value.items()) {
                        std::string nestedKey = level2.key();
                        nlohmann::json nestedValue = level2.value();
                        //qDebug() << "Level 2 - Key: " << QString::fromStdString(nestedKey);
                        //qDebug() << "Level 2 - Value: " << QString::fromStdString(nestedValue.dump());

                        // level 3
                        for (auto& level3 : nestedValue.items()) {
                            std::string subNestedKey = level3.key();
                            nlohmann::json subNestedValue = level3.value();
                            //qDebug() << "Level 3 - Key: " << QString::fromStdString(subNestedKey);
                            //qDebug() << "Level 3 - Value: " << QString::fromStdString(subNestedValue.dump());

                    
                        if (QString::fromStdString(subNestedKey) == "room_name")
                        {
                            QString rooms_ = QString::fromStdString(subNestedValue.dump());
                            rooms_.remove(0, 1);            
                            rooms_.chop(1); 
                            //qDebug() << "room_name: " << rooms_;
                            rooms.append(rooms_);          
                        }
                        
                        else if (QString::fromStdString(subNestedKey) == "room_yaml_file_path")
                        {   
                            QString filePath = QString::fromStdString(subNestedValue.dump());
                            filePath.remove(0, 1);            
                            filePath.chop(1);            
                            //qDebug() << "room_yaml_file_path: " << filePath;
                            room_yaml_file_path_list.push_back(filePath.toStdString());
                        }
                        else if (QString::fromStdString(subNestedKey) == "map_yaml_file_path")
                        {   
                            QString filePath = QString::fromStdString(subNestedValue.dump());
                            filePath.remove(0, 1);            
                            filePath.chop(1);            
                            //qDebug() << "map_yaml_file_path: " << filePath;
                            map_yaml_file_path_list.push_back(filePath.toStdString());
                        }
                        else if (QString::fromStdString(subNestedKey) == "room_id")
                        {   
                            QString room_id = QString::fromStdString(subNestedValue.dump());
                            room_id.remove(0, 1);            
                            room_id.chop(1);            
                            //qDebug() << "room_id: " << room_id;                       
                        } 
                        else if (QString::fromStdString(subNestedKey) == "nav_routes")
                        {   
                            int k = 0; // route order number
                            routes.clear();
                            for (auto& it_routes : subNestedValue.items())
                            {                       
                            std::string routes_name = it_routes.key();
                            nlohmann::json routes_id = it_routes.value();

                            //QString room_id = QString::fromStdString(subNestedValue.dump());
                            //routes_id.remove(0, 1);            
                            //routes_id.chop(1);
                            // std::string string = getElementAtIndex(room_yaml_file_path_list,j);
                            // display_routes_map[i][j][k] = {"s", std::string(routes_id.dump())};
                            routes.append(QString::fromStdString(routes_name)); 

                            //qDebug() << "routes_name: " << QString::fromStdString(routes_name);
                            //qDebug() << "routes_id: " <<  QString::fromStdString(routes_id.dump());
                            k++;
                            }
                            routes_map[i][j] = routes;
                                        
                        } 
                            
                        } // level 3 
                        //int numStringLists = vector_routes.size();
                        //qDebug() << "Numero di QStringList nel vettore: " << numStringLists;
                    j++;       
                    } //level 2
                    //vector_rooms.push_back(rooms);
                    rooms_map[i] = QStringList() << rooms; 
                i++;
                } //level 1
                 
                
                //initialization drop-down menus 
                //Populates hospitals drop-down menu
                hospital_dropdown_->clear();
                hospital_dropdown_->addItems(hospitals);

                room_dropdown_->clear();
                room_dropdown_->addItems(rooms_map[0]);

                route_dropdown_->clear();
                route_dropdown_->addItems(routes_map[0][0]);            

                // Connect the signals to the corresponding slots                
                connect(hospital_dropdown_, QOverload<int>::of(&QComboBox::activated), this, [=](int index)
                {
                    onHospitalChanged(index);
                    //qDebug() << "calling connect...onHospitalChanged";
                });

                connect(room_dropdown_, QOverload<int>::of(&QComboBox::activated), this, [=](int index)
                {
                    onRoomChanged(index);
                    //qDebug() << "calling connect...onRoomChanged";
                });

                connect(route_dropdown_, QOverload<int>::of(&QComboBox::activated), this, [=](int index)
                {
                    onRouteChanged(index);
                    //qDebug() << "calling connect...onRoomChanged";
                });
                
                
                
               //loadMap("/home/user/ros2_ws/src/nav2routes_datamanager/config/site_data/sites/st_james/demo/demo.yaml");
                 
            } 
            else
            {
                qDebug() << "hospitals dictionary not found";
            }

        };

        // send the request to the server and wait the answer
        auto request_dict = std::make_shared<custom_interfaces::srv::HospitalsServiceMessage::Request>();
        auto future = service_client_hospitals_dict->async_send_request(request_dict);
        rclcpp::spin_until_future_complete(client_hospitals_dict, future, std::chrono::seconds(5));
    
        // handle the response
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) 
        {
            response_received(std::move(future));
        } 
        else 
        {
            qDebug() << "Service call timed out";
        }
}

// this function displays the rooms related the selected hospital //
void Nav2routesDataPanel::loadMap(std::string path)
  {           
    auto client_load_map_node = rclcpp::Node::make_shared("nav2_routes_load_map_client");
    auto service_client_load_map = client_load_map_node->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    // Create the request
    auto request_load_map = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    // Set the map URL
    request_load_map->map_url = "/home/user/ros2_ws/src/nav2routes_datamanager/config/site_data/sites/st_james/demo/demo.yaml";

    // Send the request
    
    auto future = service_client_load_map->async_send_request(request_load_map);
     
    // Wait for the response
    auto result = future.wait_for(std::chrono::seconds(15));
    if (result == std::future_status::timeout)
    {
    qDebug() <<"Service call timed out.";
      
    }

    // Process the response
    auto response = future.get();
    if (response->result == nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS)
    {
      qDebug() << "Map loaded successfully.";
      // Process the loaded map data if needed
    }
    else
    {
      //RCLCPP_ERROR(get_logger(), "Failed to load the map. Result code");
    }
  



} // loadMap


std::string Nav2routesDataPanel::getElementAtIndex(const std::list<std::string>& myList, int index) 
{
    int i = 0;
    // iteration
    for (const auto& element : myList) 
    {
        if (i == index) {
            return element;
        }
        i++;
    }
    // index out of the list
    throw std::out_of_range("index out of the list");
}

// this function displays the rooms related the selected hospital //
void Nav2routesDataPanel::onHospitalChanged(int index)
{      
    hindex = hospital_dropdown_->currentIndex(); 
    room_dropdown_->clear();
    room_dropdown_->addItems(rooms_map[hindex]);
    route_dropdown_->clear();
    route_dropdown_->addItems(routes_map[hindex][0]);      
}

// this function displays the rooms related the selected hospital //
void Nav2routesDataPanel::onRoomChanged(int index)
{      
    hindex = hospital_dropdown_->currentIndex();
    rindex = room_dropdown_->currentIndex();
    route_dropdown_->clear();
    route_dropdown_->addItems(routes_map[hindex][rindex]);     
}

void Nav2routesDataPanel::onRouteChanged(int index)
{
hindex = hospital_dropdown_->currentIndex(); 
rindex = room_dropdown_->currentIndex();
roindex = route_dropdown_->currentIndex();
std::list<std::string> list = display_routes_map[hindex][rindex][roindex];
std::string room_file_path = list.front();
std::string route_it = list.back();
}

   
}  // namespace navroutes_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2routes_datapanel::Nav2routesDataPanel, rviz_common::Panel)


 
// json dictionary structure //

// {'st_james': 
// 	{'delivery':
// 		 {'room_id': 'room8_011', 
// 		'room_name': 'The Akara Office8', 
// 		'yaml_file_path': '/home/user/ros2_ws/src/nav2routes_datamanager/config/hospitals/st_james/delivery.yaml', 
// 		'nav_routes': {'Clean bed': [0], 'Clean bins': [1], 'Test': [2]}}, 
	
// 	'chemotherapy':
// 		 {'room_id': 'room7_010', 
// 		'room_name': 'The Akara Office7',
// 		 'yaml_file_path': '/home/user/ros2_ws/src/nav2routes_datamanager/config/hospitals/st_james/chemotherapy.yaml',
// 		 'nav_routes': {'Clean bed': [0], 'Clean bins': [1], 'Test': [2]}},
	
// 	'orthopedic': {'room_id': 'room9_012',
// 		 'room_name': 'The Akara Office9',
// 		 'yaml_file_path': '/home/user/ros2_ws/src/nav2routes_datamanager/config/hospitals/st_james/orthopedic.yaml', 
// 		'nav_routes': {'Clean bed9': [0], 'Clean bins9': [1], 'Test9': [2]}}},

// }


// # URL of map resource# Can be an absolute path to a file: 
// # file:///path/to/maps/floor1.yaml# 
// # Or
// # relative to a ROS package: package://my_ros_package/maps/floor2.yaml
// string map_url
// ---# Result code defintions
// uint8 RESULT_SUCCESS=0
// uint8 RESULT_MAP_DOES_NOT_EXIST=1
// uint8 RESULT_INVALID_MAP_DATA=2
// uint8 RESULT_INVALID_MAP_METADATA=3
// uint8 RESULT_UNDEFINED_FAILURE=255

// # Returned map is only valid if result equals RESULT_SUCCESS
// nav_msgs/OccupancyGrid map
//         std_msgs/Header header
//                 builtin_interfaces/Time stamp
//                         int32 sec
//                         uint32 nanosec
//                 string frame_id
//         MapMetaData info
//                 builtin_interfaces/Time map_load_time
//                         int32 sec
//                         uint32 nanosec
//                 float32 resolution
//                 uint32 width
//                 uint32 height
//                 geometry_msgs/Pose origin
//                         Point position
//                                 float64 x
//                                 float64 y
//                                 float64 z
//                         Quaternion orientation
//                                 float64 x 0
//                                 float64 y 0
//                                 float64 z 0
//                                 float64 w 1
//         int8[] data
// uint8 result 
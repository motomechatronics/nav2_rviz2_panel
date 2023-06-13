#include "navroutes_panel/navroutes_panel.h"
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


namespace navroutes_panel 
{
        // *****************************************//
        // ******** constructor definition *********//
        // *****************************************//

    NavroutesPanel::NavroutesPanel(QWidget *parent) //
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

        auto lineEdit_room = new QLineEdit(); // to be deleted
        auto lineEdit_route = new QLineEdit(); // to be deleted
        
        // button definition
        auto pushbutton = new QPushButton("display route");   
        
        // *****************************************//
        // *********** client definition ***********//
        // ***********  display routes *************//
        // *****************************************//

        pushbutton->connect(pushbutton, &QPushButton::clicked, [=]() {
       
        // clicking the button, the client calls the server //
        auto request_routes = std::make_shared<custom_interfaces::srv::NavroutesServiceMessage::Request>();               
        request_routes->room = set_room;        
        request_routes->route = set_route_id; //lineEdit_route->text().toInt(); //lineEdit_route->text().toInt(&ok);
        auto client_route_display = rclcpp::Node::make_shared("navroutes_visualization_client");
        auto service_client_route_display = client_route_display->create_client<custom_interfaces::srv::NavroutesServiceMessage>(
            "navroutes_visualization_server");
        qDebug() << "fault after...";
        // When the server responds the following function is executed //
        auto response_received = [](rclcpp::Client<custom_interfaces::srv::NavroutesServiceMessage>::SharedFuture future) {
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
            qDebug() << lineEdit_route->text().toInt();
        });

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
    }

    void NavroutesPanel::onInitialize()
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
                //int i = 0;
                // Populates hospitals drop-down menu              
                for (auto it = decoded_data.begin(); it != decoded_data.end(); ++it) {
                qDebug() << QString::fromStdString(it.key());                
                
                // Populates room drop-down menu    
                if (it.value().is_object()) {
                    rooms.clear();
                    for (auto nestedIt = it.value().begin(); nestedIt != it.value().end(); ++nestedIt)
                        {   
                        rooms.append(QString::fromStdString(nestedIt.key()));             
                        }
                    vector_rooms.push_back(rooms);
                 }
                 //i = i + 1;
               
                //hospitals.append(QString::fromStdString(it.key()));             
                }

                 // extracts dictionary data and populates routes drop-down menu    
                //int i = 0;           
                for (auto& level1 : decoded_data.items()) {
                std::string key = level1.key();
                nlohmann::json value = level1.value();

                qDebug() << "Hospital: " << QString::fromStdString(key);
                //qDebug() << "Level 1 - Value: " << QString::fromStdString(value.dump());

                // level 2
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
                        qDebug() << "room_name: " << rooms_;
                    }
                    else if (QString::fromStdString(subNestedKey) == "yaml_file_path")
                    {   
                        QString filePath = QString::fromStdString(subNestedValue.dump());
                        filePath.remove(0, 1);            
                        filePath.chop(1);            
                        qDebug() << "yaml_file_path: " << filePath;
                        yaml_file_path_list.push_back(filePath.toStdString());
                    }
                    else if (QString::fromStdString(subNestedKey) == "room_id")
                    {   
                        QString room_id = QString::fromStdString(subNestedValue.dump());
                        room_id.remove(0, 1);            
                        room_id.chop(1);            
                        qDebug() << "room_id: " << room_id;                       
                    } 
                    else if (QString::fromStdString(subNestedKey) == "nav_routes")
                    {   
                        routes.clear();
                        for (auto& it_routes : subNestedValue.items())
                        {                       
                        std::string routes_name = it_routes.key();
                        nlohmann::json routes_id = it_routes.value();

                        //QString room_id = QString::fromStdString(subNestedValue.dump());
                        //routes_id.remove(0, 1);            
                        //routes_id.chop(1);
                        routes.append(QString::fromStdString(routes_name));                 
                        qDebug() << "routes_name: " << QString::fromStdString(routes_name);
                        qDebug() << "routes_id: " <<  QString::fromStdString(routes_id.dump());
                        }  
                        vector_routes.push_back(routes);                     
                    } 

                        // for(const QString& route : routes)
                        // {
                        // qDebug() << "stamp routes QStringList" << route;
                        // } 


                         
                        } // level 3 
                            int numStringLists = vector_routes.size();

                            qDebug() << "Numero di QStringList nel vettore: " << numStringLists;
                            // for(const QString& routelist : vector_routes[1]) {
                            // qDebug() << "vector routes QStringList" << routelist;
                            // }                
                    } //level 2
                //i = i + 1;
                hospitals.append(QString::fromStdString(key));   
                } //level 1



                // Populates hospitals drop-down menu
                hospital_dropdown_->clear();
                hospital_dropdown_->addItems(hospitals);
                
                // Populates rooms drop-down menu
                QString selected_hospital = hospital_dropdown_->currentText();
                int hindex = hospitals.indexOf(selected_hospital);
                // qDebug() << "hindex: " << hindex;
                room_dropdown_->clear();
                room_dropdown_->addItems(vector_rooms[hindex]);

                // Populates routes drop-down menu
                QString selected_room = room_dropdown_->currentText();
                int rindex = vector_rooms[hindex].indexOf(selected_room);
                // qDebug() << "rindex: " << rindex;
                route_dropdown_->clear();
                route_dropdown_->addItems(vector_routes[rindex]);


                // Connect the signals to the corresponding slots                
                connect(hospital_dropdown_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index)
                {
                    onHospitalChanged(index, vector_rooms);
                    qDebug() << "calling connect...onHospitalChanged";
                });

                connect(room_dropdown_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index)
                {
                    onRoomChanged(index, vector_rooms, vector_routes, hindex, decoded_data);
                    qDebug() << "calling connect...onRoomChanged";
                });

                connect(route_dropdown_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int index)
                {
                    onRouteChanged(index);
                    qDebug() << "calling connect...onRouteChanged";
                });
                
                
                //connect(room_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onRoomChanged(int)));
                                
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


//   // Connect the signals to the corresponding slots
//   connect(hospital_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onHospitalChanged(int)));
//   connect(room_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onRoomChanged(int)));
}



// this function displays the rooms related the selected hospital //
void NavroutesPanel::onHospitalChanged(int index, std::vector<QStringList> vector_rooms)
{
    // Get the selected hospital
    qDebug() << "void NavroutesPanel::onHospitalChanged(int index)";
    selected_hospital = hospital_dropdown_->currentText();
    //qDebug() << QString::fromStdString(selected_hospital.toStdString());
    //int i = hospitals.indexOf(selected_hospital);
    selected_hospital_index = hospitals.indexOf(selected_hospital);
    qDebug() << "onHospitalChanged - hospitals i: " << selected_hospital_index;
    room_dropdown_->clear();
    //room_dropdown_->addItems(vector_rooms[index]);
    room_dropdown_->addItems(vector_rooms[selected_hospital_index]);
    
}

// void NavroutesPanel::onRoomChanged(int index, std::vector<QStringList> vector_rooms, std::vector<QStringList> vector_routes, int hindex)
// {
//   // Get the selected rooms
//   qDebug() << "void NavroutesPanel::onRoomChanged(int index, std::vector<QStringList> vector_rooms...";
//   QString selected_room = room_dropdown_->currentText();    
//   int rindex = vector_rooms[selected_hospital_index].indexOf(selected_room);
//   route_dropdown_->clear();
//   route_dropdown_->addItems(vector_routes[rindex]);
  
// }

void NavroutesPanel::onRoomChanged(int index, std::vector<QStringList> vector_rooms, std::vector<QStringList> vector_routes, int hindex, nlohmann::json decoded_data)
{
    // Get the selected rooms
    if (selected_hospital == "") 
    {
    selected_hospital = "bon_secours";
    }
    qDebug() << "void NavroutesPanel::onRoomChanged(int index, std::vector<QStringList> vector_rooms...";
    QString selected_room = room_dropdown_->currentText();    

    for (auto& level1 : decoded_data.items()) 
    {
        std::string hospital = level1.key();
        if (hospital == selected_hospital.toStdString())
        {
            qDebug() << "hospital matches!..." << QString::fromStdString(hospital) ;
            nlohmann::json value = level1.value();
            for (auto& level2 : value.items()) 
                {
                    std::string room = level2.key();
                    if (room == selected_room.toStdString())
                    {
                    qDebug() << "room matches!... " << QString::fromStdString(room);
                    nlohmann::json nestedValue = level2.value();   
                        for (auto& level3 : nestedValue.items()) 
                        {
                        std::string subNestedKey = level3.key();
                        nlohmann::json subNestedValue = level3.value(); 
                        if (QString::fromStdString(subNestedKey) == "nav_routes")
                            {
                                routes.clear();
                                for (auto& it_routes : subNestedValue.items())
                                {                       
                                std::string routes_name = it_routes.key();                                
                                nlohmann::json routes_id = it_routes.value();

                                QString room_id = QString::fromStdString(subNestedValue.dump());
                                //routes_id.remove(0, 1);            
                                //routes_id.chop(1);
                                routes.append(QString::fromStdString(routes_name));                 
                                //qDebug() << "routes_name: " << QString::fromStdString(routes_name);
                                qDebug() << "routes_id: " <<  QString::fromStdString(routes_id.dump());
                                }  
                                //vector_routes.push_back(routes);  
                            }
                        else if (QString::fromStdString(subNestedKey) == "yaml_file_path")
                            {   
                                QString filePath = QString::fromStdString(subNestedValue.dump());
                                filePath.remove(0, 1);            
                                filePath.chop(1);  
                                set_room = filePath.toStdString();         
                                qDebug() << "yaml_file_path: " << filePath;
                            }
                                  

                        }

                    }
                    else 
                    {
                        qDebug() << "room does not match!";
                        qDebug() << QString::fromStdString(room);
                    }
                    // route_dropdown_->clear();
                    // route_dropdown_->addItems(routes)

                }
             
        
        }  
        else 
        {
            qDebug() << "hospital does not match!";
            qDebug() << QString::fromStdString(hospital);
        } 
    
    }
route_dropdown_->clear();
route_dropdown_->addItems(routes);
}


void NavroutesPanel::onRouteChanged(int index)
{
    if (index >= 0)    
    {
    qDebug() << "void NavroutesPanel::onRouteChanged(int index)";
    route_menu = true; 
    QString selected_route = route_dropdown_->currentText();  
    int selectedIndex = route_dropdown_->currentIndex();  
    set_route_id = selectedIndex;   
    if (set_room.empty())
    {
    set_room = yaml_file_path_list.front();
    set_route_id = selectedIndex;
    qDebug() << "set_room";
    }

    
    //set_room = "/home/user/ros2_ws/src/nav2routes_datamanager/config/hospitals/tallaght_university/immunotherapy.yaml";
    //set_route_id = 0;
    //qDebug() << "data route setup done!";
    
    }
    else 
    {
    route_menu = false;
    }
    
}




}  // namespace navroutes_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navroutes_panel::NavroutesPanel, rviz_common::Panel)


 
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

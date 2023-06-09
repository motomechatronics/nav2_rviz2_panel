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
   
        // // *****************************************//
        // // *********** client definition ***********//
        // // *********  hospitals dictionary *********//
        // // *****************************************//
        // // starting from keys and values of a flatten dictionary constructs the hospitals original dictionary
        // // The hospitals_dict to make work drop-down menu

        // auto client_hospitals_dict = rclcpp::Node::make_shared("navroutes_datareader_client");
        // auto service_client_hospitals_dict = client_hospitals_dict->create_client<custom_interfaces::srv::HospitalsServiceMessage>(
        //     "hospitals_datareader_service");

        // auto response_received = [](rclcpp::Client<custom_interfaces::srv::HospitalsServiceMessage>::SharedFuture future) {
        //     auto response = future.get();
        //     if (response->success) {

        //     std::string encoded_data_string = response->encoded_dict;
        //     nlohmann::json decoded_data = nlohmann::json::parse(encoded_data_string);
        //     //decoded_data = nlohmann::json::parse(encoded_data_string);
        //     // debug decoded_data 
        //     std::cout << "Decoded Data: " << decoded_data.dump() << std::endl;
        //     QString str = "hello";
        //     // qDebug() << "Hospitals:";
        //     // for (auto it = decoded_data.begin(); it != decoded_data.end(); ++it) {
        //     // qDebug() << QString::fromStdString(it.key());
        //     // hospitals << QString::fromStdString(it.key());
        //     // }
        //     //std::vector<std::string> keys = response->key;
        //     //std::string values = response->value;

        //     // Debug keys/values display
        //     //auto flag = true; // false;
        //     // -------------------            
        //     // if (flag) {
        //     // for (const auto& key : keys) {
        //     // qDebug() << QString::fromStdString(key);
        //     // }                                    
        //     // qDebug() << QString::fromStdString(values); 
        //     // }             

        //     //std::map<std::string,int> hospitals_dict = response->hospitals_dict
                
        //     qDebug() << "hospitals dictionary successfully recived";
        //     } else {
        //         qDebug() << "hospitals dictionary not found";
        //     }
        // };

        // // send the request to the server and wait the answer
        // auto request_dict = std::make_shared<custom_interfaces::srv::HospitalsServiceMessage::Request>();
        // auto future = service_client_hospitals_dict->async_send_request(request_dict);
        // rclcpp::spin_until_future_complete(client_hospitals_dict, future, std::chrono::seconds(5));

        // // handle the response
        // if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        //     response_received(std::move(future));
        // } else {
        //     qDebug() << "Service call timed out";
        // }

        // *****************************************//
        // *********** client definition ***********//
        // ***********  display routes *************//
        // *****************************************//

        pushbutton->connect(pushbutton, &QPushButton::clicked, [=]() {
        //auto lineEdit_route = new QLineEdit();
        //route = 
        //bool ok;
        auto request = std::make_shared<custom_interfaces::srv::NavroutesServiceMessage::Request>();
        
        request_routes->room = "office";
        request_routes->route = lineEdit_route->text().toInt(); //lineEdit_route->text().toInt(&ok);
        auto client_route_display = rclcpp::Node::make_shared("navroutes_visualization_client");
        auto service_client_route_display = client_route_display->create_client<custom_interfaces::srv::NavroutesServiceMessage>(
            "navroutes_visualization_server");
        //routeValue = lineEdit_route->text();
        // Azione da eseguire quando il bottone viene premuto
        // Puoi inserire qui il codice che desideri eseguire al click del bottone
        // Ad esempio, puoi stampare un messaggio sulla console

        // Funzione da eseguire quando la risposta del servizio viene ricevuta
        auto response_received = [](rclcpp::Client<custom_interfaces::srv::NavroutesServiceMessage>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                qDebug() << "Route successfully displayed";
            } else {
                qDebug() << "Route not found";
            }
        };

        // Invio della richiesta al server e attesa della risposta
        auto future = service_client_route_display->async_send_request(request);
        rclcpp::spin_until_future_complete(client_route_display, future, std::chrono::seconds(5));

        // Gestione della risposta ricevuta
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            response_received(std::move(future));
        } else {
            qDebug() << "Service call timed out";
        }

  
            qDebug() << "Hai premuto il bottone!";
            qDebug() << lineEdit_route->text().toInt();
        });

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

                // populates hospitals drop-down menu                
                //QStringList hospitals;                
                for (auto it = decoded_data.begin(); it != decoded_data.end(); ++it) {
                //qDebug() << QString::fromStdString(it.key());
                hospitals << QString::fromStdString(it.key());
                }
                
                hospital_dropdown_->clear();
                hospital_dropdown_->addItems(hospitals);

                // Popolare le rooms in base all'ospedale selezionato
                QString selected_hospital = hospital_dropdown_->currentText();
                //QStringList rooms;
                if (selected_hospital == "bon_secours") {
                    rooms << "Room 1" << "Room 2" << "Room 3";
                } else if (selected_hospital == "coombe") {
                    rooms << "Room A" << "Room B" << "Room C";
                } else if (selected_hospital == "t_james") {
                    rooms << "Room X" << "Room Y" << "Room Z";
                }
                room_dropdown_->clear();
                room_dropdown_->addItems(rooms);

                // Popolare le routes in base alla room selezionata
                QString selected_room = room_dropdown_->currentText();
                QStringList routes;
                if (selected_room == "Room 1") {
                    routes << "Route 1" << "Route 2" << "Route 3";
                } else if (selected_room == "Room 2") {
                    routes << "Route A" << "Route B" << "Route C";
                } else if (selected_room == "Room 3") {
                    routes << "Route X" << "Route Y" << "Route Z";
                }
                route_dropdown_->clear();
                route_dropdown_->addItems(routes);

                // Connect the signals to the corresponding slots
                connect(hospital_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onHospitalChanged(int)));
                connect(room_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onRoomChanged(int)));
                                
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



//   // populates hospitals drop-down menu
  
//     //std::cout << "Decoded Data: " << decoded_data.dump() << std::endl;
//     //QString s = "hello";
//     QStringList hospitals;
//     qDebug() << "Hospitals:";
//     for (auto it = decoded_data.begin(); it != decoded_data.end(); ++it) {
//     qDebug() << QString::fromStdString(it.key());
//     hospitals << QString::fromStdString(it.key());
//     }
//     hospitals << str << "2" << "3";
//     hospital_dropdown_->clear();
//     hospital_dropdown_->addItems(hospitals);




//   // Popolare le rooms in base all'ospedale selezionato
//   QString selected_hospital = hospital_dropdown_->currentText();
//   QStringList rooms;
//   if (selected_hospital == "bon_secours") {
//     rooms << "Room 1" << "Room 2" << "Room 3";
//   } else if (selected_hospital == "coombe") {
//     rooms << "Room A" << "Room B" << "Room C";
//   } else if (selected_hospital == "t_james") {
//     rooms << "Room X" << "Room Y" << "Room Z";
//   }
//   room_dropdown_->clear();
//   room_dropdown_->addItems(rooms);

//   // Popolare le routes in base alla room selezionata
//   QString selected_room = room_dropdown_->currentText();
//   QStringList routes;
//   if (selected_room == "Room 1") {
//     routes << "Route 1" << "Route 2" << "Route 3";
//   } else if (selected_room == "Room 2") {
//     routes << "Route A" << "Route B" << "Route C";
//   } else if (selected_room == "Room 3") {
//     routes << "Route X" << "Route Y" << "Route Z";
//   }
//   route_dropdown_->clear();
//   route_dropdown_->addItems(routes);

//   // Connect the signals to the corresponding slots
//   connect(hospital_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onHospitalChanged(int)));
//   connect(room_dropdown_, SIGNAL(currentIndexChanged(int)), this, SLOT(onRoomChanged(int)));
}

void NavroutesPanel::onHospitalChanged(int index)
{
    // Get the selected hospital
    QString selected_hospital = hospital_dropdown_->currentText();

    // Find the corresponding entry in the decoded_data JSON object
    auto it = decoded_data.find(selected_hospital.toStdString());
    if (it != decoded_data.end() && it->is_object())
    {
        // Get the rooms from the selected hospital's entry
        const nlohmann::json& rooms_data = *it;
        QStringList rooms;
        for (const auto& room_entry : rooms_data.items())
        {
            rooms << QString::fromStdString(room_entry.key());
        }

        // Clear and populate the rooms drop-down menu
        room_dropdown_->clear();
        room_dropdown_->addItems(rooms);
    }
}

void NavroutesPanel::onRoomChanged(int index)
{
  // Aggiornare le routes in base alla room selezionata
  QString selected_room = room_dropdown_->currentText();
  QStringList routes;
  if (selected_room == "Room 1") {
    routes << "Route 1" << "Route 2" << "Route 3";
  } else if (selected_room == "Room 2") {
    routes << "Route A" << "Route B" << "Route C";
  } else if (selected_room == "Room 3") {
    routes << "Route X" << "Route Y" << "Route Z";
  }
  route_dropdown_->clear();
  route_dropdown_->addItems(routes);
}




}  // namespace navroutes_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navroutes_panel::NavroutesPanel, rviz_common::Panel)


  



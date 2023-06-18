#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include <nav2_msgs/srv/load_map.hpp>
#include "custom_interfaces/srv/navroutes_service_message.hpp"
#include "custom_interfaces/srv/hospitals_service_message.hpp"
#include <QMainWindow>
#include <QDebug>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QInputDialog>
#include <QComboBox>
#include <QStringList>
#include <QVBoxLayout>
#include <unordered_map>
#include <map>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <list>

// using json = nlohmann::json;

namespace nav2routes_datapanel
    {

        class Nav2routesDataPanel : public rviz_common::Panel // QMainWindow
        {
            Q_OBJECT

            public:
            explicit Nav2routesDataPanel(QWidget *parent = nullptr);
            nlohmann::json decoded_data;
           
            private Q_SLOTS:
            //void onLineEditRouteEditingFinished();

            protected Q_SLOTS:           
            void onHospitalChanged(int index);
            void onRoomChanged(int index);
            void onRouteChanged(int index);

            void onInitialize() override;
            
            private:  
            int hindex;
            int rindex;
            int roindex;

            std::list<std::string> room_yaml_file_path_list;
            std::list<std::string> map_yaml_file_path_list;
            std::string set_room;
            int set_route_id = 0;          
            std::vector<QStringList> vector_rooms; 
            std::vector<QStringList> vector_routes;

            QStringList hospitals; 
            QStringList rooms; 
            QStringList routes;            
            std::unordered_map<int, QStringList> rooms_map;
            std::unordered_map<int, std::unordered_map<int, QStringList>> routes_map;
            std::unordered_map<int, std::unordered_map<int, std::unordered_map<int, std::list<std::string>>>> display_routes_map;
           

            QString selected_hospital;
            QString selected_room;
            QString selected_route;
              
            QVBoxLayout* layout_;
            QLabel* title_label_;
            QLabel* room_label_;
            QLabel* route_label_;
            QLineEdit* line_edit_room_;
            QLineEdit* line_edit_route_;
            QComboBox* hospital_dropdown_;
            QComboBox* room_dropdown_;
            QComboBox* route_dropdown_;
            QPushButton* push_button_;
            
            private Q_SLOTS:
            void onPushButtonClicked();

            private:
            std::string room_yaml_file_path;
            std::string map_yaml_file_path;
            int room_id;
       
            //nlohmann::json decoded_data;

            std::string encoded_data_string;
            void initializeComboBoxes();
            std::string getElementAtIndex(const std::list<std::string>& myList, int index);
            void loadMap(std::string path);
                        
            //std::shared_ptr<custom_interfaces::srv::NavroutesServiceMessage::Request> request_routes;
            custom_interfaces::srv::NavroutesServiceMessage::Request::SharedPtr request_routes;
            custom_interfaces::srv::HospitalsServiceMessage::Request::SharedPtr request_dict;            
            //std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request_load_map;
            nav2_msgs::srv::LoadMap::Request::SharedPtr request_load_map;

            rclcpp::Node::SharedPtr client_load_map_node;
            rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr service_client_load_map;  

            rclcpp::Node::SharedPtr client_route_display;
            rclcpp::Client<custom_interfaces::srv::NavroutesServiceMessage>::SharedPtr service_client_route_display;
            
            rclcpp::Node::SharedPtr client_hospitals_dict;
            rclcpp::Client<custom_interfaces::srv::HospitalsServiceMessage>::SharedPtr service_client_hospitals_dict;            
           
        };

    } // namespace navorutes_pxxsssss
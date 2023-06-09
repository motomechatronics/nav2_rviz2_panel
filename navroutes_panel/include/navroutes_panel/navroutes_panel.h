#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
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
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

// using json = nlohmann::json;

namespace navroutes_panel
        {

                class NavroutesPanel : public rviz_common::Panel // QMainWindow
        {
            Q_OBJECT

            public:
            explicit NavroutesPanel(QWidget *parent = nullptr);
            //lohmann::json decoded_data;
            
            private Q_SLOTS:
            void onLineEditRouteEditingFinished();

            protected Q_SLOTS:
            void onHospitalChanged(int index);
            void onRoomChanged(int index);

            void onInitialize() override;
            //void onInitialize(nlohmann::json& decoded_data);

            private:

            QStringList hospitals; 
            QStringList rooms;           
            QString roomValue;
            QString routeValue;
            QLineEdit* lineEdit_room;
            QLineEdit* lineEdit_route;
              
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
            
            nlohmann::json decoded_data;

            std::string encoded_data_string;
            void initializeComboBoxes();
            void populateRoomsComboBox(const QString& selectedHospital);
            void populateRoutesComboBox(const QString& selectedRoom);

            rclcpp::Client<custom_interfaces::srv::NavroutesServiceMessage>::SharedPtr service_client_route_display;
            custom_interfaces::srv::NavroutesServiceMessage::Request::SharedPtr request_routes;
            rclcpp::Node::SharedPtr client_route_display;

            rclcpp::Client<custom_interfaces::srv::HospitalsServiceMessage>::SharedPtr service_client_hospitals_dict;
            custom_interfaces::srv::HospitalsServiceMessage::Request::SharedPtr request_dict;
            rclcpp::Node::SharedPtr client_hospitals_dict;

        };

        } // namespace navorutes_panel



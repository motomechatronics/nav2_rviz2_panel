#pragma once
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "custom_interfaces/srv/navroutes_service_message.hpp"
#include <QMainWindow>
#include <QDebug>
#include <QPushButton>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QInputDialog>


namespace demo_panel
        {

                class DemoPanel : public rviz_common::Panel // QMainWindow
        {
            Q_OBJECT

            public:
            explicit DemoPanel(QWidget *parent = nullptr);

            private Q_SLOTS:
            void onLineEditRouteEditingFinished();

            void onInitialize() override;

            private:
            
            QString roomValue;
            QString routeValue;
            QLineEdit* lineEdit_room;
            QLineEdit* lineEdit_route;

            rclcpp::Client<custom_interfaces::srv::NavroutesServiceMessage>::SharedPtr service_client;
            custom_interfaces::srv::NavroutesServiceMessage::Request::SharedPtr request;
            rclcpp::Node::SharedPtr client;
        };

        } // namespace demo_panel

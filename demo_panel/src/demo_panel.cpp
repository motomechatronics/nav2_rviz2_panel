#include "demo_panel/demo_panel.h"
#include "rviz_common/properties/property_tree_widget.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/visualization_manager.hpp"
#include "custom_interfaces/srv/navroutes_service_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QInputDialog>

namespace demo_panel //
{

    DemoPanel::DemoPanel(QWidget *parent) //
            : rviz_common::Panel(parent)
    {
        auto layout = new QVBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);

        auto title = new QLabel("Akara navroutes panel");
        auto room = new QLabel("room");
        auto route = new QLabel("route");
        auto lineEdit_room = new QLineEdit();
        auto lineEdit_route = new QLineEdit();
        //auto radio = new QRadioButton();
        auto pushbutton = new QPushButton("send");
        //routeValue = new QString();

        lineEdit_room->setPlaceholderText("insert room...");

        lineEdit_route->setPlaceholderText("insert route...");

        // QString routeValue = lineEdit_route->text();
        //qDebug() << lineEdit_route->text();

        // Connetti il segnale di editingFinished per acquisire il testo quando l'utente ha finito di modificare
        //connect(lineEdit_route, &QLineEdit::editingFinished, this, &DemoPanel::onLineEditRouteEditingFinished);
        
        pushbutton->connect(pushbutton, &QPushButton::clicked, [=]() {
        //auto lineEdit_route = new QLineEdit();
        //route = 
        //bool ok;
        auto request = std::make_shared<custom_interfaces::srv::NavroutesServiceMessage::Request>();
        
        request->room = "office";
        request->route = lineEdit_route->text().toInt(); //lineEdit_route->text().toInt(&ok);
        auto client = rclcpp::Node::make_shared("navroutes_visualization_client");
        auto service_client = client->create_client<custom_interfaces::srv::NavroutesServiceMessage>(
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
        auto future = service_client->async_send_request(request);
        rclcpp::spin_until_future_complete(client, future, std::chrono::seconds(5));

        // Gestione della risposta ricevuta
        if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            response_received(std::move(future));
        } else {
            qDebug() << "Service call timed out";
        }

  
            qDebug() << "Hai premuto il bottone!";
            qDebug() << lineEdit_route->text().toInt();
        });

        layout->addWidget(title);
        layout->addWidget(room);
        layout->addWidget(lineEdit_room);
        layout->addWidget(route);        
        layout->addWidget(lineEdit_route);
        layout->addWidget(pushbutton);
        

        setLayout(layout);
    }


    void DemoPanel::onInitialize()
    {
      //tree_widget_->setModel(getDisplayContext()->getSelectionManager()->getPropertyModel());
    }

}  // namespace demo_panel


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(demo_panel::DemoPanel, rviz_common::Panel)

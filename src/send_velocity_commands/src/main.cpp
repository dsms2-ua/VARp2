#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RobotDriver : public rclcpp::Node
{
private:
  // Publisher con el tipo de mensaje específico
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

public:
  RobotDriver() : Node("send_velocity_commands")
  {
    // Configuramos el publisher para el tópico "/cmd_vel"
    // El '10' es el tamaño de la cola (history depth)
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

  void driveKeyboard()
  {
    std::cout << "Type a command and then press enter. "
                 "Use '+' to move forward, 'l' to turn left, "
                 "'r' to turn right, '.' to exit.\n";

    geometry_msgs::msg::Twist base_cmd;
    char cmd[50];

    // rclcpp::ok() para verificar el estado del contexto
    while (rclcpp::ok()) {
      std::cin.getline(cmd, 50);

      if (cmd[0] != '+' && cmd[0] != 'l' && cmd[0] != 'r' && cmd[0] != '.') {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      // Reiniciamos valores
      base_cmd.linear.x = 0.0;
      base_cmd.linear.y = 0.0;
      base_cmd.angular.z = 0.0;

      if (cmd[0] == '+') {
        base_cmd.linear.x = 0.25;
      }
      else if (cmd[0] == 'l') {
        base_cmd.angular.z = 0.75;
        base_cmd.linear.x = 0.25;
      }
      else if (cmd[0] == 'r') {
        base_cmd.angular.z = -0.75;
        base_cmd.linear.x = 0.25;
      }
      else if (cmd[0] == '.') {
        break;
      }

      // Publicamos el mensaje
      cmd_vel_pub_->publish(base_cmd);
      
      // Procesar callbacks si los hubiera
      rclcpp::spin_some(this->get_node_base_interface());
    }
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  // Creamos el puntero al nodo
  auto driver = std::make_shared<RobotDriver>();
  
  // Ejecutamos la lógica de teclado
  driver->driveKeyboard();
  
  rclcpp::shutdown();
  return 0;
}
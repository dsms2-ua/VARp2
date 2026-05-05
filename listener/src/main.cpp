#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class ImageListener : public rclcpp::Node {
public:
    ImageListener() : Node("listener") {
        // Definimos un perfil compatible con sensores (Best Effort)
        auto qos_profile = rclcpp::SensorDataQoS();

        sub_ = image_transport::create_subscription(this, 
        "/camera/image_raw", 
        std::bind(&ImageListener::imageCallback, this, std::placeholders::_1), 
        "raw", 
        qos_profile.get_rmw_qos_profile()); // Usamos el perfil de sensor

        // Ventana redimensionable (evita que se cree más grande que la pantalla)
        cv::namedWindow("view", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::resizeWindow("view", 960, 540);
    }

    ~ImageListener() {
        cv::destroyWindow("view");
    }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv::Mat originalImage;
      cv::Mat displayImage;

      // cv_bridge funciona muy similar, pero con SharedPtr de ROS 2
      originalImage = cv_bridge::toCvShare(msg, "bgr8")->image;

      // Escalar solo si la imagen es demasiado grande (manteniendo proporción)
      const int maxWidth = 960;
      const int maxHeight = 540;
      const int width = originalImage.cols;
      const int height = originalImage.rows;

      if (width > 0 && height > 0 && (width > maxWidth || height > maxHeight)) {
        const double scaleW = static_cast<double>(maxWidth) / static_cast<double>(width);
        const double scaleH = static_cast<double>(maxHeight) / static_cast<double>(height);
        const double scale = std::min(scaleW, scaleH);
        cv::resize(originalImage, displayImage,
                   cv::Size(static_cast<int>(width * scale), static_cast<int>(height * scale)));
      } else {
        displayImage = originalImage;
      }

      cv::imshow("view", displayImage);
      cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
      // Sustituimos ROS_ERROR por RCLCPP_ERROR
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  image_transport::Subscriber sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Creamos el nodo y lo mantenemos vivo con spin
  rclcpp::spin(std::make_shared<ImageListener>());
  rclcpp::shutdown();
  return 0;
}
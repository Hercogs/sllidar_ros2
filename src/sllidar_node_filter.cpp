
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <memory>

#define RAD2DEG(x) ((x)*180./M_PI)

class SLlidarFilterNode : public rclcpp::Node{
  public:
    SLlidarFilterNode() : Node("sllidar_filter_node")
    {
      scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("lidar_scan", rclcpp::QoS(rclcpp::KeepLast(10)));
      scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                        "lidar_scan_raw", rclcpp::SensorDataQoS(),
                        std::bind(&SLlidarFilterNode::scanCb, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "sllidar_filter_node is started!");
    }

  private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

    void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan) {

      sensor_msgs::msg::LaserScan modified_scan = *scan;

      // For right hand coordinate system
      modified_scan.angle_min += modified_scan.angle_min;
      modified_scan.angle_max += modified_scan.angle_min;

      int resolution = scan->ranges.size();
      int resolution_quater = static_cast<int> (resolution / 4);

      // Use -pi/2 to pi/2
      modified_scan.ranges.clear(); // Clear all elements
      modified_scan.ranges.insert(
                    modified_scan.ranges.end(),
                    scan->ranges.end() - resolution_quater,
                    scan->ranges.end()
      );
      modified_scan.ranges.insert(
                    modified_scan.ranges.end(),
                    scan->ranges.begin(),
                    scan->ranges.begin() + resolution_quater
      );

      modified_scan.angle_max -= resolution_quater * scan->angle_increment;
      modified_scan.angle_min -= resolution_quater * scan->angle_increment;

      scan_pub->publish(modified_scan);
    }

};


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<SLlidarFilterNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();


  return 0;
}

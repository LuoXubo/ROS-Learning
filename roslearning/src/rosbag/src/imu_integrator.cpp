// 接受IMU和SLAM，轨迹递推，发布/calc/odom

#include "rosbag/imu_integrator.h"

int cnt = 0;
ImuIntegrator::ImuIntegrator(const ros::Publisher &pub)
{
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  velocity = zero;
  line_pub = pub;
  firstT = true;

  calc_pub = pub;

  // Line strip is blue
  path.color.r = 1.0;
  path.color.a = 1.0;
  path.type = visualization_msgs::Marker::LINE_STRIP;
  path.header.frame_id = "odom";
  path.ns = "points_and_lines";
  path.action = visualization_msgs::Marker::ADD;
  path.pose.orientation.w = 1.0;
  path.scale.x = 0.02;
  // geometry_msgs::Point p;
  // p.x = 0;
  // p.y = 0;
  // p.z = 0;
  // path.points.push_back(p);
}

ImuIntegrator::ImuIntegrator(const ros::Publisher &pub, const ros::Publisher &odom_line)
{
  Eigen::Vector3d zero(0, 0, 0);
  pose.pos = zero;
  pose.orien = Eigen::Matrix3d::Identity();
  velocity = zero;
  line_pub = pub;
  odom_pub = odom_line;
  firstT = true;

  // Line strip is blue
  path.color.r = 1.0;
  path.color.a = 1.0;
  path.type = visualization_msgs::Marker::LINE_STRIP;
  path.header.frame_id = "map";
  path.ns = "points_and_lines";
  path.action = visualization_msgs::Marker::ADD;
  path.pose.orientation.w = 1.0;
  path.scale.x = 0.02;

  // Line strip is blue
  gt_path.color.b = 1.0;
  gt_path.color.a = 1.0;
  gt_path.type = visualization_msgs::Marker::LINE_STRIP;
  gt_path.header.frame_id = "map";
  gt_path.ns = "points_and_lines";
  gt_path.action = visualization_msgs::Marker::ADD;
  gt_path.pose.orientation.w = 1.0;
  gt_path.scale.x = 0.02;

  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  // path.points.push_back(p);
}

void ImuIntegrator::ImuCallback(const sensor_msgs::Imu &msg)
{
  std::cout << msg << '\n';
  if (firstT)
  {
    time = msg.header.stamp;
    deltaT = 0;
    setGravity(msg.linear_acceleration);
    firstT = false;
  }
  else
  {
    if(time > msg.header.stamp)
    {
      std::cout << "imu passed ...\n";
      return;
    }
      
    deltaT = (msg.header.stamp - time).toSec();
    time = msg.header.stamp;
    calcOrientation(msg.angular_velocity);
    // calcOrientation(msg.orientation);
    calcPosition(msg.linear_acceleration);
    // updatePath(pose.pos);
    updateOdom(time, pose.pos, pose.orien);
    // publishMessage();
  }
}

void ImuIntegrator::OdomCallback(const nav_msgs::Odometry &msg)
{
  // std::cout << msg << '\n';
  if (firstT)
  {
    firstT = false;
    time = msg.header.stamp;
  }
  if(time > msg.header.stamp)
  {
    std::cout << "odom passed ...\n";
    return;
  }
    
  time = msg.header.stamp;
  deltaT = (msg.header.stamp - time).toSec();
  pose.pos(0) = msg.pose.pose.position.x;
  pose.pos(1) = msg.pose.pose.position.y;
  pose.pos(2) = msg.pose.pose.position.z;

  // Eigen::Quaterniond quat;
  // quat.x() = msg.pose.pose.orientation.x;
  // quat.y() = msg.pose.pose.orientation.y;
  // quat.z() = msg.pose.pose.orientation.z;
  // quat.w() = msg.pose.pose.orientation.w;
  // pose.orien = quat.normalized().toRotationMatrix();

  updatePath(pose.pos);
  updateGTPath(pose.pos);
  
  // publishMessage();
  // odom_pub.publish(gt_path);

  calc_pub.publish(msg);
}

void ImuIntegrator::setGravity(const geometry_msgs::Vector3 &msg)
{
  gravity[0] = msg.x;
  gravity[1] = msg.y;
  gravity[2] = msg.z;
}

void ImuIntegrator::updatePath(const Eigen::Vector3d &msg)
{
  geometry_msgs::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];

  path.points.push_back(p);
  // std::cout << p.x << ' ' << p.y << ' ' << p.z << std::endl;
}

void ImuIntegrator::updateGTPath(const Eigen::Vector3d &msg)
{
  geometry_msgs::Point p;
  p.x = msg[0];
  p.y = msg[1];
  p.z = msg[2];
  gt_path.points.push_back(p);
}

void ImuIntegrator::updateOdom(const ros::Time time, const Eigen::Vector3d &pos, const Eigen::Matrix3d &orien)
{
  nav_msgs::Odometry odom;
  odom.header.stamp = time;

  odom.pose.pose.position.x = pos(0);
  odom.pose.pose.position.y = pos(1);
  odom.pose.pose.position.z = pos(2);

  odom.pose.pose.orientation.x = (orien(2, 1) - orien(1, 2)) / 4;
  odom.pose.pose.orientation.y = (orien(0, 2) - orien(2, 0)) / 4;
  odom.pose.pose.orientation.z = (orien(1, 0) - orien(0, 1)) / 4;
  odom.pose.pose.orientation.w = std::sqrt(1 + orien(0, 0) + orien(1, 1) + orien(2, 2)) / 2;

  calc_pub.publish(odom);
}

void ImuIntegrator::publishMessage()
{
  line_pub.publish(path);
}

// 更新：直接使用IMU返回的方向, q(x,y,z,w)
void ImuIntegrator::calcOrientation(const geometry_msgs::Vector3 &msg)
{
  Eigen::Matrix3d B;
  B << 0, -msg.z * deltaT, msg.y * deltaT, msg.z * deltaT, 0, -msg.x * deltaT,
      -msg.y * deltaT, msg.x * deltaT, 0;
  double sigma =
      std::sqrt(std::pow(msg.x, 2) + std::pow(msg.y, 2) + std::pow(msg.z, 2)) *
      deltaT;
  pose.orien = pose.orien *
               (Eigen::Matrix3d::Identity() + (std::sin(sigma) / sigma) * B -
                ((1 - std::cos(sigma)) / std::pow(sigma, 2)) * B * B);

  // std::cout << pose.orien << "\n\n";
}

void ImuIntegrator::calcOrientation(const geometry_msgs::Quaternion &msg)
{
  // std::cout << msg << '\n';
  Eigen::Quaterniond quat;
  quat.x() = msg.x;
  quat.y() = msg.y;
  quat.z() = msg.z;
  quat.w() = msg.w;
  pose.orien = quat.normalized().toRotationMatrix();
}

void ImuIntegrator::calcPosition(const geometry_msgs::Vector3 &msg)
{
  // std::cout << msg << "\n\n";
  Eigen::Vector3d acc_l(msg.x, msg.y, msg.z);
  // Eigen::Vector3d acc_g = acc_l;
  Eigen::Vector3d acc_g = pose.orien * acc_l;

  auto acc = acc_g - gravity;
  // std::cout << acc << "\n\n";
  // if(std::abs(acc_g(0))>10) velocity(0) = velocity(0)+ deltaT*acc(0);
  // if(std::abs(acc_g(1))>10) velocity(1) = velocity(1)+ deltaT*acc(1);
  // if(std::abs(acc_g(2))>10) velocity(2) = velocity(2)+ deltaT*acc(2);

  velocity = velocity + deltaT * (acc_g - gravity);
  std::cout << velocity << "\n\n";
  pose.pos = pose.pos + deltaT * velocity;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "Imu_Integrator_node");
  ros::NodeHandle nh;

  ros::Publisher calc_pub = nh.advertise<nav_msgs::Odometry>("/calc/odom", 1000);

  // ImuIntegrator *imu_integrator = new ImuIntegrator(line, odom_line);
  ImuIntegrator *imu_integrator = new ImuIntegrator(calc_pub);

  std::cout << "Begin to listen ...\n";
  ros::Subscriber Imu_message = nh.subscribe(
      "/lunar/imu", 2000, &ImuIntegrator::ImuCallback, imu_integrator);

  ros::Subscriber Odom_message = nh.subscribe(
      "/lunar/odom", 1000, &ImuIntegrator::OdomCallback, imu_integrator);

  ros::spin();
}

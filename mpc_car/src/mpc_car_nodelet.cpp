#include <car_msgs/CarCmd.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <Eigen/Geometry>
#include <mpc_car/mpc_car.hpp>

namespace mpc_car {
class Nodelet : public nodelet::Nodelet {
 private:
  std::shared_ptr<MpcCar> mpcPtr_;
  ros::Timer plan_timer_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_pub_;
  VectorX state_;
  bool init = false;
  double delay_ = 0.0;

  void plan_timer_callback(const ros::TimerEvent& event) {
    if (init) {
      ROS_WARN("plan_time");
      ros::Time t1 = ros::Time::now();

      auto ret = mpcPtr_->solveQP(state_);
      assert(ret == 1);
      ros::Time t2 = ros::Time::now();
      double solve_time = (t2 - t1).toSec();
      std::cout << "solve qp costs: " << 1e3 * solve_time << "ms" << std::endl;
      // TODO
      car_msgs::CarCmd msg;
      msg.header.frame_id = "world";
      msg.header.stamp = ros::Time::now();

      VectorX x;
      VectorU u;
      mpcPtr_->getPredictXU(0, x, u);
      std::cout << "u: " << u.transpose() << std::endl;
      std::cout << "x: " << x.transpose() << std::endl;
      std::cout << std::endl;

      msg.a = u(0);
      msg.varepsilon = u(1);
      cmd_pub_.publish(msg);
      //输出msg
      std::cout << "msg.a =  " << msg.a << std::endl;
      std::cout << "msg.varepsilon =  " << msg.varepsilon << std::endl;
      mpcPtr_->visualization();
    }
    return;
  }
  void odom_call_back(const nav_msgs::Odometry::ConstPtr& msg) {
    //ROS_WARN("odom_call_back");
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                         msg->pose.pose.orientation.x,
                         msg->pose.pose.orientation.y,
                         msg->pose.pose.orientation.z);
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    // bug#002
    state_ << x, y, euler.z(), v.norm(), msg->twist.twist.linear.z;
    //这里的state(3),state(4)分别需要是机器人的速度和转向角速度，v.norm()为人的速度，那么当前的里程计信息如何转化为机器人转向角速度
    
    
    init = true;
  }

 public:
  void onInit(void) {
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    ROS_WARN("1");
    mpcPtr_ = std::make_shared<MpcCar>(nh);
    double dt = 0;
    nh.getParam("dt", dt);
    nh.getParam("delay", delay_);

    plan_timer_ = nh.createTimer(ros::Duration(dt), &Nodelet::plan_timer_callback, this);

    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &Nodelet::odom_call_back, this);

    cmd_pub_ = nh.advertise<car_msgs::CarCmd>("car_cmd", 1);
    ROS_WARN("2");
  }
};
}  // namespace mpc_car

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mpc_car::Nodelet, nodelet::Nodelet);
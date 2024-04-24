#include <car_msgs/CarCmd.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <deque>

typedef Eigen::Matrix<double, 5, 1> Vector5d;

struct Car {
  double L;
  Vector5d state;
  // state: x, y, phi, v ,delta
  // input: a, varepsilon
  inline void setInitialState(const Vector5d& s) {
    state = s;
  }
  inline Vector5d diff(const Vector5d& s,
                       const Eigen::Vector2d& input) const {
    Vector5d ds;
    double phi = s(2);
    double v = s(3);
    double delta = s(4);
    double a = input(0);
    double varepsilon = input(1);
    ds(0) = v * cos(phi) * cos(delta);
    ds(1) = v * sin(phi) * cos(delta);
    ds(2) = v / L * sin(delta);
    ds(3) = a;
    ds(4) = varepsilon;

   //std::cout << "ds: " << ds.transpose() << std::endl;
    return ds;
  }

// 非線性迭代法
  void step(const Eigen::Vector2d& input, const double dt) {
    // Runge–Kutta
    Vector5d k1 = diff(state, input);
    Vector5d k2 = diff(state + k1 * dt / 2, input);
    Vector5d k3 = diff(state + k1 * dt / 2, input);
    Vector5d k4 = diff(state + k3 * dt, input);
    state = state + (k1 + k2 * 2 + k3 * 2 + k4) * dt / 6;
    //std::cout << "step_state: " << state.transpose() << std::endl;
  }
};

namespace car_simulator {
class Nodelet : public nodelet::Nodelet {
 private:
  Car car;
  double delay_ = 0.0;
  Eigen::Vector2d input_;
  ros::Publisher odom_pub_;
  ros::Subscriber cmd_sub_;
  ros::Timer sim_timer_;

  struct DelayedMsg {
    ros::Time t;
    double a, varepsilon;
    DelayedMsg() {}
    DelayedMsg(const ros::Time& _t, double _a, double _varepsilon) : t(_t), a(_a), varepsilon(_varepsilon) {}
  };
  std::deque<DelayedMsg> delayedMsgs_;

  void cmd_callback(const car_msgs::CarCmd::ConstPtr& msg) {
    delayedMsgs_.emplace_back(ros::Time::now(), msg->a, msg->varepsilon);
    //std::cout << "sim_cmd_callback_a: " << msg->a << " sim_cmd_callback_varepsilon: " << msg->varepsilon << std::endl;
    // input_(0) = msg->a;
    // input_(1) = msg->varepsilon;
  }
  void timer_callback(const ros::TimerEvent& event) {
    if (!delayedMsgs_.empty()) {
      auto& msg = delayedMsgs_.front();
      if ((ros::Time::now() - msg.t).toSec() > delay_) {
        input_(0) = msg.a;
        input_(1) = msg.varepsilon;
        //std::cout << "sim_timer_callback_a: " << msg.a << " sim_timer_callback_varepsilon: " << msg.varepsilon << std::endl;
        delayedMsgs_.pop_front();
      }
    }

    car.step(input_, 1.0 / 400);
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";
    odom_msg.pose.pose.position.x = car.state(0);
    odom_msg.pose.pose.position.y = car.state(1);
    odom_msg.pose.pose.position.z = 0.0;
    double phi = car.state(2);
    double v = car.state(3);
    double delta = car.state(4); 
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sin(phi / 2);
    odom_msg.pose.pose.orientation.w = cos(phi / 2);

    odom_msg.twist.twist.linear.x = v * cos(phi) * cos(delta);
    odom_msg.twist.twist.linear.y = v * sin(phi) * cos(delta);
    odom_msg.twist.twist.linear.z = 0;
    //std::cout <<"odom_msg.phi: "<< phi << std::endl;
    //std::cout <<"odom_msg.v: "<< v << std::endl;
    //std::cout <<"odom_msg.delta: "<< delta << std::endl;
    odom_pub_.publish(odom_msg);
  }

 public:
  void onInit(void) {
    ROS_WARN("sim_1");
    ros::NodeHandle nh(getMTPrivateNodeHandle());
    nh.getParam("L", car.L);
    Vector5d initS;
    nh.getParam("x", initS(0));
    nh.getParam("y", initS(1));
    nh.getParam("phi", initS(2));
    nh.getParam("v", initS(3));
    nh.getParam("delta", initS(4));
    nh.getParam("delay", delay_);
    input_.setZero();
    car.setInitialState(initS);
    ROS_WARN("sim_2");
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 1);
    cmd_sub_ = nh.subscribe<car_msgs::CarCmd>("car_cmd", 1, &Nodelet::cmd_callback, this, ros::TransportHints().tcpNoDelay());
    
    ROS_WARN("car_cmd");
    sim_timer_ = nh.createTimer(ros::Duration(1.0 / 400), &Nodelet::timer_callback, this);
    ROS_WARN("sim_3");
  }
};
}  // namespace car_simulator

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(car_simulator::Nodelet, nodelet::Nodelet);
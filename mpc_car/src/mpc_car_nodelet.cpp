#include <car_msgs/CarCmd.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>

// todo include omniGKFcmd and omniGKFinfo
#include <omniGKF_control/omniGKFcmd.h>

#include <omniGKF_control/omniGKFinfo.h>
// todo include omniGKFcmd and omniGKFinfo
#include <omniGKF_control/omniGKFcmd.h>

#include <omniGKF_control/omniGKFinfo.h>
// todo include omniGKFcmd and omniGKFinfo
#include <omniGKF_control/omniGKFcmd.h>

#include <omniGKF_control/omniGKFinfo.h>

#include <Eigen/Geometry>
#include <mpc_car/mpc_car.hpp>

namespace mpc_car
{
  class Nodelet : public nodelet::Nodelet
  {
  private:
    std::shared_ptr<MpcCar> mpcPtr_;
    ros::Timer plan_timer_;
    ros::Subscriber odom_sub_;
    ros::Subscriber odom_sub_head_;
    ros::Subscriber omni_odom_sub_;

    ros::Subscriber odom_sub_head_;
    ros::Subscriber omni_odom_sub_;

    ros::Publisher cmd_pub_;
    ros::Publisher cmd_omniGKF_pub_;

    ros::Publisher cmd_omniGKF_pub_;

    VectorX state_;
    bool init_1 = false;
    bool init_2 = false;
    bool init_1 = false;
    bool init_2 = false;
    double delay_ = 0.0;
    bool nmpc_ = false;

    void plan_timer_callback(const ros::TimerEvent &event)
    {
      if (init_1 == true && init_2 == true)
      {
        int ret = 0;
        if (nmpc_)
        {
          ros::Time t1 = ros::Time::now();
          ret = mpcPtr_->solveNMPC(state_);
          ros::Time t2 = ros::Time::now();
          double solve_time = (t2 - t1).toSec();
          std::cout << "solve nmpc costs: " << 1e3 * solve_time << "ms" << std::endl;
        }
        else
        {
          ros::Time t1 = ros::Time::now();
          ret = mpcPtr_->solveQP(state_);
          ros::Time t2 = ros::Time::now();
          double solve_time = (t2 - t1).toSec();
          std::cout << "solve qp costs: " << 1e3 * solve_time << "ms" << std::endl;
        }
        assert(ret == 1);

        // simulation
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

        // TODO 输出omniGKFcmd
        omniGKF_control::omniGKFcmd msg_omniGKF;
        msg_omniGKF.header.frame_id = "world";
        msg_omniGKF.header.stamp = ros::Time::now();

        VectorX x_omniGKF;
        VectorU u_omniGKF;
        mpcPtr_->getPredictXU(0, x_omniGKF, u_omniGKF);
        std::cout << "u_omniGKF: " << u_omniGKF.transpose() << std::endl;
        std::cout << "x_omniGKF: " << x_omniGKF.transpose() << std::endl;

        // 输出msg
        std::cout << "msg.a =  " << msg.a << std::endl;
        std::cout << "msg.varepsilon =  " << msg.varepsilon << std::endl;
        mpcPtr_->visualization();
        cmd_omniGKF_pub_.publish(msg_omniGKF);
      }
      return;
    }

    void odom_call_back(const nav_msgs::Odometry::ConstPtr &msg)
    {
      // ROS_WARN("odom_call_back");
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
      Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
      double vw = msg->twist.twist.linear.z;
      double delta = msg->twist.twist.angular.z;
      // bug#002
      state_ << x, y, euler.z(), vw, delta;
      // std::cout<<"MPC_odom_state_ = "<<state_.transpose()<<std::endl;
      // 这里的state(3),state(4)分别需要是机器人的速度和转向角速度，v.norm()为人的速度，那么当前的里程计信息如何转化为机器人转向角速度

      init_1 = true;
      init_2 = true;
    }

    void odom_call_back_head(const nav_msgs::Odometry::ConstPtr &msg)
    {
      ROS_WARN("odom_call_back_head");
      double x = msg->pose.pose.position.x;
      double y = msg->pose.pose.position.y;
      Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
      Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      // Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
      // double vw = msg_omniGKF->velocity[0];
      // double delta = msg_omniGKF->heading;
      // bug#002
      state_.head(3) << x, y, euler.z();
      // std::cout<<"MPC_odom_state_ = "<<state_.transpose()<<std::endl;
      // 这里的state(3),state(4)分别需要是机器人的速度和转向角速度，v.norm()为人的速度，那么当前的里程计信息如何转化为机器人转向角速度

      init_1 = true;
    }

    void omni_odom_call_back(const omniGKF_control::omniGKFinfo::ConstPtr &msg_omniGKF)
    {
      ROS_WARN("omni_odom_call_back");
      // double x = msg->pose.pose.position.x;
      // double y = msg->pose.pose.position.y;
      // Eigen::Quaterniond q(msg->pose.pose.orientation.w,
      //                      msg->pose.pose.orientation.x,
      //                      msg->pose.pose.orientation.y,
      //                      msg->pose.pose.orientation.z);
      // Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      // Eigen::Vector2d v(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
      double vw = msg_omniGKF->velocity[0];
      double delta = msg_omniGKF->heading;

      // bug#002
      state_.tail(2) << vw, delta;

      init_2 = true;
    }

  public:
    void onInit(void)
    {
      ros::NodeHandle nh(getMTPrivateNodeHandle());
      mpcPtr_ = std::make_shared<MpcCar>(nh);
      double dt = 0;
      nh.getParam("dt", dt);
      nh.getParam("delay", delay_);
      nh.getParam("nmpc", nmpc_);

      plan_timer_ = nh.createTimer(ros::Duration(dt), &Nodelet::plan_timer_callback, this);
      odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &Nodelet::odom_call_back, this);
      cmd_pub_ = nh.advertise<car_msgs::CarCmd>("car_cmd", 1);

      // todo
      odom_sub_head_ = nh.subscribe<nav_msgs::Odometry>("Odometry", 1, &Nodelet::odom_call_back_head, this);
      // 下面的omni_odom_sub_没有正确的调用，导致omni_odom_call_back函数没有被调用
      omni_odom_sub_ = nh.subscribe<omniGKF_control::omniGKFinfo>("omniGKFinfo", 1, &Nodelet::omni_odom_call_back, this);
      //rqt_graph显示的是omniGKFinfo_cmd没有发布
      cmd_omniGKF_pub_ = nh.advertise<omniGKF_control::omniGKFcmd>("omniGKFcmd", 1);
    }
  };
} // namespace mpc_car

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mpc_car::Nodelet, nodelet::Nodelet);
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <Eigen/Geometry>
#include <pid_car/pid_car.hpp>

namespace mpc_car
{
    class PidNodelet : public nodelet::Nodelet
    {
    private:
        std::shared_ptr<PidCar> pidPtr_;
        ros::Timer plan_timer_;
        ros::Subscriber odom_sub_;

        VectorS state_;

        void plan_timer_callback(const ros::TimerEvent &event)
        {
            ROS_INFO("plan_timer_callback");
        }

    public:
        void onInit(void)
        {
            ros::NodeHandle nh(getMTPrivateNodeHandle());
            pidPtr_ = std::make_shared<PidCar>(nh);
            double dt = 0;
            nh.getParam("dt", dt);

            plan_timer_ = nh.createTimer(ros::Duration(dt), &PidNodelet::plan_timer_callback, this);
        }
    };

} // namespace mpc_car

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mpc_car::PidNodelet, nodelet::Nodelet);
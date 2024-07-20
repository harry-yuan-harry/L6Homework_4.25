#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <serial/serial.h>

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

        serial::Serial ser_;
        bool use_serial_ = false;
        std::string port_;
        int baudrate_;
        VectorS state_;

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

            state_ << x, y, euler.z();
        }

        void plan_timer_callback(const ros::TimerEvent &event)
        {
            int ret = 0;
            double pid_out = 0;
            ret = pidPtr_->solvePID(state_, pid_out);
            assert(ret == 1);
            if (use_serial_)
            {
                std::string send_data = "z" + std::to_string((int)(pid_out * 100)) + "\n";
                u_char send_data_char[send_data.size()];
                for (size_t i = 0; i < send_data.size(); i++)
                    send_data_char[i] = send_data.c_str()[i];
                ser_.write(send_data_char, send_data.size());
            }
            else
            {
                std::string send_data = "z" + std::to_string((int)(pid_out * 100)) + "\n";
                std::cout << send_data;
                // TODO,让仿真动起来
            }

            pidPtr_->visualization();
        }

        int SerialInit()
        {
            ser_.setPort(port_);
            ser_.setBaudrate(baudrate_);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ROS_WARN("---[param] serial set:%s, baudrate set:%d----", port_.c_str(), baudrate_);
            try
            {
                ser_.open();
            }
            catch (serial::IOException &e)
            {
                ROS_ERROR_STREAM("Unable to open port ");
                return 0;
            }
            if (ser_.isOpen())
            {
                ROS_INFO_STREAM("Serial Port initialized");
            }
            else
            {
                ROS_ERROR_STREAM("Serial Port fail");
                return 0;
            }
            std::string send_data = "z000\n";
            u_char send_data_char[send_data.size()];
            for (size_t i = 0; i < send_data.size(); i++)
                send_data_char[i] = send_data.c_str()[i];
            ser_.write(send_data_char, send_data.size());

            return 1;
        }

    public:
        void onInit(void)
        {
            ros::NodeHandle nh(getMTPrivateNodeHandle());
            pidPtr_ = std::make_shared<PidCar>(nh);
            double dt = 0;
            nh.getParam("dt", dt);
            nh.getParam("use_serial", use_serial_);
            nh.getParam("port", port_);
            nh.getParam("baudrate", baudrate_);

            // serial use initialization
            if (use_serial_)
            {
                int ret = 0;
                ret = SerialInit();
                assert(ret == 1);
            }

            plan_timer_ = nh.createTimer(ros::Duration(dt), &PidNodelet::plan_timer_callback, this);
            odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 1, &PidNodelet::odom_call_back, this);
        }
    };

} // namespace mpc_car

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mpc_car::PidNodelet, nodelet::Nodelet);
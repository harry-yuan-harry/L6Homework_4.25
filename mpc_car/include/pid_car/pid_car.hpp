#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <arc_spline/arc_spline.hpp>
#include <tf/transform_datatypes.h>
#include <deque>

namespace mpc_car
{
    // state x y phi
    typedef Eigen::Matrix<double, 3, 1> VectorS;

    class PidCar
    {
    private:
        ros::NodeHandle nh_;
        ros::Publisher ref_pub_;
        ros::Publisher ref_state_pub_;

        arc_spline::ArcSpline s_;
        double desired_v_;

        int N_;
        double dt_;
        double prev_error_;
        double error_sum_;

        double kp_;
        double ki_;

        VectorS x0_observe_;

        std::vector<Eigen::VectorXd> reference_states_;

        VectorS compensateDelay(const VectorS &x0)
        {
            VectorS x0_delay = x0;
            // : compensate delay
            // ...
            // double dt = 0.001;
            // for (double t = delay_; t > 0; t -= dt) {
            //   int i = ceil(t / dt_);
            //   VectorU input = historyInput_[history_length_ - i];
            //   step(x0_delay, input, dt);
            // }

            return x0_delay;
        }

    public:
        PidCar(ros::NodeHandle &nh) : nh_(nh)
        {
            // load map
            std::vector<double> track_points_x, track_points_y;
            nh.getParam("track_points_x", track_points_x);
            nh.getParam("track_points_y", track_points_y);
            s_.setWayPoints(track_points_x, track_points_y);

            nh.getParam("desired_v", desired_v_);
            nh.getParam("dt", dt_);
            nh.getParam("N", N_);
            nh.getParam("Kp", kp_);
            nh.getParam("Ki", ki_);

            ref_pub_ = nh.advertise<nav_msgs::Path>("reference_path", 1);
            ref_state_pub_ = nh.advertise<geometry_msgs::PoseStamped>("ref_state", 1);

            prev_error_ = 0.0;
            error_sum_ = 0.0;
        }

        int solvePID(const VectorS &x0_observe, double &pid_out)
        {
            x0_observe_ = compensateDelay(x0_observe); // compensate delay
            // set reference states
            double s0 = s_.findS(x0_observe_.head(2));
            reference_states_.resize(N_);
            Eigen::Vector2d xy_r;
            double phi, last_phi = x0_observe_(2);

            for (int i = 0; i < N_; ++i)
            {
                s0 += desired_v_ * dt_;
                s0 = s0 < s_.arcL() ? s0 : s_.arcL();
                // calculate desired x,y.phi`
                xy_r = s_(s0);
                Eigen::Vector2d dxy = s_(s0, 1);
                phi = std::atan2(dxy.y(), dxy.x());
                if (phi - last_phi > M_PI)
                {
                    phi -= 2 * M_PI;
                }
                else if (phi - last_phi < -M_PI)
                {
                    phi += 2 * M_PI;
                }
                last_phi = phi;
                reference_states_[i] = Eigen::Vector3d(xy_r.x(), xy_r.y(), phi);
            }
            double ref_phi = reference_states_[N_ - 1](2);
            double obs_phi = x0_observe_(2);
            auto err = obs_phi - ref_phi;
            error_sum_ += err;

            // double pid_out = kp_ * err + ki_ * error_sum_;
            pid_out = kp_ * err;

            std::cout << "\033[32m"
                      << "error: " << err << " " << "pid_out: " << pid_out
                      << "\033[0m" << std::endl;
            return 1;
        }

        void visualization()
        {
            nav_msgs::Path msg;
            msg.header.frame_id = "world";
            msg.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped p;
            for (double s = 0; s < s_.arcL(); s += 0.01)
            {
                p.header.stamp = ros::Time::now();
                p.pose.position.x = s_(s).x();
                p.pose.position.y = s_(s).y();
                p.pose.position.z = 0.0;
                msg.poses.push_back(p);
            }
            ref_pub_.publish(msg);
            msg.poses.clear();

            geometry_msgs::PoseStamped msg1;
            msg1.header.stamp = ros::Time::now();
            msg1.header.frame_id = "world";
            msg1.pose.position.x = reference_states_[N_ - 1](0);
            msg1.pose.position.y = reference_states_[N_ - 1](1);
            msg1.pose.position.z = 0.0;
            double ref_phi = reference_states_[N_ - 1](2);
            geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(ref_phi);
            msg1.pose.orientation = quat;
            ref_state_pub_.publish(msg1);
        }
    };
} // namespace mpc_car

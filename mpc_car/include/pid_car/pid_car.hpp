#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <arc_spline/arc_spline.hpp>
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

            ref_pub_ = nh.advertise<nav_msgs::Path>("reference_path", 1);
            ref_state_pub_ = nh.advertise<geometry_msgs::PoseStamped>("ref_state", 1);
        }

        int solvePID(const VectorS &x0_observe)
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
            return 0;
        }
    };
} // namespace mpc_car

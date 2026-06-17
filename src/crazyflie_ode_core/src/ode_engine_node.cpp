#include <chrono>
#include <memory>
#include <algorithm>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace std::chrono_literals;
using namespace Eigen;

typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, 4, 1> Vector4d;

// --- 障碍物数据结构 ---
struct CylinderObstacle {
    std::string id;
    double x, y, z;
    double radius;
    double height;
};

class OdeEngineNode : public rclcpp::Node {
public:
    OdeEngineNode() : Node("crazyflie_ode_engine") {
        // --- 1. solver + step size ---
        this->declare_parameter<std::string>("solver", "rk4");
        this->declare_parameter<double>("step_size", 0.01);
        this->get_parameter("solver", solver_type_);
        this->get_parameter("step_size", h_step_);

        // --- 2. num_drones ---
        this->declare_parameter<int>("num_drones", 1);
        this->get_parameter("num_drones", num_drones_);
        if (num_drones_ < 1) num_drones_ = 1;

        clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/obstacles", rclcpp::QoS(20).transient_local());

        // --- 3. obstacles ---
        this->declare_parameter<bool>("enable_obstacles", true);
        this->get_parameter("enable_obstacles", enable_obstacles_);

        if (enable_obstacles_) {
            loadObstaclesFromYAML();
        } else {
            RCLCPP_INFO(this->get_logger(), "Obstacles DISABLED by parameter.");
        }

        // --- batch init ---
        states_.resize(num_drones_);
        target_poses_.resize(num_drones_);

        for (int i = 0; i < num_drones_; ++i) {
            std::string prefix = "/cf" + std::to_string(i);
            odom_pubs_.push_back(this->create_publisher<nav_msgs::msg::Odometry>(prefix + "/odom", 10));
            target_pubs_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/target", 10));

            double angle = i * (2.0 * M_PI / num_drones_);
            states_[i].setZero();
            states_[i](0) = 1.5 * cos(angle);
            states_[i](1) = 1.5 * sin(angle);
            states_[i](2) = 0.5;
            target_poses_[i] << 0.0, states_[i](1), 1.5;
        }

        mass_ = 0.027; g_ = 9.81;
        I_diag_ << 2.395e-5, 2.395e-5, 3.235e-5;
        I_matrix_ = I_diag_.asDiagonal();
        I_inv_ = I_matrix_.inverse();

        sim_time_ns_ = 0; target_yaw_ = 0.0;

        auto timer_period = std::chrono::duration<double>(h_step_);
        timer_ = this->create_wall_timer(timer_period, std::bind(&OdeEngineNode::physicsStepLoop, this));

        if (!obstacles_.empty()) {
            publishObstacleMarkersInit();
        }

        RCLCPP_INFO(this->get_logger(),
            "Swarm ODE Engine Ready | Drones: %d | Solver: %s | dt: %.4f | Obstacles: %zu",
            num_drones_, solver_type_.c_str(), h_step_, obstacles_.size());
    }

private:
    int num_drones_;
    std::vector<Vector12d> states_;
    std::vector<Vector3d> target_poses_;
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> target_pubs_;

    bool enable_obstacles_ = true;
    std::vector<CylinderObstacle> obstacles_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
    double d_safe_obs_ = 0.5;
    double k_rep_obs_  = 8.0;

    void loadObstaclesFromYAML() {
        try {
            std::string pkg_path = ament_index_cpp::get_package_share_directory("crazyflie_ode_core");
            std::string yaml_path = pkg_path + "/config/obstacles.yaml";
            RCLCPP_INFO(this->get_logger(), "Loading obstacles from: %s", yaml_path.c_str());
            YAML::Node config = YAML::LoadFile(yaml_path);
            if (!config["obstacles"]) {
                RCLCPP_WARN(this->get_logger(), "No 'obstacles' key found in YAML config.");
                return;
            }
            for (const auto& node : config["obstacles"]) {
                CylinderObstacle obs;
                obs.id     = node["id"].as<std::string>("unnamed");
                obs.x      = node["x"].as<double>(0.0);
                obs.y      = node["y"].as<double>(0.0);
                obs.z      = node["z"].as<double>(0.75);
                obs.radius = node["radius"].as<double>(0.15);
                obs.height = node["height"].as<double>(1.5);
                obstacles_.push_back(obs);
                RCLCPP_INFO(this->get_logger(), "  Obstacle [%s]: pos=(%.2f, %.2f, %.2f) r=%.2f h=%.2f",
                    obs.id.c_str(), obs.x, obs.y, obs.z, obs.radius, obs.height);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacle(s).", obstacles_.size());
        } catch (const YAML::BadFile& e) {
            RCLCPP_WARN(this->get_logger(), "Obstacle YAML file not found.");
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parse error: %s", e.what());
        }
    }

    Vector3d computeObstacleRepulsion(const Vector3d& pos, const Vector3d& vel) {
        Vector3d a_rep(0.0, 0.0, 0.0);
        if (!enable_obstacles_ || obstacles_.empty()) return a_rep;

        for (const auto& obs : obstacles_) {
            double dx = pos(0) - obs.x;
            double dy = pos(1) - obs.y;
            double d_xy = std::sqrt(dx*dx + dy*dy);
            if (d_xy < 1e-4) { d_xy = 1e-4; dx = 1e-4; }
            double half_height = obs.height / 2.0;
            double dz = pos(2) - obs.z;
            double d_surf_xy = d_xy - obs.radius;
            double dz_abs = std::abs(dz);
            double d_surf_z  = dz_abs - half_height;
            bool in_xy = d_surf_xy < d_safe_obs_;
            bool in_z  = d_surf_z  < d_safe_obs_;
            if (!in_xy || !in_z) continue;

            double t_xy = std::clamp(1.0 - d_surf_xy / d_safe_obs_, 0.0, 1.0);

            if (d_surf_xy > 1e-4) {
                double f_radial = k_rep_obs_ * t_xy * t_xy * t_xy;
                a_rep(0) += f_radial * (dx / d_xy);
                a_rep(1) += f_radial * (dy / d_xy);
            } else {
                a_rep(0) += k_rep_obs_ * 3.0 * (dx / d_xy);
                a_rep(1) += k_rep_obs_ * 3.0 * (dy / d_xy);
            }

            double v_radial = vel(0) * (dx / d_xy) + vel(1) * (dy / d_xy);
            if (v_radial > 0.0) {
                double damp = k_rep_obs_ * 0.6 * t_xy * t_xy * v_radial;
                a_rep(0) -= damp * (dx / d_xy);
                a_rep(1) -= damp * (dy / d_xy);
            }

            if (d_surf_xy < d_safe_obs_ * 0.7 && d_surf_xy > 1e-4) {
                double t_tan = 1.0 - d_surf_xy / (d_safe_obs_ * 0.7);
                Vector3d radial_dir(dx / d_xy, dy / d_xy, 0.0);
                Vector3d tangent_dir(-radial_dir(1), radial_dir(0), 0.0);
                double v_dot_tan = vel(0) * tangent_dir(0) + vel(1) * tangent_dir(1);
                if (v_dot_tan < 0) tangent_dir = -tangent_dir;
                double f_tan = k_rep_obs_ * 0.35 * t_tan * t_tan * t_tan;
                a_rep(0) += f_tan * tangent_dir(0);
                a_rep(1) += f_tan * tangent_dir(1);
            }

            if (d_surf_z > 1e-4 && d_surf_z < d_safe_obs_) {
                double t_z = std::clamp(1.0 - d_surf_z / d_safe_obs_, 0.0, 1.0);
                double f_z = k_rep_obs_ * 0.5 * t_z * t_z * t_z;
                double z_sign = (dz > 0) ? 1.0 : -1.0;
                a_rep(2) += f_z * z_sign;
            }
        }
        double mag = a_rep.norm();
        if (mag > 10.0) a_rep = a_rep * (10.0 / mag);
        return a_rep;
    }

    void publishObstacleMarkersInit() {
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_marker.header.frame_id = "odom";
        clear_marker.header.stamp = this->now();
        obstacle_marker_pub_->publish(clear_marker);
        publishObstacleMarkers();
    }

    void publishObstacleMarkers() {
        int marker_id = 0;
        for (const auto& obs : obstacles_) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "odom";
            m.header.stamp    = this->now();
            m.ns       = "obstacles";
            m.id       = marker_id++;
            m.type     = visualization_msgs::msg::Marker::CYLINDER;
            m.action   = visualization_msgs::msg::Marker::ADD;
            m.lifetime = rclcpp::Duration::from_seconds(0);
            m.pose.position.x = obs.x;
            m.pose.position.y = obs.y;
            m.pose.position.z = obs.z;
            m.pose.orientation.w = 1.0;
            m.scale.x = obs.radius * 2.0;
            m.scale.y = obs.radius * 2.0;
            m.scale.z = obs.height;
            m.color.r = 1.0f; m.color.g = 0.2f; m.color.b = 0.2f; m.color.a = 0.6f;
            obstacle_marker_pub_->publish(m);
        }
    }

    Vector12d crazyflie_dynamics(const Vector12d & x, const Vector4d & u) {
        Vector12d dxdt;
        Vector3d v = x.segment<3>(3);
        Vector3d euler = x.segment<3>(6);
        Vector3d omega = x.segment<3>(9);
        double phi = euler(0), psi = euler(2);
        double theta = std::clamp(euler(1), -1.5, 1.5);
        dxdt.segment<3>(0) = v;
        Matrix3d R = (AngleAxisd(psi, Vector3d::UnitZ()) * AngleAxisd(theta, Vector3d::UnitY()) * AngleAxisd(phi, Vector3d::UnitX())).toRotationMatrix();
        dxdt.segment<3>(3) = Vector3d(0, 0, -g_) + (1.0 / mass_) * R * Vector3d(0, 0, u(0));
        Matrix3d W;
        W << 1, sin(phi)*tan(theta), cos(phi)*tan(theta), 0, cos(phi), -sin(phi), 0, sin(phi)/cos(theta), cos(phi)/cos(theta);
        dxdt.segment<3>(6) = W * omega;
        dxdt.segment<3>(9) = I_inv_ * (Vector3d(u(1), u(2), u(3)) - omega.cross(I_matrix_ * omega));
        return dxdt;
    }

    Vector12d safe_dynamics(Vector12d probe_state, const Vector4d& u) {
        probe_state(7) = std::clamp(probe_state(7), -1.45, 1.45);
        probe_state(9)  = std::clamp(probe_state(9),  -40.0, 40.0);
        probe_state(10) = std::clamp(probe_state(10), -40.0, 40.0);
        probe_state(11) = std::clamp(probe_state(11), -40.0, 40.0);
        return crazyflie_dynamics(probe_state, u);
    }

    Vector4d cascade_pid_control(const Vector12d& state, const Vector3d& target_pos, const Vector3d& a_rep) {
        Vector3d pos = state.segment<3>(0);
        Vector3d vel = state.segment<3>(3);
        Vector3d euler = state.segment<3>(6);
        Vector3d omega = state.segment<3>(9);
        Vector3d vel_des = Vector3d(1.5, 1.5, 1.5).cwiseProduct(target_pos - pos);
        vel_des = vel_des.cwiseMin(1.0).cwiseMax(-1.0);
        Vector3d acc_des = Vector3d(2.0, 2.0, 3.0).cwiseProduct(vel_des - vel) + a_rep;
        acc_des = acc_des.cwiseMin(4.0).cwiseMax(-4.0);
        double thrust = std::clamp(mass_ * (g_ + acc_des(2)), 0.0, 0.6);
        double phi_des = std::clamp((1.0/g_)*(acc_des(0)*sin(target_yaw_) - acc_des(1)*cos(target_yaw_)), -0.52, 0.52);
        double theta_des = std::clamp((1.0/g_)*(acc_des(0)*cos(target_yaw_) + acc_des(1)*sin(target_yaw_)), -0.52, 0.52);
        Vector3d omega_des = Vector3d(8.0, 8.0, 3.0).cwiseProduct(Vector3d(phi_des, theta_des, target_yaw_) - euler);
        Vector3d tau = Vector3d(0.002, 0.002, 0.003).cwiseProduct(omega_des - omega);
        return Vector4d(thrust, tau(0), tau(1), tau(2));
    }

    Vector3d compute_repulsive_acc(int current_idx) {
        Vector3d a_rep(0.0, 0.0, 0.0);
        Vector3d p_i = states_[current_idx].segment<3>(0);
        double d_safe = 0.6, k_rep = 3.0;
        for (int j = 0; j < num_drones_; ++j) {
            if (current_idx == j) continue;
            Vector3d p_j = states_[j].segment<3>(0);
            Vector3d dir = p_i - p_j;
            double dist = dir.norm();
            if (dist < d_safe && dist > 0.01) {
                double force_mag = k_rep * (1.0 / dist - 1.0 / d_safe) * (1.0 / (dist * dist));
                a_rep += force_mag * (dir / dist);
            }
        }
        return a_rep;
    }

    Vector12d euler_step(const Vector12d& x, const Vector4d& u, double h) { return x + h * crazyflie_dynamics(x, u); }

    Vector12d heun_step(const Vector12d& x, const Vector4d& u, double h) {
        Vector12d k1 = crazyflie_dynamics(x, u);
        return x + (h/2.0) * (k1 + crazyflie_dynamics(x + h*k1, u));
    }

    Vector12d rk4_step(const Vector12d& x, const Vector4d& u, double h) {
        Vector12d k1 = crazyflie_dynamics(x, u), k2 = crazyflie_dynamics(x + 0.5*h*k1, u);
        Vector12d k3 = crazyflie_dynamics(x + 0.5*h*k2, u), k4 = crazyflie_dynamics(x + h*k3, u);
        return x + (h/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
    }

    Vector12d rk45_step(const Vector12d& x, const Vector4d& u, double h_total) {
        double t_local = 0.0; double h = h_total; Vector12d cur_x = x; double tol = 1e-4;
        while (t_local < h_total) {
            if (t_local + h > h_total) h = h_total - t_local;
            Vector12d k1 = safe_dynamics(cur_x, u);
            Vector12d k2 = safe_dynamics(cur_x + h * (1.0/4.0) * k1, u);
            Vector12d k3 = safe_dynamics(cur_x + h * (3.0/32.0) * k1 + h * (9.0/32.0) * k2, u);
            Vector12d k4 = safe_dynamics(cur_x + h * (1932.0/2197.0) * k1 - h * (7200.0/2197.0) * k2 + h * (7296.0/2197.0) * k3, u);
            Vector12d k5 = safe_dynamics(cur_x + h * (439.0/216.0) * k1 - h * 8.0 * k2 + h * (3680.0/513.0) * k3 - h * (845.0/4104.0) * k4, u);
            Vector12d k6 = safe_dynamics(cur_x - h * (8.0/27.0) * k1 + h * 2.0 * k2 - h * (3544.0/2565.0) * k3 + h * (1859.0/4104.0) * k4 - h * (11.0/40.0) * k5, u);
            Vector12d x4 = cur_x + h * ((25.0/216.0)*k1 + (1408.0/2565.0)*k3 + (2197.0/4104.0)*k4 - (1.0/5.0)*k5);
            Vector12d x5 = cur_x + h * ((16.0/135.0)*k1 + (6656.0/12825.0)*k3 + (28561.0/56430.0)*k4 - (9.0/50.0)*k5 + (2.0/55.0)*k6);
            if (std::isnan(x5(0)) || std::isnan(x5(7))) { h = h * 0.1; continue; }
            double error = (x5 - x4).norm();
            if (error <= tol || h < 1e-6) { t_local += h; cur_x = x5; }
            double s = 0.84 * pow(tol / (error + 1e-10), 0.25);
            h = h * std::max(0.1, std::min(4.0, s));
        }
        return cur_x;
    }

    Vector12d implicit_euler_step(const Vector12d& x, const Vector4d& u, double h) {
        Vector12d x_next = x;
        for(int i=0; i<5; ++i) {
            Vector12d f = x_next - x - h * crazyflie_dynamics(x_next, u);
            Matrix<double, 12, 12> J = Matrix<double, 12, 12>::Identity();
            double eps = 1e-6;
            for(int j=0; j<12; ++j) {
                Vector12d x_eps = x_next; x_eps(j) += eps;
                J.col(j) -= h * (crazyflie_dynamics(x_eps, u) - crazyflie_dynamics(x_next, u)) / eps;
            }
            x_next -= J.inverse() * f;
        }
        return x_next;
    }

    void physicsStepLoop() {
        sim_time_ns_ += static_cast<uint64_t>(h_step_ * 1e9);
        double t_sec = sim_time_ns_ / 1e9;
        rclcpp::Time current_sim_time(sim_time_ns_);

        auto clock_msg = rosgraph_msgs::msg::Clock();
        clock_msg.clock.sec = sim_time_ns_ / 1000000000;
        clock_msg.clock.nanosec = sim_time_ns_ % 1000000000;
        clock_publisher_->publish(clock_msg);

        double base_radius = 1.1;
        double shrink_amplitude = 0.6;
        double radial_freq = 0.4;
        double angular_freq = 0.8;
        double current_radius = base_radius + shrink_amplitude * cos(radial_freq * t_sec);

        for (int i = 0; i < num_drones_; ++i) {
            double phase_offset = i * (2.0 * M_PI / num_drones_);
            double current_angle = phase_offset + angular_freq * t_sec;
            target_poses_[i](0) = current_radius * cos(current_angle);
            target_poses_[i](1) = current_radius * sin(current_angle);
            target_poses_[i](2) = 1.5 + 0.2 * sin(1.0 * t_sec + phase_offset);

            geometry_msgs::msg::PoseStamped target_msg;
            target_msg.header.stamp = current_sim_time;
            target_msg.header.frame_id = "odom";
            target_msg.pose.position.x = target_poses_[i](0);
            target_msg.pose.position.y = target_poses_[i](1);
            target_msg.pose.position.z = target_poses_[i](2);
            target_pubs_[i]->publish(target_msg);

            Vector3d inter_drone_rep = compute_repulsive_acc(i);
            Vector3d pos_i = states_[i].segment<3>(0);
            Vector3d vel_i = states_[i].segment<3>(3);
            Vector3d obstacle_rep = computeObstacleRepulsion(pos_i, vel_i);
            Vector4d u = cascade_pid_control(states_[i], target_poses_[i], inter_drone_rep + obstacle_rep);

            if (solver_type_ == "euler") states_[i] = euler_step(states_[i], u, h_step_);
            else if (solver_type_ == "heun") states_[i] = heun_step(states_[i], u, h_step_);
            else if (solver_type_ == "rk45") states_[i] = rk45_step(states_[i], u, h_step_);
            else if (solver_type_ == "implicit") states_[i] = implicit_euler_step(states_[i], u, h_step_);
            else states_[i] = rk4_step(states_[i], u, h_step_);

            publishState(i, current_sim_time);
        }

        if (!obstacles_.empty()) {
            static int marker_repub = 0;
            if (++marker_repub % 10 == 0) publishObstacleMarkers();
        }
    }

    void publishState(int i, const rclcpp::Time & t_now) {
        Quaterniond q = AngleAxisd(states_[i](8), Vector3d::UnitZ()) * AngleAxisd(states_[i](7), Vector3d::UnitY()) * AngleAxisd(states_[i](6), Vector3d::UnitX());
        std::string child_frame = "cf" + std::to_string(i) + "/base_link";

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = t_now;
        t.header.frame_id = "odom";
        t.child_frame_id = child_frame;
        t.transform.translation.x = states_[i](0);
        t.transform.translation.y = states_[i](1);
        t.transform.translation.z = states_[i](2);
        t.transform.rotation.w = q.w();
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        tf_broadcaster_->sendTransform(t);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = t_now;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = child_frame;
        odom_msg.pose.pose.position.x = states_[i](0);
        odom_msg.pose.pose.position.y = states_[i](1);
        odom_msg.pose.pose.position.z = states_[i](2);
        odom_msg.pose.pose.orientation.w = q.w();
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_pubs_[i]->publish(odom_msg);
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string solver_type_; double h_step_, mass_, g_;
    Matrix3d I_matrix_, I_inv_; Vector3d I_diag_; double target_yaw_;
    uint64_t sim_time_ns_;
};

int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<OdeEngineNode>()); rclcpp::shutdown(); return 0; }

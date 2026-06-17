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

#include "crazyflie_ode_core/solver_registry.hpp"
#include "crazyflie_ode_core/safety_monitor.hpp"
#include "crazyflie_ode_core/interval_reachability.hpp"

using namespace std::chrono_literals;
using namespace Eigen;

// --- 障碍物数据结构 ---
struct CylinderObstacle {
    std::string id;
    double x, y, z;      // 圆柱体中心位置
    double radius;        // 半径
    double height;        // 高度
};

class OdeEngineNode : public rclcpp::Node {
public:
    OdeEngineNode() : Node("crazyflie_ode_engine") {
        // --- 1. 读取原有的算法和步长参数 ---
        this->declare_parameter<std::string>("solver", "rk4");
        this->declare_parameter<double>("step_size", 0.01);
        this->get_parameter("solver", solver_type_);
        this->get_parameter("step_size", h_step_);

        // 构建统一求解器调度 (函数指针 dispatch, 避免每步 if/else)
        solver_type_enum_ = solverTypeFromString(solver_type_);
        buildSolverDispatch();

        // --- 2. 【核心新增】：动态读取集群数量参数 ---
        this->declare_parameter<int>("num_drones", 1); // 默认单机
        this->get_parameter("num_drones", num_drones_);

        // 增加一层安全保护，防止输入负数或 0
        if (num_drones_ < 1) num_drones_ = 1;

        clock_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 创建障碍物 Marker 发布者 (Transient Local: RViz 后连接也能收到, depth=20 容纳所有 markers)
        obstacle_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/obstacles", rclcpp::QoS(20).transient_local());

        // --- 3. 安全监视器参数 ---
        this->declare_parameter<double>("safety.v_max", 2.5);
        this->declare_parameter<double>("safety.theta_max", 0.8);
        this->declare_parameter<double>("safety.energy_drift_eps", 0.01);
        this->declare_parameter<std::string>("safety.log_path", "safety_log.csv");
        this->declare_parameter<bool>("safety.verbose", false);
        this->declare_parameter<double>("safety.warmup_time", 2.0);

        // --- 4. 在线安全控制器参数 ---
        this->declare_parameter<bool>("safety_controller.enabled", false);
        this->declare_parameter<int>("safety_controller.window_size", 100);
        this->declare_parameter<double>("safety_controller.threshold_halve", 0.7);
        this->declare_parameter<double>("safety_controller.threshold_switch", 0.3);
        this->declare_parameter<double>("safety_controller.threshold_stop", 0.05);

        bool sc_enabled;
        this->get_parameter("safety_controller.enabled", sc_enabled);
        if (sc_enabled) {
            int sc_win; double sc_h, sc_s, sc_st;
            this->get_parameter("safety_controller.window_size", sc_win);
            this->get_parameter("safety_controller.threshold_halve", sc_h);
            this->get_parameter("safety_controller.threshold_switch", sc_s);
            this->get_parameter("safety_controller.threshold_stop", sc_st);
            safety_controller_ = std::make_unique<SafetyController>(
                sc_win, 0.0001, 0.5, sc_h, sc_s, sc_st);
            RCLCPP_INFO(this->get_logger(),
                "Online Safety Controller ENABLED | "
                "window=%d thresholds: halve=%.2f switch=%.2f stop=%.2f",
                sc_win, sc_h, sc_s, sc_st);
        }

        // --- 5. 实验模式参数 ---
        this->declare_parameter<bool>("experiment.enabled", false);
        this->declare_parameter<double>("experiment.duration", 30.0);
        this->declare_parameter<std::string>("experiment.metrics_path", "experiment_metrics.csv");
        this->declare_parameter<bool>("experiment.headless", false);

        // --- 5. 障碍物开关参数 ---
        this->declare_parameter<bool>("enable_obstacles", true);
        this->get_parameter("enable_obstacles", enable_obstacles_);

        // 加载障碍物配置
        if (enable_obstacles_) {
            loadObstaclesFromYAML();
        } else {
            RCLCPP_INFO(this->get_logger(), "Obstacles DISABLED by parameter.");
        }

        // --- 核心重构：批量初始化集群 ---
        states_.resize(num_drones_);
        target_poses_.resize(num_drones_);

        for (int i = 0; i < num_drones_; ++i) {
            std::string prefix = "/cf" + std::to_string(i);

            // 为每架飞机创建独立的 Topic
            odom_pubs_.push_back(this->create_publisher<nav_msgs::msg::Odometry>(prefix + "/odom", 10));
            target_pubs_.push_back(this->create_publisher<geometry_msgs::msg::PoseStamped>(prefix + "/target", 10));

            // 初始化状态：环形起飞阵列 (半径 1.5 米)
            double angle = i * (2.0 * M_PI / num_drones_);
            states_[i].setZero();
            states_[i](0) = 1.5 * cos(angle); // X
            states_[i](1) = 1.5 * sin(angle); // Y
            states_[i](2) = 0.5;              // Z (起飞高度)     

            target_poses_[i] << 0.0, states_[i](1), 1.5;
        }

        mass_ = 0.027; g_ = 9.81;
        I_diag_ << 2.395e-5, 2.395e-5, 3.235e-5;
        I_matrix_ = I_diag_.asDiagonal();
        I_inv_ = I_matrix_.inverse();

        sim_time_ns_ = 0; step_count_ = 0;
        target_yaw_ = 0.0;

        // --- 构建安全监视器 ---
        {
            double v_max, theta_max, drift_eps, warmup;
            std::string log_path, metrics_path;
            bool verbose;
            this->get_parameter("safety.v_max", v_max);
            this->get_parameter("safety.theta_max", theta_max);
            this->get_parameter("safety.energy_drift_eps", drift_eps);
            this->get_parameter("safety.log_path", log_path);
            this->get_parameter("safety.verbose", verbose);
            this->get_parameter("safety.warmup_time", warmup);
            this->get_parameter("experiment.metrics_path", metrics_path);
            safety_monitor_ = std::make_unique<SafetyMonitor>(
                v_max, theta_max, drift_eps, log_path, verbose, metrics_path, warmup);
        }

        // --- 读取实验模式参数 ---
        this->get_parameter("experiment.enabled", experiment_enabled_);
        this->get_parameter("experiment.duration", experiment_duration_);
        this->get_parameter("experiment.headless", headless_);

        // --- 区间可达集参数 ---
        this->declare_parameter<bool>("reachability.enabled", false);
        this->declare_parameter<double>("reachability.epsilon", 0.01);
        this->declare_parameter<std::string>("reachability.method", "sampling");
        this->get_parameter("reachability.enabled", reachability_enabled_);
        if (reachability_enabled_ && !obstacles_.empty()) {
            double eps;
            std::string method_str;
            this->get_parameter("reachability.epsilon", eps);
            this->get_parameter("reachability.method", method_str);
            auto method = (method_str == "jacobian")
                ? IntervalReachability::Method::JACOBIAN
                : IntervalReachability::Method::SAMPLING;
            reachability_ = std::make_unique<IntervalReachability>(
                eps, obstacles_, method);
            RCLCPP_INFO(this->get_logger(),
                "Interval Reachability ENABLED | epsilon=%.4f method=%s",
                eps, method_str.c_str());
        }

        auto timer_period = std::chrono::duration<double>(h_step_);
        timer_ = this->create_wall_timer(timer_period, std::bind(&OdeEngineNode::physicsStepLoop, this));

        // 初始发布: 清除旧标记 + 发布当前障碍物
        if (!obstacles_.empty()) {
            publishObstacleMarkersInit();
        }

        RCLCPP_INFO(this->get_logger(), "\033[1;32m Swarm ODE Engine Ready | Drones: %d | Solver: %s | dt: %.4f | Obstacles: %zu \033[0m", num_drones_, solverTypeToString(solver_type_enum_).c_str(), h_step_, obstacles_.size());
    }

    ~OdeEngineNode() {
        if (safety_monitor_) {
            safety_monitor_->writeCSV();
            auto scores = safety_monitor_->getScores();
            RCLCPP_INFO(this->get_logger(),
                "\033[1;33m Safety Summary: collision=%.3f velocity=%.3f attitude=%.3f energy=%.3f overall=%.3f \033[0m",
                scores.collision_compliance, scores.velocity_compliance,
                scores.attitude_compliance, scores.energy_drift_compliance,
                scores.overall_score());
        }
    }

private:

    // --- 求解器调度 ---
    SolverType solver_type_enum_;
    SolverFn solver_fn_;

    void buildSolverDispatch() {
        switch (solver_type_enum_) {
            case SolverType::EULER:
                solver_fn_ = [this](const Vector12d& x, const Vector4d& u, double h) {
                    return this->euler_step(x, u, h);
                }; break;
            case SolverType::HEUN:
                solver_fn_ = [this](const Vector12d& x, const Vector4d& u, double h) {
                    return this->heun_step(x, u, h);
                }; break;
            case SolverType::RK4:
                solver_fn_ = [this](const Vector12d& x, const Vector4d& u, double h) {
                    return this->rk4_step(x, u, h);
                }; break;
            case SolverType::RK45:
                solver_fn_ = [this](const Vector12d& x, const Vector4d& u, double h) {
                    return this->rk45_step(x, u, h);
                }; break;
            case SolverType::IMPLICIT:
                solver_fn_ = [this](const Vector12d& x, const Vector4d& u, double h) {
                    return this->implicit_euler_step(x, u, h);
                }; break;
            default:
                solver_fn_ = [this](const Vector12d& x, const Vector4d& u, double h) {
                    return this->rk4_step(x, u, h);
                };
        }
    }

    int num_drones_;

    std::vector<Vector12d> states_;
    std::vector<Vector3d> target_poses_;
    
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odom_pubs_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> target_pubs_;

    // --- 障碍物相关成员 ---
    bool enable_obstacles_ = true;
    std::vector<CylinderObstacle> obstacles_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_marker_pub_;
    double d_safe_obs_ = 0.5;    // 障碍物安全距离 (提前预警)
    double k_rep_obs_  = 8.0;    // 障碍物排斥增益 (抵御编队拉力)

    // --- 安全监视器 ---
    std::unique_ptr<SafetyMonitor> safety_monitor_;

    // --- 实验模式 ---
    bool experiment_enabled_ = false;
    bool headless_ = true;
    double experiment_duration_ = 30.0;

    // --- 区间可达集 ---
    bool reachability_enabled_ = false;
    std::unique_ptr<IntervalReachability> reachability_;

    // --- 在线安全控制器 ---
    std::unique_ptr<SafetyController> safety_controller_;

    void applySafetyAction(SafetyAction action) {
        if (action == SafetyAction::NONE) return;
        auto scores = safety_monitor_->getScores();
        RCLCPP_WARN(this->get_logger(),
            "SAFETY ACTION: %s | overall=%.3f | trigger-count=%d",
            safetyActionName(action), scores.overall_score(),
            safety_controller_->actionCount());
        switch (action) {
            case SafetyAction::REDUCE_DT: {
                double ndt = safety_controller_->reducedStep(h_step_);
                if (ndt < h_step_ * 0.9) {
                    h_step_ = ndt;
                    timer_->cancel();
                    timer_ = this->create_wall_timer(
                        std::chrono::duration<double>(h_step_),
                        std::bind(&OdeEngineNode::physicsStepLoop, this));
                    RCLCPP_WARN(this->get_logger(), "  -> Step halved to dt=%.6f", h_step_);
                }
                break;
            }
            case SafetyAction::SWITCH_SOLVER: {
                SolverType ns = safety_controller_->saferSolver(solver_type_enum_);
                if (ns != solver_type_enum_) {
                    RCLCPP_WARN(this->get_logger(), "  -> Solver: %s -> %s",
                                solverTypeToString(solver_type_enum_).c_str(),
                                solverTypeToString(ns).c_str());
                    solver_type_enum_ = ns;
                    buildSolverDispatch();
                }
                break;
            }
            case SafetyAction::EMERGENCY_STOP:
                RCLCPP_ERROR(this->get_logger(), "  -> EMERGENCY STOP");
                safety_monitor_->writeCSV();
                rclcpp::shutdown();
                break;
            default: break;
        }
    }

    // --- 加载障碍物 YAML 配置 ---
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

                RCLCPP_INFO(this->get_logger(),
                    "  Obstacle [%s]: pos=(%.2f, %.2f, %.2f) r=%.2f h=%.2f",
                    obs.id.c_str(), obs.x, obs.y, obs.z, obs.radius, obs.height);
            }
            RCLCPP_INFO(this->get_logger(), "Loaded %zu obstacle(s).", obstacles_.size());
        } catch (const YAML::BadFile& e) {
            RCLCPP_WARN(this->get_logger(), "Obstacle YAML file not found, running without obstacles.");
        } catch (const YAML::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parse error: %s", e.what());
        }
    }

    // --- 障碍物排斥力计算 (平滑势场 + 速度阻尼 + 切向绕行) ---
    Vector3d computeObstacleRepulsion(const Vector3d& pos, const Vector3d& vel) {
        Vector3d a_rep(0.0, 0.0, 0.0);
        if (!enable_obstacles_ || obstacles_.empty()) return a_rep;

        for (const auto& obs : obstacles_) {
            double dx = pos(0) - obs.x;
            double dy = pos(1) - obs.y;
            double d_xy = std::sqrt(dx * dx + dy * dy);
            if (d_xy < 1e-4) { d_xy = 1e-4; dx = 1e-4; }

            double half_height = obs.height / 2.0;
            double dz = pos(2) - obs.z;

            double d_surf_xy = d_xy - obs.radius;
            double dz_abs = std::abs(dz);
            double d_surf_z  = dz_abs - half_height;

            bool in_xy = d_surf_xy < d_safe_obs_;
            bool in_z  = d_surf_z  < d_safe_obs_;
            if (!in_xy || !in_z) continue;

            // ---- 计算无量纲侵入比 ----
            double t_xy = std::clamp(1.0 - d_surf_xy / d_safe_obs_, 0.0, 1.0);

            // ================================================
            // 1. 径向排斥 (cubic ramp: 近处急剧增大)
            //    t³ 比 t² 在边界更柔和，在表面更强劲
            // ================================================
            if (d_surf_xy > 1e-4) {
                double f_radial = k_rep_obs_ * t_xy * t_xy * t_xy;
                a_rep(0) += f_radial * (dx / d_xy);
                a_rep(1) += f_radial * (dy / d_xy);
            } else {
                // 已穿透表面：最大力 x2 紧急推出
                a_rep(0) += k_rep_obs_ * 3.0 * (dx / d_xy);
                a_rep(1) += k_rep_obs_ * 3.0 * (dy / d_xy);
            }

            // ================================================
            // 2. 速度阻尼 — 抵抗朝向障碍物的速度分量
            //    这是防止穿透的关键：接近越快，制动越强
            // ================================================
            double v_radial = vel(0) * (dx / d_xy) + vel(1) * (dy / d_xy);
            if (v_radial > 0.0) {
                double damp = k_rep_obs_ * 0.6 * t_xy * t_xy * v_radial;
                a_rep(0) -= damp * (dx / d_xy);
                a_rep(1) -= damp * (dy / d_xy);
            }

            // ================================================
            // 3. 切向绕行引导
            // ================================================
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

            // ================================================
            // 4. Z 轴 (纵向) — cubic ramp, 力度减半
            // ================================================
            if (d_surf_z > 1e-4 && d_surf_z < d_safe_obs_) {
                double t_z = std::clamp(1.0 - d_surf_z / d_safe_obs_, 0.0, 1.0);
                double f_z = k_rep_obs_ * 0.5 * t_z * t_z * t_z;
                double z_sign = (dz > 0) ? 1.0 : -1.0;
                a_rep(2) += f_z * z_sign;
            }
        }

        // 上限 10 m/s² — 有效抵御编队控制器的 4 m/s²
        double mag = a_rep.norm();
        if (mag > 10.0) a_rep = a_rep * (10.0 / mag);
        return a_rep;
    }

    // --- 发布障碍物可视化 Marker ---
    // 初始调用: 先清除可能的旧标记再发布
    void publishObstacleMarkersInit() {
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_marker.header.frame_id = "odom";
        clear_marker.header.stamp = this->now();
        obstacle_marker_pub_->publish(clear_marker);
        publishObstacleMarkers();
    }

    // 周期调用: 只重发 ADD，不 DELETEALL（避免闪烁）
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
            m.lifetime = rclcpp::Duration::from_seconds(0); // 永久

            m.pose.position.x = obs.x;
            m.pose.position.y = obs.y;
            m.pose.position.z = obs.z;
            m.pose.orientation.w = 1.0;

            m.scale.x = obs.radius * 2.0;  // 直径
            m.scale.y = obs.radius * 2.0;
            m.scale.z = obs.height;

            // 半透明红色
            m.color.r = 1.0f;
            m.color.g = 0.2f;
            m.color.b = 0.2f;
            m.color.a = 0.6f;

            obstacle_marker_pub_->publish(m);
        }
    }

    // --- 核心动力学 (保持不变) ---
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

    // 级联PID
    Vector4d cascade_pid_control(const Vector12d& state, const Vector3d& target_pos, const Vector3d& a_rep) {
        Vector3d pos = state.segment<3>(0); 
        Vector3d vel = state.segment<3>(3);
        Vector3d euler = state.segment<3>(6); 
        Vector3d omega = state.segment<3>(9);
        
        Vector3d vel_des = Vector3d(1.5, 1.5, 1.5).cwiseProduct(target_pos - pos);
        vel_des = vel_des.cwiseMin(1.0).cwiseMax(-1.0);
        
        // 【核心】：将排斥加速度 a_rep 直接叠加到期望加速度上！
        Vector3d acc_des = Vector3d(2.0, 2.0, 3.0).cwiseProduct(vel_des - vel) + a_rep;
        
        // --- 限制最大逃逸加速度，防止飞机为了避障而翻车 ---
        acc_des = acc_des.cwiseMin(4.0).cwiseMax(-4.0); 
        
        double thrust = std::clamp(mass_ * (g_ + acc_des(2)), 0.0, 0.6);
        double phi_des = std::clamp((1.0/g_)*(acc_des(0)*sin(target_yaw_) - acc_des(1)*cos(target_yaw_)), -0.52, 0.52);
        double theta_des = std::clamp((1.0/g_)*(acc_des(0)*cos(target_yaw_) + acc_des(1)*sin(target_yaw_)), -0.52, 0.52);
        Vector3d omega_des = Vector3d(8.0, 8.0, 3.0).cwiseProduct(Vector3d(phi_des, theta_des, target_yaw_) - euler);
        Vector3d tau = Vector3d(0.002, 0.002, 0.003).cwiseProduct(omega_des - omega);
        return Vector4d(thrust, tau(0), tau(1), tau(2));
    }

    // --- 新增：人工势场避障计算 ---
    Vector3d compute_repulsive_acc(int current_idx) {
        Vector3d a_rep(0.0, 0.0, 0.0);
        Vector3d p_i = states_[current_idx].segment<3>(0);
        
        double d_safe = 0.6; // 安全防撞半径：60cm
        double k_rep = 3.0;  // 势场排斥增益
        
        for (int j = 0; j < num_drones_; ++j) {
            if (current_idx == j) continue;
            
            Vector3d p_j = states_[j].segment<3>(0);
            Vector3d dir = p_i - p_j;
            double dist = dir.norm();
            
            // 只有当距离小于安全半径时，才触发排斥力
            if (dist < d_safe && dist > 0.01) {
                double force_mag = k_rep * (1.0 / dist - 1.0 / d_safe) * (1.0 / (dist * dist));
                a_rep += force_mag * (dir / dist);
            }
        }
        return a_rep; // 返回三维排斥加速度
    }

    // ================= 数值算法集 =================
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

    // ================= 仿真逻辑 =================

    void physicsStepLoop() {
        step_count_++;
        sim_time_ns_ += static_cast<uint64_t>(h_step_ * 1e9);
        double t_sec = sim_time_ns_ / 1e9;
        rclcpp::Time current_sim_time(sim_time_ns_);

        // --- 先发布 /clock ---
        // RViz/RSP 通过 use_sim_time:=true 订阅此 topic, 必须先同步时钟再处理 TF
        auto clock_msg = rosgraph_msgs::msg::Clock();
        clock_msg.clock.sec = sim_time_ns_ / 1000000000;
        clock_msg.clock.nanosec = sim_time_ns_ % 1000000000;
        clock_publisher_->publish(clock_msg);

        // ==========================================
        // 集群编队艺术：呼吸圆环 (Breathing Circle)
        // ==========================================
        double base_radius = 1.1;       // 基础半径
        double shrink_amplitude = 0.6;  // 振幅 (范围 0.5m ~ 1.7m, 不穿越外侧柱)
        double radial_freq = 0.4;       // 半径缩放的频率 (控制呼吸快慢)
        double angular_freq = 0.8;      // 整体公转的频率 (控制旋转快慢)

        // 计算当前时刻的全体统一半径 (从 2.0m 收缩到 0.4m，再发散回 2.0m)
        double current_radius = base_radius + shrink_amplitude * cos(radial_freq * t_sec);

        for (int i = 0; i < num_drones_; ++i) {
            // 每架飞机在环上的固有相位
            double phase_offset = i * (2.0 * M_PI / num_drones_);
            
            // 当前时刻的动态旋转角 (固有相位 + 随时间推移的公转)
            double current_angle = phase_offset + angular_freq * t_sec;

            // X 和 Y 极坐标转化为笛卡尔坐标
            target_poses_[i](0) = current_radius * cos(current_angle);
            target_poses_[i](1) = current_radius * sin(current_angle);
            
            // Z 轴高度加入交错的波浪起伏，增强 3D 视觉张力
            target_poses_[i](2) = 1.5 + 0.2 * sin(1.0 * t_sec + phase_offset);

            geometry_msgs::msg::PoseStamped target_msg;
            target_msg.header.stamp = current_sim_time;
            target_msg.header.frame_id = "odom";
            target_msg.pose.position.x = target_poses_[i](0);
            target_msg.pose.position.y = target_poses_[i](1);
            target_msg.pose.position.z = target_poses_[i](2);
            target_pubs_[i]->publish(target_msg);

            // 计算势场排斥力 (机间避障 + 障碍物避障)
            Vector3d inter_drone_rep = compute_repulsive_acc(i);
            Vector3d pos_i = states_[i].segment<3>(0);
            Vector3d vel_i = states_[i].segment<3>(3);
            Vector3d obstacle_rep    = computeObstacleRepulsion(pos_i, vel_i);
            Vector3d total_repulsion = inter_drone_rep + obstacle_rep;

            // 将排斥力注入到底层控制计算中
            Vector4d u = cascade_pid_control(states_[i], target_poses_[i], total_repulsion);

            // 数值积分求解 (统一调度, 无分支开销)
            auto t0 = std::chrono::steady_clock::now();
            states_[i] = solver_fn_(states_[i], u, h_step_);
            auto t1 = std::chrono::steady_clock::now();
            if (experiment_enabled_) {
                safety_monitor_->recordComputeTime(
                    std::chrono::duration<double, std::nano>(t1 - t0).count());
            }

            // ---- 区间可达集检查 ----
            if (reachability_enabled_ && reachability_) {
                IntervalBounds bounds = reachability_->computeStepInterval(
                    solver_fn_, states_[i], u, h_step_);
                double min_d;
                bool overlap = reachability_->checkObstacleOverlap(bounds, min_d);
                if (overlap) {
                    safety_monitor_->logViolation(
                        t_sec, i, SafetyCondition::COLLISION, min_d, 0.0,
                        "reachable_set_overlap");
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                        "Reachability: drone %d reachable set overlaps obstacle at t=%.2f (min_d=%.4f)",
                        i, t_sec, min_d);
                }
            }

            // 发布状态 (实验 headless 模式下跳过)
            if (!headless_) {
                publishState(i, current_sim_time);
            }
        }

        // ---- 安全监视器评估 ----
        safety_monitor_->evaluateStep(t_sec, states_, target_poses_, obstacles_,
                                      mass_, g_, I_matrix_);

        // ---- 在线安全控制器 (每 5 步检查一次) ----
        if (safety_controller_ && step_count_ % 5 == 0) {
            auto scores = safety_monitor_->getScores();
            SafetyAction action = safety_controller_->evaluate(scores.overall_score());
            if (action != SafetyAction::NONE) {
                applySafetyAction(action);
            }
        }

        // ---- 实验模式: 到达指定时长自动关机 ----
        if (experiment_enabled_ && t_sec >= experiment_duration_) {
            safety_monitor_->writeExperimentMetrics(
                solverTypeToString(solver_type_enum_), h_step_,
                num_drones_, enable_obstacles_, experiment_duration_);
            RCLCPP_INFO(this->get_logger(),
                "Experiment complete. Metrics written. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // ---- 发布 (headless 模式下跳过) ----
        if (!headless_) {
            if (!obstacles_.empty() && step_count_ % 10 == 0) {
                publishObstacleMarkers();
            }
        }
    }

    void publishState(int i, const rclcpp::Time & t_now) {
        Quaterniond q = AngleAxisd(states_[i](8), Vector3d::UnitZ()) * AngleAxisd(states_[i](7), Vector3d::UnitY()) * AngleAxisd(states_[i](6), Vector3d::UnitX());
        std::string child_frame = "cf" + std::to_string(i) + "/base_link";

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = t_now; 
        t.header.frame_id = "odom"; 
        // 【修复 3】：让模型分离，不重叠
        t.child_frame_id = child_frame; 
        
        t.transform.translation.x = states_[i](0); 
        t.transform.translation.y = states_[i](1); 
        t.transform.translation.z = states_[i](2);
        t.transform.rotation.w = q.w(); t.transform.rotation.x = q.x(); t.transform.rotation.y = q.y(); t.transform.rotation.z = q.z();
        tf_broadcaster_->sendTransform(t);

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = t_now; 
        odom_msg.header.frame_id = "odom"; 
        odom_msg.child_frame_id = child_frame; 
        // 【修复 4】：使用数组变量 states_[i]
        odom_msg.pose.pose.position.x = states_[i](0);
        odom_msg.pose.pose.position.y = states_[i](1);
        odom_msg.pose.pose.position.z = states_[i](2);
        odom_msg.pose.pose.orientation.w = q.w(); odom_msg.pose.pose.orientation.x = q.x(); odom_msg.pose.pose.orientation.y = q.y(); odom_msg.pose.pose.orientation.z = q.z();

        // 填入速度 (twist) — 之前缺失
        odom_msg.twist.twist.linear.x = states_[i](3);
        odom_msg.twist.twist.linear.y = states_[i](4);
        odom_msg.twist.twist.linear.z = states_[i](5);
        odom_msg.twist.twist.angular.x = states_[i](9);
        odom_msg.twist.twist.angular.y = states_[i](10);
        odom_msg.twist.twist.angular.z = states_[i](11);

        odom_pubs_[i]->publish(odom_msg);
    }

    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string solver_type_; double h_step_, mass_, g_; Matrix3d I_matrix_, I_inv_; Vector3d I_diag_; double target_yaw_; uint64_t sim_time_ns_; int step_count_;
};

int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<OdeEngineNode>()); rclcpp::shutdown(); return 0; }
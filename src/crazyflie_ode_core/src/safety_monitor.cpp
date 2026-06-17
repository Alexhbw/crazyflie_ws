#include "crazyflie_ode_core/safety_monitor.hpp"
#include "crazyflie_ode_core/solver_registry.hpp"
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <limits>

// We need the CylinderObstacle definition — include from the main file's context
// It's defined in ode_engine_node.cpp, so we forward-declare it here and
// pass obstacles as a const ref vector.  The actual struct is:
struct CylinderObstacle {
    std::string id;
    double x, y, z;
    double radius;
    double height;
};

// ------------------------------------------------------------
// Constructor
// ------------------------------------------------------------
SafetyMonitor::SafetyMonitor(double v_max, double theta_max,
                             double energy_drift_eps,
                             const std::string& log_path,
                             bool verbose,
                             const std::string& metrics_path,
                             double warmup_time)
    : v_max_(v_max), theta_max_(theta_max),
      energy_drift_eps_(energy_drift_eps),
      log_path_(log_path), metrics_path_(metrics_path),
      verbose_(verbose), warmup_time_(warmup_time)
{
    compliant_steps_.fill(0);
    total_checks_.fill(0);
}

// ------------------------------------------------------------
// Per-step evaluation
// ------------------------------------------------------------
void SafetyMonitor::evaluateStep(
    double t_sec,
    const std::vector<Vector12d>& states,
    const std::vector<Vector3d>& target_poses,
    const std::vector<CylinderObstacle>& obstacles,
    double mass, double g,
    const Eigen::Matrix3d& I_matrix)
{
    const int drone_count = static_cast<int>(states.size());

    // Lazily init energy tracking
    if (prev_energy_.size() != static_cast<size_t>(drone_count)) {
        prev_energy_.resize(drone_count);
        for (int i = 0; i < drone_count; ++i) {
            prev_energy_[i] = computeEnergy(states[i], mass, g, I_matrix);
        }
    }

    bool in_warmup = (t_sec < warmup_time_);

    for (int i = 0; i < drone_count; ++i) {
        const auto& s = states[i];

        // ---- (a0) NaN/Inf divergence check (数值发散的最可靠信号) ----
        if (!std::isfinite(s(0)) || !std::isfinite(s(1)) || !std::isfinite(s(2)) ||
            !std::isfinite(s(3)) || !std::isfinite(s(4)) || !std::isfinite(s(5))) {
            total_checks_[0]++;
            violations_.push_back({t_sec, i, SafetyCondition::COLLISION,
                                   std::numeric_limits<double>::quiet_NaN(), 0.0,
                                   "NUMERICAL DIVERGENCE (NaN/Inf)"});
            if (verbose_) {
                fprintf(stderr, "[SAFETY] t=%.2f drone=%d NaN DIVERGENCE!\n", t_sec, i);
            }
            // 一个 NaN 就无需继续检查此无人机
            continue;
        }

        Eigen::Vector3d pos = s.segment<3>(0);
        Eigen::Vector3d vel = s.segment<3>(3);
        Eigen::Vector3d euler = s.segment<3>(6);
        double phi = euler(0), theta = euler(1);

        // ---- (a) Velocity check ----
        double v_norm = vel.norm();
        total_checks_[1]++;
        if (v_norm < v_max_) {
            compliant_steps_[1]++;
        } else {
            violations_.push_back({t_sec, i, SafetyCondition::VELOCITY,
                                   v_norm, v_max_,
                                   "|v|=" + std::to_string(v_norm).substr(0,5)});
            if (verbose_) {
                fprintf(stderr, "[SAFETY] t=%.2f drone=%d VELOCITY %.2f > %.2f\n",
                        t_sec, i, v_norm, v_max_);
            }
        }

        // Update global velocity max for experiment metrics
        if (v_norm > experiment_metrics_.max_velocity)
            experiment_metrics_.max_velocity = v_norm;

        // ---- (b) Attitude check (skip during warmup) ----
        double attitude_ang = std::sqrt(phi*phi + theta*theta);
        if (in_warmup) {
            // 预热期内跳过姿态检查
        } else {
            total_checks_[2]++;
            if (attitude_ang < theta_max_) {
                compliant_steps_[2]++;
            } else {
                violations_.push_back({t_sec, i, SafetyCondition::ATTITUDE,
                                       attitude_ang, theta_max_,
                                       "|tilt|=" + std::to_string(attitude_ang).substr(0,5)});
                if (verbose_) {
                    fprintf(stderr, "[SAFETY] t=%.2f drone=%d ATTITUDE %.2f > %.2f\n",
                            t_sec, i, attitude_ang, theta_max_);
                }
            }
        }

        if (attitude_ang > experiment_metrics_.max_attitude)
            experiment_metrics_.max_attitude = attitude_ang;

        // ---- (c) Collision check: min obstacle distance (skip warmup) ----
        double min_obs_dist = 1e9;
        if (!in_warmup) {
            for (const auto& obs : obstacles) {
                double dx = pos(0) - obs.x;
                double dy = pos(1) - obs.y;
                double d_xy = std::sqrt(dx*dx + dy*dy);
                double d_surf = d_xy - obs.radius;

                double half_h = obs.height / 2.0;
                double dz = std::abs(pos(2) - obs.z);
                double d_surf_z = dz - half_h;

                double dist = std::max(d_surf, d_surf_z);
                if (dist < min_obs_dist) min_obs_dist = dist;
            }

            total_checks_[0]++;
            if (min_obs_dist > 0.0) {
                compliant_steps_[0]++;
            } else {
                violations_.push_back({t_sec, i, SafetyCondition::COLLISION,
                                       min_obs_dist, 0.0,
                                       "penetration depth=" + std::to_string(-min_obs_dist).substr(0,5)});
                if (verbose_) {
                    fprintf(stderr, "[SAFETY] t=%.2f drone=%d COLLISION d_surf=%.3f\n",
                            t_sec, i, min_obs_dist);
                }
            }
        }

        if (min_obs_dist < experiment_metrics_.min_obstacle_distance)
            experiment_metrics_.min_obstacle_distance = min_obs_dist;

        // ---- (d) Energy drift check ----
        double E_now = computeEnergy(s, mass, g, I_matrix);
        double drift = std::abs(E_now - prev_energy_[i]);
        prev_energy_[i] = E_now;

        experiment_metrics_.total_energy_drift += drift;
        experiment_metrics_.energy_drift_samples++;

        total_checks_[3]++;
        if (drift < energy_drift_eps_) {
            compliant_steps_[3]++;
        } else {
            violations_.push_back({t_sec, i, SafetyCondition::ENERGY_DRIFT,
                                   drift, energy_drift_eps_,
                                   "dE=" + std::to_string(drift).substr(0,7)});
            // ENERGY_DRIFT is not a safety-critical violation; don't print unless verbose
        }
    }

    // ---- (e) Position tracking error ----
    for (int i = 0; i < drone_count; ++i) {
        Eigen::Vector3d pos = states[i].segment<3>(0);
        double err_sq = (pos - target_poses[i]).squaredNorm();
        experiment_metrics_.total_position_error_sq += err_sq;
        experiment_metrics_.position_error_samples++;
    }
}

// ------------------------------------------------------------
void SafetyMonitor::recordComputeTime(double dt_ns) {
    experiment_metrics_.total_compute_time_ns += dt_ns;
    experiment_metrics_.compute_samples++;
}

// ------------------------------------------------------------
void SafetyMonitor::recordPositionError(double error_sq, double vel_norm, double attitude_ang) {
    experiment_metrics_.total_position_error_sq += error_sq;
    experiment_metrics_.position_error_samples++;
    if (vel_norm > experiment_metrics_.max_velocity)
        experiment_metrics_.max_velocity = vel_norm;
    if (attitude_ang > experiment_metrics_.max_attitude)
        experiment_metrics_.max_attitude = attitude_ang;
}

// ------------------------------------------------------------
SafetyScores SafetyMonitor::getScores() const {
    SafetyScores s;
    s.collision_compliance   = total_checks_[0] > 0 ? (double)compliant_steps_[0] / total_checks_[0] : 1.0;
    s.velocity_compliance    = total_checks_[1] > 0 ? (double)compliant_steps_[1] / total_checks_[1] : 1.0;
    s.attitude_compliance    = total_checks_[2] > 0 ? (double)compliant_steps_[2] / total_checks_[2] : 1.0;
    s.energy_drift_compliance= total_checks_[3] > 0 ? (double)compliant_steps_[3] / total_checks_[3] : 1.0;
    return s;
}

// ------------------------------------------------------------
void SafetyMonitor::reset() {
    violations_.clear();
    compliant_steps_.fill(0);
    total_checks_.fill(0);
    prev_energy_.clear();
    experiment_metrics_.reset();
}

// ------------------------------------------------------------
void SafetyMonitor::writeCSV() {
    std::ofstream f(log_path_);
    if (!f.is_open()) return;

    f << "sim_time,drone_id,condition,actual_value,limit,message\n";
    for (const auto& v : violations_) {
        f << std::fixed << std::setprecision(4)
          << v.sim_time << ","
          << v.drone_id << ","
          << safetyConditionName(v.condition) << ","
          << v.actual_value << ","
          << v.limit << ","
          << v.message << "\n";
    }
    f.close();
}

// ------------------------------------------------------------
void SafetyMonitor::writeExperimentMetrics(
    const std::string& solver_name, double step_size,
    int num_drones, bool obstacles_enabled, double duration)
{
    std::ofstream f(metrics_path_);
    if (!f.is_open()) return;

    f << std::fixed << std::setprecision(6);
    f << "solver,step_size,num_drones,obstacles_enabled,sim_duration,"
      << "energy_drift_rate,min_obstacle_dist,max_velocity,max_attitude,"
      << "rms_position_error,avg_compute_ms,"
      << "collision_compliance,velocity_compliance,attitude_compliance\n";

    auto scores = getScores();
    f << solver_name << ","
      << step_size << ","
      << num_drones << ","
      << (obstacles_enabled ? "true" : "false") << ","
      << duration << ","
      << experiment_metrics_.energy_drift_rate() << ","
      << experiment_metrics_.min_obstacle_distance << ","
      << experiment_metrics_.max_velocity << ","
      << experiment_metrics_.max_attitude << ","
      << experiment_metrics_.rms_position_error() << ","
      << experiment_metrics_.avg_compute_time_ms() << ","
      << scores.collision_compliance << ","
      << scores.velocity_compliance << ","
      << scores.attitude_compliance << "\n";
    f.close();
}

// ------------------------------------------------------------
void SafetyMonitor::logViolation(double sim_time, int drone_id,
                                 SafetyCondition cond, double actual, double limit,
                                 const std::string& msg) {
    violations_.push_back({sim_time, drone_id, cond, actual, limit,
                           msg.empty() ? "" : msg});
}

// ------------------------------------------------------------
double SafetyMonitor::computeEnergy(const Vector12d& state, double mass, double g,
                                    const Eigen::Matrix3d& I_matrix) const {
    Eigen::Vector3d vel = state.segment<3>(3);
    Eigen::Vector3d omega = state.segment<3>(9);
    double z = state(2);

    double ke_trans = 0.5 * mass * vel.squaredNorm();
    double ke_rot = 0.5 * omega.dot(I_matrix * omega);
    double pe = mass * g * z;
    return ke_trans + ke_rot + pe;
}

// ================================================================
// SafetyController 实现
// ================================================================

SafetyController::SafetyController(int window_size, double dt_min,
                                   double dt_factor,
                                   double thresh_halve,
                                   double thresh_switch,
                                   double thresh_stop)
    : window_size_(window_size), dt_min_(dt_min), dt_factor_(dt_factor),
      thresh_halve_(thresh_halve), thresh_switch_(thresh_switch),
      thresh_stop_(thresh_stop), action_count_(0)
{}

SafetyAction SafetyController::evaluate(double overall_score) {
    // 指数平滑 (低通滤波，防止单步误触发)
    smoothed_score_ = alpha_ * overall_score + (1.0 - alpha_) * smoothed_score_;
    scores_.push_back(overall_score);
    if (static_cast<int>(scores_.size()) > window_size_) {
        scores_.erase(scores_.begin());
    }

    double score = smoothed_score_;

    if (score < thresh_stop_) {
        action_count_++;
        return SafetyAction::EMERGENCY_STOP;
    }
    if (score < thresh_switch_) {
        action_count_++;
        return SafetyAction::SWITCH_SOLVER;
    }
    if (score < thresh_halve_) {
        action_count_++;
        return SafetyAction::REDUCE_DT;
    }
    return SafetyAction::NONE;
}

double SafetyController::reducedStep(double current_dt) const {
    double new_dt = current_dt * dt_factor_;
    return (new_dt < dt_min_) ? dt_min_ : new_dt;
}

SolverType SafetyController::saferSolver(SolverType current) const {
    // 求解器安全层级: euler < heun < rk4 < rk45 < implicit
    switch (current) {
        case SolverType::EULER:    return SolverType::HEUN;
        case SolverType::HEUN:     return SolverType::RK4;
        case SolverType::RK4:      return SolverType::RK45;
        case SolverType::RK45:     return SolverType::IMPLICIT;
        case SolverType::IMPLICIT: return SolverType::IMPLICIT; // already safest
    }
    return SolverType::IMPLICIT; // fallback to safest
}

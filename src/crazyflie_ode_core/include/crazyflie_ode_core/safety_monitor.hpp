#ifndef CRAZYFLIE_ODE_CORE__SAFETY_MONITOR_HPP_
#define CRAZYFLIE_ODE_CORE__SAFETY_MONITOR_HPP_

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>
#include <fstream>

// 前向声明 SolverType (用于 SafetyController)
enum class SolverType : int;

// 前向声明
struct CylinderObstacle;

typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

// ---- 安全条件枚举 ----
enum class SafetyCondition {
    COLLISION,      // □(min_obstacle_distance > 0)
    VELOCITY,       // □(|v_i| < v_max)
    ATTITUDE,       // □(|θ_i| < θ_max)
    ENERGY_DRIFT    // □(|E_i - E_{i-1}| < ε)
};

inline std::string safetyConditionName(SafetyCondition c) {
    switch (c) {
        case SafetyCondition::COLLISION:   return "COLLISION";
        case SafetyCondition::VELOCITY:    return "VELOCITY";
        case SafetyCondition::ATTITUDE:    return "ATTITUDE";
        case SafetyCondition::ENERGY_DRIFT:return "ENERGY_DRIFT";
    }
    return "UNKNOWN";
}

// ---- 单次违规记录 ----
struct SafetyViolation {
    double sim_time;
    int drone_id;
    SafetyCondition condition;
    double actual_value;
    double limit;
    std::string message;
};

// ---- 安全分数 ----
struct SafetyScores {
    double collision_compliance = 1.0;
    double velocity_compliance = 1.0;
    double attitude_compliance = 1.0;
    double energy_drift_compliance = 1.0;

    double overall_score() const {
        return std::min({collision_compliance, velocity_compliance,
                         attitude_compliance, energy_drift_compliance});
    }
};

// ---- 实验指标 ----
struct ExperimentMetrics {
    double total_energy_drift = 0.0;
    int energy_drift_samples = 0;
    double min_obstacle_distance = 1e9;
    double max_velocity = 0.0;
    double max_attitude = 0.0;
    double total_position_error_sq = 0.0;
    int position_error_samples = 0;
    double total_compute_time_ns = 0.0;
    int compute_samples = 0;

    double energy_drift_rate() const {
        return energy_drift_samples > 0 ? total_energy_drift / energy_drift_samples : 0.0;
    }
    double avg_compute_time_ms() const {
        return compute_samples > 0 ? total_compute_time_ns / compute_samples / 1e6 : 0.0;
    }
    double rms_position_error() const {
        return position_error_samples > 0 ?
            std::sqrt(total_position_error_sq / position_error_samples) : 0.0;
    }

    void reset() {
        total_energy_drift = 0.0; energy_drift_samples = 0;
        min_obstacle_distance = 1e9; max_velocity = 0.0; max_attitude = 0.0;
        total_position_error_sq = 0.0; position_error_samples = 0;
        total_compute_time_ns = 0.0; compute_samples = 0;
    }
};

// ---- 安全监视器 ----
class SafetyMonitor {
public:
    SafetyMonitor(double v_max = 2.5, double theta_max = 0.8,
                  double energy_drift_eps = 0.01,
                  const std::string& log_path = "safety_log.csv",
                  bool verbose = false,
                  const std::string& metrics_path = "experiment_metrics.csv",
                  double warmup_time = 2.0);

    // 每步评估：检查所有安全条件
    void evaluateStep(double t_sec,
                      const std::vector<Vector12d>& states,
                      const std::vector<Vector3d>& target_poses,
                      const std::vector<CylinderObstacle>& obstacles,
                      double mass, double g,
                      const Eigen::Matrix3d& I_matrix);

    // 实验指标: 注册单步计算时间
    void recordComputeTime(double dt_ns);

    // 实验指标: 注册位置误差和全局极值
    void recordPositionError(double error_sq, double vel_norm, double attitude_ang);

    // 获取分数
    SafetyScores getScores() const;

    // 获取实验指标（const 引用）
    const ExperimentMetrics& metrics() const { return experiment_metrics_; }

    // 重置
    void reset();

    // 写入违规 CSV
    void writeCSV();

    // 写入实验指标 CSV
    void writeExperimentMetrics(const std::string& solver_name, double step_size,
                                int num_drones, bool obstacles_enabled, double duration);

    // 设置日志路径
    void setLogPath(const std::string& p) { log_path_ = p; }
    void setMetricsPath(const std::string& p) { metrics_path_ = p; }

    // 显式记录一次违规（供外部调用，如 interval reachability）
    void logViolation(double sim_time, int drone_id,
                      SafetyCondition cond, double actual, double limit,
                      const std::string& msg = "");

private:
    double v_max_;
    double theta_max_;
    double energy_drift_eps_;
    std::string log_path_;
    std::string metrics_path_;
    bool verbose_;
    double warmup_time_;

    std::vector<SafetyViolation> violations_;
    std::array<int, 4> compliant_steps_;  // per-condition
    std::array<int, 4> total_checks_;      // per-condition
    std::vector<double> prev_energy_;       // per-drone, for drift tracking

    ExperimentMetrics experiment_metrics_;

    double computeEnergy(const Vector12d& state, double mass, double g,
                         const Eigen::Matrix3d& I_matrix) const;
};

// ---- 在线安全控制器 ----
// 根据安全监视器输出的分数，自动触发递进式保护动作
// 三级递进: REDUCE_DT (减半步长) → SWITCH_SOLVER (切换更保守求解器) → EMERGENCY_STOP (关机)

enum class SafetyAction { NONE, REDUCE_DT, SWITCH_SOLVER, EMERGENCY_STOP };

inline const char* safetyActionName(SafetyAction a) {
    switch (a) {
        case SafetyAction::NONE:           return "NONE";
        case SafetyAction::REDUCE_DT:      return "REDUCE_DT";
        case SafetyAction::SWITCH_SOLVER:  return "SWITCH_SOLVER";
        case SafetyAction::EMERGENCY_STOP: return "EMERGENCY_STOP";
    }
    return "UNKNOWN";
}

class SafetyController {
public:
    SafetyController(int window_size = 100, double dt_min = 0.0001,
                     double dt_factor = 0.5,
                     double thresh_halve = 0.7,
                     double thresh_switch = 0.3,
                     double thresh_stop = 0.05);

    // 每步喂入安全分数，返回是否需要采取行动
    SafetyAction evaluate(double overall_score);

    // 计算新的步长 (减半)
    double reducedStep(double current_dt) const;

    // 计算下一个更安全的求解器
    SolverType saferSolver(SolverType current) const;

    // 获取当前状态
    int actionCount() const { return action_count_; }
    void resetActions() { action_count_ = 0; scores_.clear(); }

private:
    int window_size_;
    double dt_min_;
    double dt_factor_;
    double thresh_halve_;
    double thresh_switch_;
    double thresh_stop_;
    int action_count_;
    std::vector<double> scores_;       // 滑动窗口内的综合分数
    double smoothed_score_ = 1.0;      // 指数平滑分数
    double alpha_ = 0.05;              // 平滑系数 (慢速响应噪声)
};

#endif  // CRAZYFLIE_ODE_CORE__SAFETY_MONITOR_HPP_

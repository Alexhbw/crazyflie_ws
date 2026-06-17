#ifndef CRAZYFLIE_ODE_CORE__INTERVAL_REACHABILITY_HPP_
#define CRAZYFLIE_ODE_CORE__INTERVAL_REACHABILITY_HPP_

#include <Eigen/Dense>
#include <functional>
#include <vector>
#include <string>

// 前向声明
struct CylinderObstacle;

typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 4, 1> Vector4d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

// ---- 求解器函数签名 ----
using SolverFn = std::function<Vector12d(const Vector12d&, const Vector4d&, double)>;

// ---- 区间包围盒 ----
struct IntervalBounds {
    Vector12d lower;
    Vector12d upper;
    Vector12d center;

    Vector3d posLower() const { return lower.segment<3>(0); }
    Vector3d posUpper() const { return upper.segment<3>(0); }

    // 位置区间对角长度 (可达集大小度量)
    double posSpan() const {
        return (posUpper() - posLower()).norm();
    }
};

// ---- 区间可达集分析 ----
class IntervalReachability {
public:
    enum class Method { SAMPLING, JACOBIAN };

    IntervalReachability(double epsilon,
                         const std::vector<CylinderObstacle>& obstacles,
                         Method method = Method::SAMPLING);

    // 核心: 一步正向区间传播
    IntervalBounds computeStepInterval(SolverFn solver_fn,
                                       const Vector12d& x,
                                       const Vector4d& u,
                                       double h);

    // 保守检测: 可达集是否与任何障碍物重叠?
    // 返回 true 如果重叠, 并设置 min_dist 为最近距离
    bool checkObstacleOverlap(const IntervalBounds& bounds, double& min_dist) const;

    // 仅计算最近距离 (不返回 bool)
    double minObstacleDistance(const IntervalBounds& bounds) const;

    // 参数访问
    void setEpsilon(double eps) { epsilon_ = eps; }
    double epsilon() const { return epsilon_; }
    void setMethod(Method m) { method_ = m; }

private:
    double epsilon_;
    const std::vector<CylinderObstacle>& obstacles_;
    Method method_;

    // 采样法: 每维 ±epsilon 扰动, 共 25 次求解器调用
    IntervalBounds computeSampling(SolverFn solver_fn,
                                   const Vector12d& x,
                                   const Vector4d& u, double h);

    // Jacobian 法: 有限差分求 Jacobian, 线性传播
    IntervalBounds computeJacobian(SolverFn solver_fn,
                                   const Vector12d& x,
                                   const Vector4d& u, double h);
};

#endif  // CRAZYFLIE_ODE_CORE__INTERVAL_REACHABILITY_HPP_

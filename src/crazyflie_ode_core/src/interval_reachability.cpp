#include "crazyflie_ode_core/interval_reachability.hpp"
#include <cmath>
#include <algorithm>

// CylinderObstacle 定义 (与 ode_engine_node.cpp 保持一致)
struct CylinderObstacle {
    std::string id;
    double x, y, z;
    double radius;
    double height;
};

// ------------------------------------------------------------
IntervalReachability::IntervalReachability(
    double epsilon,
    const std::vector<CylinderObstacle>& obstacles,
    Method method)
    : epsilon_(epsilon), obstacles_(obstacles), method_(method)
{}

// ------------------------------------------------------------
IntervalBounds IntervalReachability::computeStepInterval(
    SolverFn solver_fn, const Vector12d& x, const Vector4d& u, double h)
{
    switch (method_) {
        case Method::JACOBIAN:
            return computeJacobian(solver_fn, x, u, h);
        case Method::SAMPLING:
        default:
            return computeSampling(solver_fn, x, u, h);
    }
}

// ------------------------------------------------------------
// 采样法: 对 12 维状态逐维 ±ε 扰动, 运行求解器, 取 min/max
// 共 25 次求解器调用 = 1 中心 + 12×2 扰动
// ------------------------------------------------------------
IntervalBounds IntervalReachability::computeSampling(
    SolverFn solver_fn, const Vector12d& x, const Vector4d& u, double h)
{
    IntervalBounds bounds;
    bounds.center = solver_fn(x, u, h);
    bounds.lower = bounds.center;
    bounds.upper = bounds.center;

    for (int j = 0; j < 12; ++j) {
        // +epsilon
        Vector12d x_plus = x;
        x_plus(j) += epsilon_;
        Vector12d xp = solver_fn(x_plus, u, h);

        // -epsilon
        Vector12d x_minus = x;
        x_minus(j) -= epsilon_;
        Vector12d xm = solver_fn(x_minus, u, h);

        for (int k = 0; k < 12; ++k) {
            bounds.lower(k) = std::min({bounds.lower(k), xp(k), xm(k)});
            bounds.upper(k) = std::max({bounds.upper(k), xp(k), xm(k)});
        }
    }

    return bounds;
}

// ------------------------------------------------------------
// Jacobian 法: 有限差分求 ∂(solver)/∂x, 线性传播区间
// 共 13 次求解器调用 = 1 中心 + 12 次 (中心差分)
//   区间: [x_next - |J|*ε, x_next + |J|*ε]
// ------------------------------------------------------------
IntervalBounds IntervalReachability::computeJacobian(
    SolverFn solver_fn, const Vector12d& x, const Vector4d& u, double h)
{
    IntervalBounds bounds;
    bounds.center = solver_fn(x, u, h);

    // 构建 Jacobian 的绝对值矩阵 |J|
    Eigen::Matrix<double, 12, 12> J_abs = Eigen::Matrix<double, 12, 12>::Zero();

    for (int j = 0; j < 12; ++j) {
        Vector12d x_plus = x;
        x_plus(j) += epsilon_;
        Vector12d xp = solver_fn(x_plus, u, h);

        Vector12d x_minus = x;
        x_minus(j) -= epsilon_;
        Vector12d xm = solver_fn(x_minus, u, h);

        for (int i = 0; i < 12; ++i) {
            double J_ij = (xp(i) - xm(i)) / (2.0 * epsilon_);
            J_abs(i, j) = std::abs(J_ij);
        }
    }

    // 区间传播: delta = |J| * ε * 1_{12}
    Vector12d delta = J_abs * Vector12d::Constant(epsilon_);
    bounds.lower = bounds.center - delta;
    bounds.upper = bounds.center + delta;

    return bounds;
}

// ------------------------------------------------------------
// AABB-圆柱 碰撞检测 (保守)
// 计算三维包围盒到圆柱表面的最近距离
// ------------------------------------------------------------
bool IntervalReachability::checkObstacleOverlap(
    const IntervalBounds& bounds, double& min_dist) const
{
    min_dist = 1e9;

    Vector3d p_low = bounds.posLower();
    Vector3d p_high = bounds.posUpper();

    for (const auto& obs : obstacles_) {
        // ---- XY 平面: AABB 到圆心的最近距离 ----
        double dx = 0.0;
        if (obs.x < p_low(0))      dx = p_low(0) - obs.x;
        else if (obs.x > p_high(0)) dx = obs.x - p_high(0);

        double dy = 0.0;
        if (obs.y < p_low(1))      dy = p_low(1) - obs.y;
        else if (obs.y > p_high(1)) dy = obs.y - p_high(1);

        double d_xy = std::sqrt(dx*dx + dy*dy);

        // ---- Z 轴: AABB 到圆柱端盖的最近距离 ----
        double half_h = obs.height / 2.0;
        double obs_z_low = obs.z - half_h;
        double obs_z_high = obs.z + half_h;

        double dz = 0.0;
        if (obs_z_high < p_low(2))  dz = p_low(2) - obs_z_high;
        else if (obs_z_low > p_high(2)) dz = obs_z_low - p_high(2);

        // 3D 距离
        double dist_3d = std::sqrt(d_xy*d_xy + dz*dz);
        if (dist_3d < min_dist) min_dist = dist_3d;

        // ---- 重叠判断 ----
        // 圆柱内: d_xy < radius AND |dz| < half_height
        double d_surf_xy = d_xy - obs.radius;
        double d_surf_z  = dz;
        double d_surf = std::max(d_surf_xy, d_surf_z);

        if (d_surf < 0.0) {
            // 可达集与障碍物重叠!
            min_dist = d_surf;   // 负值表示穿透深度
            return true;
        }
    }

    return false;  // 无重叠
}

// ------------------------------------------------------------
double IntervalReachability::minObstacleDistance(
    const IntervalBounds& bounds) const
{
    double d;
    checkObstacleOverlap(bounds, d);
    return d;
}

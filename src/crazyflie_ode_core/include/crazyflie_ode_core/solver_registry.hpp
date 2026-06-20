#ifndef CRAZYFLIE_ODE_CORE__SOLVER_REGISTRY_HPP_
#define CRAZYFLIE_ODE_CORE__SOLVER_REGISTRY_HPP_

#include <Eigen/Dense>
#include <functional>
#include <string>
#include <stdexcept>

using namespace Eigen;

typedef Matrix<double, 12, 1> Vector12d;
typedef Matrix<double, 4, 1> Vector4d;

// ---- 求解器枚举 ----
enum class SolverType {
    EULER,
    HEUN,
    RK4,
    RK45,
    IMPLICIT
};

inline SolverType solverTypeFromString(const std::string& name) {
    if (name == "euler")    return SolverType::EULER;
    if (name == "heun")     return SolverType::HEUN;
    if (name == "rk4")      return SolverType::RK4;
    if (name == "rk45")     return SolverType::RK45;
    if (name == "implicit") return SolverType::IMPLICIT;
    throw std::runtime_error("Unknown solver type: " + name);
}

inline std::string solverTypeToString(SolverType t) {
    switch (t) {
        case SolverType::EULER:    return "euler";
        case SolverType::HEUN:     return "heun";
        case SolverType::RK4:      return "rk4";
        case SolverType::RK45:     return "rk45";
        case SolverType::IMPLICIT: return "implicit";
    }
    return "unknown";
}

// ---- 统一求解器函数签名 ----
using SolverFn = std::function<Vector12d(const Vector12d&, const Vector4d&, double)>;

// ---- 动力学参数包（解耦类成员依赖） ----
struct DynamicsParams {
    double mass;
    double g;
    Matrix3d I_matrix;
    Matrix3d I_inv;
    Vector3d I_diag;
};

#endif  // CRAZYFLIE_ODE_CORE__SOLVER_REGISTRY_HPP_

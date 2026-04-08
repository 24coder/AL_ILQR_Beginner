#pragma once

#include <Eigen/Dense>

namespace my_al_ilqr {

// 通用动态大小向量类型。
//
// 设计目的：
// - 在动力学、代价、约束、优化器等模块之间统一线性代数接口；
// - 降低模板噪声，提升教学与实验代码可读性。
using Vector = Eigen::VectorXd;

// 通用动态大小矩阵类型。
//
// 典型用途：
// - 状态/控制权重矩阵（Q, R, Qf）；
// - 线性化雅可比、二次近似 Hessian/增益矩阵等。
using Matrix = Eigen::MatrixXd;

}  // namespace my_al_ilqr

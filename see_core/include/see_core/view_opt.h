// Copyright (c) 2020 Oxford Estimation, Search, and Planning Research Group
#pragma once

#ifndef VIEW_OPT_H
#define VIEW_OPT_H

#include <Eigen/Dense>
#include <nlopt.hpp>
#include <vector>

using namespace Eigen;

//! Objection function for Hemispherical View Optimisation
/*!
  \param n Number of variables
  \param p Function variables
  \param grad Partial derivatives of objective function
  \param constants Function constants
  \return Calculated function value
*/
inline double ViewOptHemiObjFunction(unsigned n, const double *p, double *grad,
                                     void *constants) {
  if (grad) {
    grad[0] = -1;
    grad[1] = 0;
    grad[2] = 0;
    grad[3] = 0;
  }
  return -p[0];
}

//! Objection function for Full Sphere View Optimisation
/*!
  \param n Number of variables
  \param p Function variables
  \param grad Partial derivatives of objective function
  \param constants Function constants
  \return Calculated function value
*/
inline double ViewOptObjFunction(unsigned n, const double *p, double *grad,
                                 void *constants) {
  if (grad) {
    grad[0] = 1;
    grad[1] = 0;
    grad[2] = 0;
    grad[3] = 0;
  }
  return p[0];
}

//! Quadratic constraint function for Hemispherical View Optimisation
/*!
  \param n Number of variables
  \param p Function variables
  \param grad Partial derivatives of objective function
  \param constants Function constants
  \return Calculated function value
*/
inline double ViewOptHemiQuadConst(unsigned n, const double *p, double *grad,
                                   void *constants) {
  if (grad) {
    grad[0] = -1;
    grad[1] = 2 * p[1];
    grad[2] = 2 * p[2];
    grad[3] = 2 * p[3];
  }
  return -p[0] + pow(p[1], 2) + pow(p[2], 2) + pow(p[3], 2);
}

//! Quadratic constraint function for Full Sphere View Optimisation
/*!
  \param n Number of variables
  \param p Function variables
  \param grad Partial derivatives of objective function
  \param constants Function constants
  \return Calculated function value
*/
inline double ViewOptQuadConst(unsigned n, const double *p, double *grad,
                               void *constants) {
  if (grad) {
    grad[0] = 1;
    grad[1] = -2 * p[1];
    grad[2] = -2 * p[2];
    grad[3] = -2 * p[3];
  }
  return p[0] - pow(p[1], 2) - pow(p[2], 2) - pow(p[3], 2);
}

//! Pointwise constraint function for Hemispherical View Optimisation
/*!
  \param n Number of variables
  \param p Function variables
  \param grad Partial derivatives of objective function
  \param constants Pointwise constraints
  \return Calculated function value
*/
inline double ViewOptHemiPointConst(unsigned n, const double *p, double *grad,
                                    void *constants) {
  float *C = static_cast<float *>(constants);
  if (grad) {
    grad[0] = 1;
    grad[1] = -C[0];
    grad[2] = -C[1];
    grad[3] = -C[2];
  }
  return -C[0] * p[1] - C[1] * p[2] - C[2] * p[3] + p[0];
}

//! Pointwise constraint function for Full Sphere View Optimisation
/*!
  \param n Number of variables
  \param p Function variables
  \param grad Partial derivatives of objective function
  \param constants Pointwise constraints
  \return Calculated function value
*/
inline double ViewOptPointConst(unsigned n, const double *p, double *grad,
                                void *constants) {
  float *C = static_cast<float *>(constants);
  if (grad) {
    grad[0] = -1;
    grad[1] = C[0];
    grad[2] = C[1];
    grad[3] = C[2];
  }
  return C[0] * p[1] + C[1] * p[2] + C[2] * p[3] - p[0];
}

//! Hemispherical View Optimisation
/*!
  \param vobs View orientation from which the frontier point was observed
  \param P Set of occluding points
  \param vopt Optimal view orientation (relative to the frontier point)
  \param result Optimisation status flag
  \return Was Hemispherical View Optimisation successful
*/
inline bool ViewOptHemi(Vector3f vobs, MatrixXf &P, Vector3f &vopt,
                        int &result) {
  double minf, eps = 1e-3;
  std::vector<double> lb, ub, v;
  std::shared_ptr<float[]> C(new float[P.rows() * P.cols()]);

  Map<MatrixXf>(C.get(), P.rows(), P.cols()) = P;
  nlopt::opt opt(nlopt::LD_SLSQP, 4);

  opt.add_inequality_constraint(ViewOptHemiQuadConst, NULL, eps);
  for (int i = 0; i < P.cols(); i++) {
    void *C_i = static_cast<void *>(C.get() + (i * 4));
    opt.add_inequality_constraint(ViewOptHemiPointConst, C_i, eps);
  }

  lb = {0, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL};
  ub = {1, HUGE_VAL, HUGE_VAL, HUGE_VAL};
  v = {0, vobs(0), vobs(1), vobs(2)};

  opt.set_maxtime(0.01);
  opt.set_xtol_abs(eps);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(ViewOptHemiObjFunction, NULL);

  try {
    if (!result) {
      result = opt.optimize(v, minf);
    }
    if (v[0] > eps) {
      vopt << v[1] / sqrt(v[0]), v[2] / sqrt(v[0]), v[3] / sqrt(v[0]);
      vopt.normalize();
      return true;
    }
  } catch (std::exception &e) {
  }
  return false;
}

//! Full Sphere View Optimisation
/*!
  \param vobs View orientation from which the frontier point was observed
  \param P Set of occluding points
  \param vopt Optimal view orientation (relative to the frontier point)
  \param result Optimisation status flag
  \return Was Full Sphere View Optimisation successful
*/
inline bool ViewOpt(Vector3f vobs, MatrixXf &P, Vector3f &vopt, int &result) {
  double minf, eps = 1e-3;
  std::vector<double> lb, ub, v;
  std::shared_ptr<float[]> C(new float[P.rows() * P.cols()]);

  Map<MatrixXf>(C.get(), P.rows(), P.cols()) = P;
  nlopt::opt opt(nlopt::LD_SLSQP, 4);

  opt.add_inequality_constraint(ViewOptQuadConst, NULL, eps);
  for (int i = 0; i < P.cols(); i++) {
    void *C_i = static_cast<void *>(C.get() + (i * 4));
    opt.add_inequality_constraint(ViewOptPointConst, C_i, eps);
  }

  lb = {0, -HUGE_VAL, -HUGE_VAL, -HUGE_VAL};
  ub = {1, HUGE_VAL, HUGE_VAL, HUGE_VAL};
  v = {1, -vobs(0), -vobs(1), -vobs(2)};

  opt.set_maxtime(0.01);
  opt.set_xtol_abs(eps);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_min_objective(ViewOptObjFunction, NULL);

  try {
    result = opt.optimize(v, minf);
    if (v[0] > eps) {
      vopt << -v[1] / sqrt(v[0]), -v[2] / sqrt(v[0]), -v[3] / sqrt(v[0]);
      vopt.normalize();
      return true;
    } else {
      result = 0;
    }
  } catch (std::exception &e) {
  }
  return false;
}

#endif // VIEW_OPT_H

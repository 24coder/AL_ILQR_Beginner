#pragma once

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "core/types.hpp"

namespace my_al_ilqr::debug {

inline std::ostream& DebugStream() {
  return std::cerr;
}

inline std::string Separator(int width = 88, char ch = '=') {
  return std::string(width, ch);
}

inline std::string FormatScalar(double value, int width = 12, int precision = 6) {
  std::ostringstream oss;
  oss << std::setw(width) << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

inline std::string FormatVectorRow(const Vector& vec, int width = 12, int precision = 6) {
  std::ostringstream oss;
  for (int i = 0; i < vec.size(); ++i) {
    oss << std::setw(width) << std::fixed << std::setprecision(precision) << vec(i);
  }
  return oss.str();
}

inline std::string FormatMatrix(const Matrix& mat, int width = 12, int precision = 6) {
  std::ostringstream oss;
  for (int i = 0; i < mat.rows(); ++i) {
    oss << "    row[" << std::setw(2) << i << "]";
    for (int j = 0; j < mat.cols(); ++j) {
      oss << std::setw(width) << std::fixed << std::setprecision(precision) << mat(i, j);
    }
    if (i + 1 < mat.rows()) {
      oss << '\n';
    }
  }
  return oss.str();
}

inline void PrintMatrix(const std::string& name, const Matrix& mat, int width = 12, int precision = 6) {
  DebugStream() << name << " [" << mat.rows() << " x " << mat.cols() << "]\n"
                << FormatMatrix(mat, width, precision) << "\n";
}

inline void PrintVector(const std::string& name, const Vector& vec, int width = 12, int precision = 6) {
  DebugStream() << name << " [" << vec.size() << "]\n"
                << "    " << FormatVectorRow(vec, width, precision) << "\n";
}

inline void PrintHeader(const std::string& title) {
  DebugStream() << "\n" << Separator() << "\n";
  DebugStream() << title << "\n";
  DebugStream() << Separator() << "\n";
}

inline void PrintNamedScalar(const std::string& name, double value, int width = 12, int precision = 6) {
  DebugStream() << std::left << std::setw(24) << name << " = "
                << std::right << std::setw(width) << std::fixed << std::setprecision(precision) << value
                << "\n";
}

inline void PrintPhase7SetupSnapshot(const Matrix& Q,
                                     const Matrix& R,
                                     const Matrix& Qf,
                                     const Vector& x_ref,
                                     const Vector& u_ref,
                                     const Vector& x0,
                                     const std::vector<Vector>& initial_controls) {
  PrintHeader("[DEBUG] Phase7 variable snapshot");
  PrintMatrix("Q", Q);
  PrintMatrix("R", R);
  PrintMatrix("Qf", Qf);
  PrintVector("x_ref", x_ref);
  PrintVector("u_ref", u_ref);
  PrintVector("x0", x0);

  DebugStream() << "initial_controls size = " << initial_controls.size() << "\n";
  const int preview = std::min<int>(3, initial_controls.size());
  for (int k = 0; k < preview; ++k) {
    PrintVector("initial_controls[" + std::to_string(k) + "]", initial_controls[k]);
  }
  if (static_cast<int>(initial_controls.size()) > preview) {
    DebugStream() << "    ...\n";
    PrintVector("initial_controls[last]", initial_controls.back());
  }
  DebugStream() << Separator() << "\n";
  DebugStream() << std::flush;
}

}  // namespace my_al_ilqr::debug

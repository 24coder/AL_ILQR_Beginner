#include <filesystem>
#include <iostream>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "visualization/al_ilqr_log_writer.hpp"
#include "visualization/obstacle_prediction_writer.hpp"
#include "visualization/trajectory_writer.hpp"

int main() {
  // Phase 11: dynamic obstacle demo.
  // This demo also exports trajectory evolution and inner iLQR cost history CSVs
  // to visualize how AL-iLQR converges from the initial guess to the final trajectory.
  using my_al_ilqr::ALILQRSolver;

  const auto scenario = my_al_ilqr::CreateDynamicObstacleDemoScenario();
  const auto initial_trajectory =
      scenario.base_problem->Rollout(scenario.initial_state, scenario.initial_controls);

  ALILQRSolver solver(*scenario.constrained_problem, scenario.solver_options);
  const auto result = solver.Solve(scenario.initial_state, scenario.initial_controls);

  const std::filesystem::path initial_csv = "build/phase11_dynamic_initial_trajectory.csv";
  const std::filesystem::path optimized_csv = "build/phase11_dynamic_optimized_trajectory.csv";
  const std::filesystem::path obstacle_csv = "build/phase11_dynamic_obstacle_prediction.csv";
  const std::filesystem::path outer_log_csv = "build/phase11_dynamic_outer_log.csv";
  const std::filesystem::path inner_cost_csv = "build/phase11_dynamic_inner_cost_history.csv";
  const std::filesystem::path traj_evolution_csv =
      "build/phase11_dynamic_trajectory_evolution.csv";

  my_al_ilqr::WriteTrajectoryCsv(initial_csv, initial_trajectory);
  my_al_ilqr::WriteTrajectoryCsv(optimized_csv, result.trajectory);
  my_al_ilqr::WriteObstaclePredictionCsv(
      obstacle_csv, scenario.obstacle_centers, scenario.obstacle_radius);
  my_al_ilqr::WriteALILQROuterLogCsv(outer_log_csv, result.outer_logs);
  my_al_ilqr::WriteALILQRInnerCostHistoryCsv(inner_cost_csv, result.outer_logs);
  my_al_ilqr::WriteALILQRTrajectoryEvolutionCsv(traj_evolution_csv, result.outer_logs);

  std::cout << "Phase 11 dynamic obstacle demo\n";
  std::cout << "initial trajectory csv   = " << initial_csv << "\n";
  std::cout << "optimized trajectory csv = " << optimized_csv << "\n";
  std::cout << "obstacle prediction csv  = " << obstacle_csv << "\n";
  std::cout << "outer log csv            = " << outer_log_csv << "\n";
  std::cout << "inner cost history csv   = " << inner_cost_csv << "\n";
  std::cout << "trajectory evolution csv = " << traj_evolution_csv << "\n";
  std::cout << "converged                = " << (result.converged ? "true" : "false") << "\n";
  std::cout << "final violation          = " << result.final_violation << "\n";
  std::cout << "best violation           = " << result.best_violation << "\n";
  std::cout << "terminal state           = "
            << result.trajectory.State(scenario.base_problem->Horizon()).transpose() << "\n";

  // Print per-outer-iteration summary to show the optimization process.
  std::cout << "\nAL-iLQR outer iteration summary:\n";
  std::cout << "  iter  inner_iters  base_cost       violation       penalty_updated\n";
  for (const auto& log : result.outer_logs) {
    std::cout << "  " << log.outer_iteration
              << "     " << log.inner_iterations
              << "            " << log.base_cost
              << "  " << log.max_violation
              << "  " << (log.penalty_updated ? "yes" : "no") << "\n";
  }

  return 0;
}

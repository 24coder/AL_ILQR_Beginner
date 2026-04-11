#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

#include "al/al_ilqr_solver.hpp"
#include "autodrive/demo_scenario.hpp"
#include "visualization/al_ilqr_log_writer.hpp"
#include "visualization/obstacle_prediction_writer.hpp"
#include "visualization/trajectory_writer.hpp"

namespace {

std::string QuoteShellArg(const std::filesystem::path& path) {
  const std::string raw = path.string();
  std::string quoted;
  quoted.reserve(raw.size() + 2);
  quoted.push_back('"');
  for (const char ch : raw) {
    if (ch == '"' || ch == '\\') {
      quoted.push_back('\\');
    }
    quoted.push_back(ch);
  }
  quoted.push_back('"');
  return quoted;
}

void TryGeneratePhase11Gif(const std::filesystem::path& output_dir) {
  const std::filesystem::path repo_root = std::filesystem::path(MY_AL_ILQR_SOURCE_DIR);
  const std::filesystem::path script_path =
      repo_root / "visualization" / "plot_phase11_optimization_animation.py";
  const std::filesystem::path output_gif = output_dir / "phase11_optimization_animation.gif";

  if (!std::filesystem::exists(script_path)) {
    std::cout << "matplotlib animation script missing: " << script_path << "\n";
    return;
  }

  const std::string command =
      "python3 " + QuoteShellArg(script_path) + " --build-dir " + QuoteShellArg(output_dir) +
      " --output-gif " + QuoteShellArg(output_gif);

  std::cout << "Generating matplotlib GIF with command: " << command << "\n";
  const int exit_code = std::system(command.c_str());
  if (exit_code == 0) {
    std::cout << "matplotlib GIF saved to        = " << output_gif << "\n";

    // Auto-open GIF with system default viewer
    const std::string open_command = "xdg-open " + QuoteShellArg(output_gif) + " &";
    std::cout << "Opening GIF with: " << open_command << "\n";
    std::system(open_command.c_str());
  } else {
    std::cout << "matplotlib GIF generation failed with exit code " << exit_code << "\n";
  }
}

}  // namespace

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

  const std::filesystem::path output_dir =
      std::filesystem::current_path().parent_path() == std::filesystem::path("al-ilqr-starter")
          ? std::filesystem::current_path()
          : std::filesystem::current_path() / "build";
  const std::filesystem::path initial_csv = output_dir / "phase11_dynamic_initial_trajectory.csv";
  const std::filesystem::path optimized_csv = output_dir / "phase11_dynamic_optimized_trajectory.csv";
  const std::filesystem::path obstacle_csv = output_dir / "phase11_dynamic_obstacle_prediction.csv";
  const std::filesystem::path outer_log_csv = output_dir / "phase11_dynamic_outer_log.csv";
  const std::filesystem::path inner_cost_csv = output_dir / "phase11_dynamic_inner_cost_history.csv";
  const std::filesystem::path traj_evolution_csv =
      output_dir / "phase11_dynamic_trajectory_evolution.csv";
  const std::filesystem::path traj_evolution_meta_csv =
      output_dir / "phase11_dynamic_trajectory_evolution_meta.csv";

  my_al_ilqr::WriteTrajectoryCsv(initial_csv, initial_trajectory);
  my_al_ilqr::WriteTrajectoryCsv(optimized_csv, result.trajectory);
  my_al_ilqr::WriteObstaclePredictionCsv(
      obstacle_csv, scenario.obstacle_centers, scenario.obstacle_radius);
  my_al_ilqr::WriteALILQROuterLogCsv(outer_log_csv, result.outer_logs);
  my_al_ilqr::WriteALILQRInnerCostHistoryCsv(inner_cost_csv, result.outer_logs);
  my_al_ilqr::WriteALILQRTrajectoryEvolutionCsv(traj_evolution_csv, result.outer_logs);
  my_al_ilqr::WriteALILQRTrajectoryEvolutionMetaCsv(traj_evolution_meta_csv, result.outer_logs);

  std::cout << "Phase 11 dynamic obstacle demo\n";
  std::cout << "output dir                    = " << output_dir << "\n";
  std::cout << "initial trajectory csv        = " << initial_csv << "\n";
  std::cout << "optimized trajectory csv      = " << optimized_csv << "\n";
  std::cout << "obstacle prediction csv       = " << obstacle_csv << "\n";
  std::cout << "outer log csv                 = " << outer_log_csv << "\n";
  std::cout << "inner cost history csv        = " << inner_cost_csv << "\n";
  std::cout << "trajectory evolution csv      = " << traj_evolution_csv << "\n";
  std::cout << "trajectory evolution meta csv = " << traj_evolution_meta_csv << "\n";
  std::cout << "converged                     = " << (result.converged ? "true" : "false") << "\n";
  std::cout << "final violation               = " << result.final_violation << "\n";
  std::cout << "best violation                = " << result.best_violation << "\n";
  std::cout << "terminal state                = "
            << result.trajectory.State(scenario.base_problem->Horizon()).transpose() << "\n";

  TryGeneratePhase11Gif(output_dir);

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

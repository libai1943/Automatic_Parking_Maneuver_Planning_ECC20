// ECC 2020: two-disc safe travel corridors and minimum-time parking.
// The NLP follows NLP.mod: 100 nodes, backward Euler, h = tf / 100.
#include <algorithm>
#include <array>
#include <casadi/casadi.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>
extern "C" int plan_rigid_path(const double *, const double *, const double *,
                               const double *, int, int, double *, int, char *,
                               int);
using casadi::DM;
using casadi::SX;
namespace fs = std::filesystem;
constexpr int nodes = 100, fields = 11;
constexpr double wheelbase = 2.8, rear_center = .2432, front_center = 2.5877;
using Pose = std::array<double, 3>;
using Box = std::array<double, 4>;
struct Scene {
  std::array<double, 6> boundary;
  std::vector<double> obstacles;
  std::vector<std::vector<int>> map;
};
Scene load_scene(const fs::path &path) {
  Scene s;
  std::ifstream input(path / "case_1.txt");
  int count = 0;
  for (auto &q : s.boundary)
    input >> q;
  input >> count;
  s.obstacles.resize(8 * count);
  for (auto &q : s.obstacles)
    input >> q;
  if (!input || count < 1)
    throw std::runtime_error("Cannot read case_1.txt");
  std::ifstream grid(path / "dilated_map.csv");
  std::string line;
  while (std::getline(grid, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    std::vector<int> values;
    int v;
    while (row >> v)
      values.push_back(v);
    s.map.push_back(values);
  }
  if (s.map.size() != 201)
    throw std::runtime_error("Expected the original 201 x 201 dilated map");
  for (auto &row : s.map)
    if (row.size() != 201)
      throw std::runtime_error("Invalid map width");
  return s;
}
bool free_box(const Scene &s, const Box &b) {
  if (b[0] < -20 || b[1] > 20 || b[2] < -20 || b[3] > 20)
    return false;
  int xl = int(std::ceil((b[0] + 20) / .2)),
      xu = int(std::ceil((b[1] + 20) / .2));
  int yl = int(std::ceil((b[2] + 20) / .2)),
      yu = int(std::ceil((b[3] + 20) / .2));
  for (int x = xl; x <= xu; ++x)
    for (int y = yl; y <= yu; ++y)
      if (s.map[x][y])
        return false;
  return true;
}
Box grow_box(const Scene &s, double x, double y) {
  if (!free_box(s, {x, x, y, y})) {
    bool found = false;
    for (int step = 1; step <= 4000 && !found; ++step)
      for (auto delta : std::array<std::array<double, 2>, 4>{
               {{1, 0}, {-1, 0}, {0, 1}, {0, -1}}}) {
        double xx = x + step * .01 * delta[0], yy = y + step * .01 * delta[1];
        if (free_box(s, {xx, xx, yy, yy})) {
          x = xx;
          y = yy;
          found = true;
          break;
        }
      }
    if (!found)
      throw std::runtime_error("No free corridor seed");
  }
  Box b{x, x, y, y};
  std::array<double, 4> length{};
  std::array<bool, 4> done{};
  // The original order is up, left, down, right, with 0.03 m increments.
  std::array<int, 4> side{3, 0, 2, 1};
  int remaining = 4;
  while (remaining) {
    for (int d = 0; d < 4; ++d)
      if (!done[d]) {
        int k = side[d];
        Box proposed = b;
        proposed[k] += (k == 0 || k == 2 ? -.03 : .03);
        if (length[d] + .03 > 10 || !free_box(s, proposed)) {
          done[d] = true;
          --remaining;
        } else {
          b = proposed;
          length[d] += .03;
        }
      }
  }
  return b;
}
std::vector<Pose> search(const Scene &s) {
  double bounds[] = {-20, 20, -20, 20},
         vehicle[] = {2.8, 3.76, .929, 1.942, .7};
  std::vector<double> buffer(300000);
  char message[512]{};
  int n = plan_rigid_path(bounds, vehicle, s.boundary.data(),
                          s.obstacles.data(), int(s.obstacles.size() / 8), 1,
                          buffer.data(), 100000, message, 512);
  if (n < 2)
    throw std::runtime_error(message);
  std::vector<Pose> path(n);
  for (int i = 0; i < n; ++i)
    for (int k = 0; k < 3; ++k)
      path[i][k] = buffer[i * 3 + k];
  return path;
}
std::vector<Pose> resample(const std::vector<Pose> &path, double &length) {
  std::vector<double> distance(path.size(), 0);
  for (size_t i = 1; i < path.size(); ++i)
    distance[i] = distance[i - 1] + std::hypot(path[i][0] - path[i - 1][0],
                                               path[i][1] - path[i - 1][1]);
  length = distance.back();
  std::vector<Pose> sampled(nodes);
  size_t j = 1;
  for (int i = 0; i < nodes; ++i) {
    double s = length * i / (nodes - 1);
    while (j + 1 < path.size() && distance[j] < s)
      ++j;
    double a =
        (s - distance[j - 1]) / std::max(1e-12, distance[j] - distance[j - 1]);
    for (int k = 0; k < 3; ++k)
      sampled[i][k] = path[j - 1][k] + a * (path[j][k] - path[j - 1][k]);
  }
  return sampled;
}
int main(int argc, char **argv) {
  try {
    fs::path data = PARKING_DATA_DIR, output = "results";
    std::string linear_solver = "mumps";
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      if (a == "--data" && i + 1 < argc)
        data = fs::u8path(argv[++i]);
      else if (a == "--output" && i + 1 < argc)
        output = fs::u8path(argv[++i]);
      else if (a == "--linear-solver" && i + 1 < argc)
        linear_solver = argv[++i];
      else
        throw std::runtime_error("Usage: parking_demo [--data DIR] [--output "
                                 "DIR] [--linear-solver mumps]");
    }
    auto start = std::chrono::steady_clock::now();
    Scene scene = load_scene(data);
    auto path = search(scene);
    double length;
    auto reference = resample(path, length);
    std::vector<std::array<Box, 2>> corridors(nodes);
    for (int i = 0; i < nodes; ++i)
      for (int d = 0; d < 2; ++d) {
        double offset = d ? front_center : rear_center;
        corridors[i][d] =
            grow_box(scene, reference[i][0] + offset * cos(reference[i][2]),
                     reference[i][1] + offset * sin(reference[i][2]));
      }
    SX z = SX::sym("z", fields * nodes + 1);
    SX tf = z(fields * nodes), h = tf / nodes;
    auto q = [&](int i, int j) { return z(i * fields + j); };
    std::vector<double> lb(fields * nodes + 1, -1e20),
        ub(fields * nodes + 1, 1e20), guess(fields * nodes + 1, 0), gl, gu;
    std::vector<SX> expressions;
    auto add = [&](const SX &expression, double lower = 0, double upper = 0) {
      expressions.push_back(expression);
      gl.push_back(lower);
      gu.push_back(upper);
    };
    guess.back() = std::clamp(length / 1.5 + 5., 10., 45.);
    lb.back() = .1;
    ub.back() = 50;
    double dt = guess.back() / nodes;
    for (int i = 0; i < nodes; ++i) {
      for (int j = 0; j < 3; ++j)
        guess[i * fields + j] = reference[i][j];
      for (int j = 3; j <= 6; ++j) {
        double bound = j == 3 ? 2.5 : j == 4 ? 1. : j == 5 ? .7 : .5;
        lb[i * fields + j] = -bound;
        ub[i * fields + j] = bound;
      }
      if (i > 0 && i < nodes - 1) {
        double dx = reference[i][0] - reference[i - 1][0],
               dy = reference[i][1] - reference[i - 1][1],
               signed_ds =
                   dx * cos(reference[i][2]) + dy * sin(reference[i][2]);
        guess[i * fields + 3] = std::clamp(signed_ds / dt, -2.5, 2.5);
        guess[i * fields + 5] = std::clamp(
            std::atan(wheelbase * (reference[i][2] - reference[i - 1][2]) /
                      (std::abs(signed_ds) < 1e-9 ? 1e-9 : signed_ds)),
            -.7, .7);
      }
      for (int d = 0; d < 2; ++d) {
        double offset = d ? front_center : rear_center;
        int j = 7 + 2 * d;
        add(q(i, j) - q(i, 0) - offset * cos(q(i, 2)));
        add(q(i, j + 1) - q(i, 1) - offset * sin(q(i, 2)));
        auto b = corridors[i][d];
        lb[i * fields + j] = b[0];
        ub[i * fields + j] = b[1];
        lb[i * fields + j + 1] = b[2];
        ub[i * fields + j + 1] = b[3];
        guess[i * fields + j] = std::clamp(
            reference[i][0] + offset * cos(reference[i][2]), b[0], b[1]);
        guess[i * fields + j + 1] = std::clamp(
            reference[i][1] + offset * sin(reference[i][2]), b[2], b[3]);
      }
      if (i) {
        add(q(i, 0) - q(i - 1, 0) - h * q(i, 3) * cos(q(i, 2)));
        add(q(i, 1) - q(i - 1, 1) - h * q(i, 3) * sin(q(i, 2)));
        add(q(i, 2) - q(i - 1, 2) - h * q(i, 3) * tan(q(i, 5)) / wheelbase);
        add(q(i, 3) - q(i - 1, 3) - h * q(i, 4));
        add(q(i, 5) - q(i - 1, 5) - h * q(i, 6));
      }
    }
    for (int i : {0, nodes - 1}) {
      int offset = i ? 3 : 0;
      for (int j = 0; j < 2; ++j)
        add(q(i, j) - scene.boundary[offset + j]);
      if (!i)
        add(q(i, 2) - scene.boundary[2]);
      else {
        add(sin(q(i, 2)) - sin(scene.boundary[5]));
        add(cos(q(i, 2)) - cos(scene.boundary[5]));
      }
      for (int j = 3; j <= 6; ++j) {
        lb[i * fields + j] = ub[i * fields + j] = 0;
        guess[i * fields + j] = 0;
      }
    }
    SX g = SX::vertcat(expressions);
    casadi::Dict options;
    options["print_time"] = false;
    options["ipopt.print_level"] = 0;
    options["ipopt.sb"] = "yes";
    options["ipopt.tol"] = 1e-7;
    options["ipopt.max_iter"] = 3000;
    options["ipopt.max_cpu_time"] = 120.;
    options["ipopt.linear_solver"] = linear_solver;
    if (const char *hsl = std::getenv("HSL_LIBRARY"))
      options["ipopt.hsllib"] = hsl;
    auto solver =
        casadi::nlpsol("parking", "ipopt",
                       casadi::SXDict{{"x", z}, {"f", tf}, {"g", g}}, options);
    auto result = solver(casadi::DMDict{
        {"x0", guess}, {"lbx", lb}, {"ubx", ub}, {"lbg", gl}, {"ubg", gu}});
    if (!bool(solver.stats().at("success")))
      throw std::runtime_error(solver.stats().at("return_status").to_string());
    auto solution = result.at("x").nonzeros(),
         constraints = result.at("g").nonzeros();
    double residual = 0;
    for (size_t i = 0; i < constraints.size(); ++i)
      residual =
          std::max({residual, gl[i] - constraints[i], constraints[i] - gu[i]});
    for (size_t i = 0; i < solution.size(); ++i)
      residual = std::max({residual, lb[i] - solution[i], solution[i] - ub[i]});
    if (residual > 1e-5)
      throw std::runtime_error("Constraint validation failed");
    fs::create_directories(output);
    std::ofstream file(output / "trajectory.csv");
    file << std::setprecision(17) << "t,x,y,theta,v,a,phi,omega\n";
    for (int i = 0; i < nodes; ++i) {
      file << i * solution.back() / nodes;
      for (int j = 0; j < 7; ++j)
        file << ',' << solution[i * fields + j];
      file << '\n';
    }
    std::ofstream warm(output / "reference.csv");
    warm << "x,y,theta\n";
    for (auto p : reference)
      warm << p[0] << ',' << p[1] << ',' << p[2] << '\n';
    std::cout << "Case 1: Tf=" << solution.back() << " s, nodes=" << nodes
              << ", constraint residual=" << residual << ", planning="
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - start)
                     .count()
              << " s\n";
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

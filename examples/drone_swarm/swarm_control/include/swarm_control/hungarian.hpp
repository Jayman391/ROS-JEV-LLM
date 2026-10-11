// Minimum-cost assignment (Hungarian / Kuhn-Munkres with potentials), O(n^3) for an n x n matrix.
// Rectangular inputs are padded square with zero-cost dummies; dummy matches come back as -1.
#pragma once

#include <algorithm>
#include <limits>
#include <vector>

namespace swarm_control
{

// cost: row-major rows x cols. Returns, per row, the assigned column or -1.
inline std::vector<int> hungarian(const std::vector<double> & cost, int rows, int cols)
{
  const int n = std::max(rows, cols);
  auto c = [&](int i, int j) {          // 1-based, padded
      return (i <= rows && j <= cols) ? cost[(i - 1) * cols + (j - 1)] : 0.0;
    };
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> u(n + 1, 0), v(n + 1, 0);
  std::vector<int> match(n + 1, 0), way(n + 1, 0);   // match[col] = row
  for (int i = 1; i <= n; ++i) {
    match[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, INF);
    std::vector<char> used(n + 1, false);
    do {
      used[j0] = true;
      const int i0 = match[j0];
      double delta = INF;
      int j1 = 0;
      for (int j = 1; j <= n; ++j) {
        if (used[j]) {continue;}
        const double cur = c(i0, j) - u[i0] - v[j];
        if (cur < minv[j]) {minv[j] = cur; way[j] = j0;}
        if (minv[j] < delta) {delta = minv[j]; j1 = j;}
      }
      for (int j = 0; j <= n; ++j) {
        if (used[j]) {u[match[j]] += delta; v[j] -= delta;} else {minv[j] -= delta;}
      }
      j0 = j1;
    } while (match[j0] != 0);
    do {
      const int j1 = way[j0];
      match[j0] = match[j1];
      j0 = j1;
    } while (j0);
  }
  std::vector<int> out(rows, -1);
  for (int j = 1; j <= n; ++j) {
    if (match[j] >= 1 && match[j] <= rows && j <= cols) {out[match[j] - 1] = j - 1;}
  }
  return out;
}

}  // namespace swarm_control

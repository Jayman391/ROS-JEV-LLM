// Drone swarm world model shared by the C++ nodes (SYSTEM_MODEL §2-§4, §7): scenario params,
// energy, link quality, ray casting, formation slots and the H1-H5 safety filter. Pure functions,
// so gtest covers them without ROS. The energy model is mirrored in drone_swarm/scenario.py.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace swarm_control
{

struct V3
{
  double x{0}, y{0}, z{0};
  V3 operator+(const V3 & o) const {return {x + o.x, y + o.y, z + o.z};}
  V3 operator-(const V3 & o) const {return {x - o.x, y - o.y, z - o.z};}
  V3 operator*(double k) const {return {x * k, y * k, z * k};}
  double hnorm() const {return std::hypot(x, y);}
  double norm() const {return std::sqrt(x * x + y * y + z * z);}
};

struct Cylinder
{
  double x, y, r, h;
};

struct Params
{
  int num_drones{4};
  int seed{0};
  std::array<double, 4> ao{20, 320, -150, 150};   // x0, x1, y0, y1
  double pad_spacing{5};
  std::vector<Cylinder> obstacles, nfz;          // nfz: h = inf
  double v_max_h{5}, v_max_up{2}, v_max_down{1.5}, a_max{2}, yaw_rate_max{1};
  double v_survey{4}, v_transit{5}, v_inspect{2};
  double alt_survey{30}, alt_transit{40}, alt_inspect{10}, alt_min{5}, alt_max{60};
  double E_0{40}, P_hover{180}, k_h{2}, k_c{25}, k_w{1.5}, reserve{0.1}, E_land_wh{0.3};
  double det_fov_deg{60}, det_p0{0.9}, det_h_opt{15}, det_sigma_h{15}, det_v_blur{8};
  double det_lambda_fp{0.02};
  double R_c{250}, link_slope{20}, latency_ms{50}, latency_jitter_ms{200};
  double d_sep{4}, obstacle_clear{3}, nfz_buffer{5}, lost_link_hold_s{10}, lost_link_rtb_s{30};
};

inline Params load_params(rclcpp::Node & n)
{
  Params p;
  auto d = [&n](const std::string & name, double def) {
      return n.declare_parameter<double>(name, def);
    };
  p.num_drones = static_cast<int>(n.declare_parameter<int64_t>("num_drones", p.num_drones));
  p.seed = static_cast<int>(n.declare_parameter<int64_t>("seed", p.seed));
  auto ao = n.declare_parameter<std::vector<double>>("ao", {20, 320, -150, 150});
  std::copy_n(ao.begin(), 4, p.ao.begin());
  p.pad_spacing = d("pad_spacing", p.pad_spacing);
  auto ob = n.declare_parameter<std::vector<double>>("obstacles", std::vector<double>{});
  for (size_t i = 0; i + 3 < ob.size(); i += 4) {
    p.obstacles.push_back({ob[i], ob[i + 1], ob[i + 2], ob[i + 3]});
  }
  auto nf = n.declare_parameter<std::vector<double>>("nfz", std::vector<double>{});
  for (size_t i = 0; i + 2 < nf.size(); i += 3) {
    p.nfz.push_back({nf[i], nf[i + 1], nf[i + 2], std::numeric_limits<double>::infinity()});
  }
#define SWARM_P(f) p.f = d(#f, p.f)
  SWARM_P(v_max_h); SWARM_P(v_max_up); SWARM_P(v_max_down); SWARM_P(a_max); SWARM_P(yaw_rate_max);
  SWARM_P(v_survey); SWARM_P(v_transit); SWARM_P(v_inspect);
  SWARM_P(alt_survey); SWARM_P(alt_transit); SWARM_P(alt_inspect); SWARM_P(alt_min);
  SWARM_P(alt_max);
  SWARM_P(E_0); SWARM_P(P_hover); SWARM_P(k_h); SWARM_P(k_c); SWARM_P(k_w); SWARM_P(reserve);
  SWARM_P(E_land_wh);
  SWARM_P(det_fov_deg); SWARM_P(det_p0); SWARM_P(det_h_opt); SWARM_P(det_sigma_h);
  SWARM_P(det_v_blur); SWARM_P(det_lambda_fp);
  SWARM_P(R_c); SWARM_P(link_slope); SWARM_P(latency_ms); SWARM_P(latency_jitter_ms);
  SWARM_P(d_sep); SWARM_P(obstacle_clear); SWARM_P(nfz_buffer); SWARM_P(lost_link_hold_s);
  SWARM_P(lost_link_rtb_s);
#undef SWARM_P
  return p;
}

// Drone index (0-based) from a name like "uav3" (-> 2); -1 if not of that form.
inline int drone_index(const std::string & name)
{
  auto i = name.find_last_not_of("0123456789");
  if (i == std::string::npos || i + 1 >= name.size()) {return -1;}
  return std::stoi(name.substr(i + 1)) - 1;
}

inline V3 pad(const Params & p, int k)
{
  return {0.0, (k - (p.num_drones - 1) / 2.0) * p.pad_spacing, 0.0};
}

// --- energy (§3.1) ----------------------------------------------------------------------------

inline double power_w(const Params & p, const V3 & v, const V3 & wind)
{
  const double vh2 = v.x * v.x + v.y * v.y;
  const double ax = v.x - wind.x, ay = v.y - wind.y;
  return p.P_hover + p.k_h * vh2 + p.k_c * std::max(v.z, 0.0) + p.k_w * (ax * ax + ay * ay);
}

// Energy to get home from pos and land: climb to transit altitude if below, fly the leg at
// v_transit against the wind, descend, land (Wh). Deterministic: never asked of a model.
inline double rtb_wh(const Params & p, const V3 & pos, const V3 & home, const V3 & wind)
{
  double e = 0.0;
  if (pos.z < p.alt_transit - 1) {
    e += (p.alt_transit - pos.z) / p.v_max_up * power_w(p, {0, 0, p.v_max_up}, wind) / 3600;
  }
  const V3 leg{home.x - pos.x, home.y - pos.y, 0};
  const double d = leg.hnorm();
  if (d > 1e-6) {
    const V3 v = leg * (p.v_transit / d);
    e += d / p.v_transit * power_w(p, v, wind) / 3600;
  }
  e += std::max(pos.z, p.alt_transit) / p.v_max_down * p.P_hover / 3600;
  return e + p.E_land_wh;
}

// m_E = (E - E_rtb - R) / E_0
inline double energy_margin(const Params & p, double energy_wh, double rtb)
{
  return (energy_wh - rtb - p.reserve * p.E_0) / p.E_0;
}

// --- comms (§4.8) ------------------------------------------------------------------------------

inline double link_quality(const Params & p, double d)
{
  return 1.0 / (1.0 + std::exp((d - p.R_c) / p.link_slope));
}

// --- geometry ---------------------------------------------------------------------------------

// Distance along a horizontal ray from (x, y) at angle a to a cylinder's wall; inf if missed or
// the cylinder is lower than z. Inside the cylinder: 0.
inline double ray_cylinder(double x, double y, double z, double a, const Cylinder & c)
{
  if (z > c.h) {return std::numeric_limits<double>::infinity();}
  const double dx = std::cos(a), dy = std::sin(a), ox = x - c.x, oy = y - c.y;
  const double b = ox * dx + oy * dy, cc = ox * ox + oy * oy - c.r * c.r;
  if (cc <= 0) {return 0.0;}
  const double disc = b * b - cc;
  if (disc < 0 || b > 0) {return std::numeric_limits<double>::infinity();}
  return -b - std::sqrt(disc);
}

// Formation slots in the leader frame (x forward, y left), slot 0 = the leader point.
inline std::vector<V3> formation_slots(const std::string & shape, int n, double s)
{
  std::vector<V3> out;
  for (int i = 0; i < n; ++i) {
    const int k = (i + 1) / 2, side = (i % 2) ? 1 : -1;
    if (shape == "line") {
      out.push_back({0, (i - (n - 1) / 2.0) * s, 0});
    } else if (shape == "v") {
      out.push_back({-k * s, side * k * s, 0});
    } else if (shape == "diamond") {
      static const double dia[4][2] = {{0, 0}, {-1, 1}, {-1, -1}, {-2, 0}};
      out.push_back(i < 4 ? V3{dia[i][0] * s, dia[i][1] * s, 0} : V3{-(i - 1) * s, 0, 0});
    } else if (shape == "circle") {
      const double r = n > 1 ? s / (2 * std::sin(M_PI / n)) : 0.0, a = 2 * M_PI * i / n;
      out.push_back({r * std::cos(a), r * std::sin(a), 0});
    } else {                                                    // column (default)
      out.push_back({-i * s, 0, 0});
    }
  }
  return out;
}

// --- safety filter (H1-H5) ----------------------------------------------------------------------

struct Safety
{
  V3 v;
  std::vector<std::string> reflexes;
};

namespace detail
{
// Remove the velocity component toward an obstacle along n (unit, pointing *away*) and redirect it
// tangentially, so the drone slides around instead of stopping.
inline void slide(V3 & v, double nx, double ny)
{
  const double vin = -(v.x * nx + v.y * ny);
  if (vin <= 0) {return;}
  const double tx = -ny, ty = nx, side = (v.x * tx + v.y * ty) >= 0 ? 1.0 : -1.0;
  v.x += nx * vin + side * tx * vin;
  v.y += ny * vin + side * ty * vin;
}
inline void add(std::vector<std::string> & r, const std::string & s)
{
  if (std::find(r.begin(), r.end(), s) == r.end()) {r.push_back(s);}
}
}  // namespace detail

// v: desired world velocity. ranges: proximity beams (world frame, from angle a0 step da).
// others: positions of other airborne drones. vh_cap / vup_cap: speed limits (motor faults lower
// them). Returns the commanded velocity and the reflexes that acted.
inline Safety safety_filter(
  const Params & p, const V3 & pos, V3 v, const std::vector<double> & ranges, double a0,
  double da, const std::vector<V3> & others, double vh_cap, double vup_cap)
{
  Safety out;
  // H2: no-fly zones (buffered), repulsive field + tangential detour
  for (const auto & z : p.nfz) {
    const double dx = pos.x - z.x, dy = pos.y - z.y, d = std::max(1e-6, std::hypot(dx, dy));
    const double R = z.r + p.nfz_buffer;
    if (d < R + 15) {
      const double nx = dx / d, ny = dy / d;
      const double before = v.x * nx + v.y * ny;
      detail::slide(v, nx, ny);
      if (d < R) {
        v.x += nx * std::min(3.0, 0.5 * (R - d) + 0.5);
        v.y += ny * std::min(3.0, 0.5 * (R - d) + 0.5);
      }
      if (d < R || (before < 0 && d < R + 5)) {detail::add(out.reflexes, "nfz");}
    }
  }
  // H3: obstacles seen by the proximity scan: slide along the nearest, push out of clearance
  int nearest = -1;
  for (size_t i = 0; i < ranges.size(); ++i) {
    if (ranges[i] < 3 * p.obstacle_clear && (nearest < 0 || ranges[i] < ranges[nearest])) {
      nearest = static_cast<int>(i);
    }
  }
  if (nearest >= 0) {
    const double a = a0 + nearest * da, r = ranges[nearest];
    const double nx = -std::cos(a), ny = -std::sin(a);           // away from the obstacle
    const double toward = -(v.x * nx + v.y * ny);
    if (toward > 0) {
      if (r < 1.5) {
        v.x += nx * toward;                                       // stop: no inward motion
        v.y += ny * toward;
      } else {
        detail::slide(v, nx, ny);
      }
      detail::add(out.reflexes, "obstacle");
    }
    for (size_t i = 0; i < ranges.size(); ++i) {
      if (ranges[i] < p.obstacle_clear) {
        const double ai = a0 + i * da;
        v.x -= std::cos(ai) * (p.obstacle_clear - ranges[i]);
        v.y -= std::sin(ai) * (p.obstacle_clear - ranges[i]);
        detail::add(out.reflexes, "obstacle");
      }
    }
  }
  // H4: separation from other drones
  for (const auto & o : others) {
    const V3 dv = pos - o;
    const double d = std::max(1e-3, dv.norm());
    if (d < 2 * p.d_sep && std::abs(dv.z) < 4) {
      const double k = std::min(2.0, (2 * p.d_sep - d) / p.d_sep * 1.5);
      v = v + V3{dv.x / d, dv.y / d, 0} * k;
      if (d < p.d_sep) {detail::add(out.reflexes, "separation");}
    }
  }
  // H1: geofence (AO + base corridor) and altitude band
  const double gx0 = std::min(-20.0, p.ao[0] - 10), gx1 = p.ao[1] + 10;
  const double gy0 = p.ao[2] - 10, gy1 = p.ao[3] + 10;
  auto fence = [&](double pv, double lo, double hi, double & vel) {
      if (pv <= lo + 2 && vel < 0) {vel = std::max(0.5, lo + 2 - pv); detail::add(out.reflexes, "geofence");}
      if (pv >= hi - 2 && vel > 0) {vel = std::min(-0.5, hi - 2 - pv); detail::add(out.reflexes, "geofence");}
    };
  fence(pos.x, gx0, gx1, v.x);
  fence(pos.y, gy0, gy1, v.y);
  if (pos.z >= p.alt_max - 0.5 && v.z > 0) {v.z = 0; detail::add(out.reflexes, "geofence");}
  // H5: saturation
  const double h = v.hnorm();
  if (h > vh_cap) {v.x *= vh_cap / h; v.y *= vh_cap / h;}
  v.z = std::clamp(v.z, -p.v_max_down, vup_cap);
  out.v = v;
  return out;
}

}  // namespace swarm_control

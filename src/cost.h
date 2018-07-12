#ifndef COST_H
#define COST_H
#include "stdio.h"
#include "cmath"

using namespace std;


double Logistic2(double x) { return 2.0 / (1 + exp(-x)) - 1.0; }

// L curve max value
// x evaluate at
// m sigmoid midpoint
// k steepness 
double Logistic(double x, double L, double m, double k) {
  return L / (1 + exp(-k * (x - m)));
}

void GenerateDerivative(vector<double> &vel, const vector<double> &s,
                      const double dt) {
  for (int i = 1; i < s.size(); i++) {
    vel.push_back((s[i] - s[i - 1]) / dt);
  }
}

void GenerateVelMag(vector<double> &vel, const vector<double> &s,
                      const vector<double> &d, const double dt) {
  for (int i = 1; i < s.size(); i++) {
    double vs = (s[i] - s[i - 1]) / dt;
    double vd = (d[i] - d[i - 1]) / dt;
    vel.push_back(sqrt(vs*vs+vd*vd));
  }
}

// Target speed: The speed at which you would like the vehicle to travel. Speed
// Limit.
// Intended lane: The intended lane for the given behavior. For PLCR, PLCL, LCR,
// and LCL, this would be the one lane over from the current lane.
// Final lane: the immediate resulting lane of the given behavior. For LCR and
// LCL, this would be one lane over.
// Array of lane speeds, based on traffic in that lane.
// PositionEfficiencyCost
double InefficiencyCost(double target_speed, int intended_lane, int final_lane,
                        double lane_speed[]) {
  /*
  Cost becomes higher for trajectories with intended lane and final lane that
  have traffic slower than target_speed.
  */
  double speed_intended = lane_speed[intended_lane];
  double speed_final = lane_speed[final_lane];
  double cost =
      (2.0 * target_speed - speed_intended - speed_final) / target_speed;
  return cost;
}

// Avoids collision with other vehicles!
double PositionFeasibilityCost(const vector<vector<double>> &pred_s, const vector<vector<double>> &pred_d, const vector<double> &s, const vector<double> &d, const double min_dist) {
  double dist = min_dist;
  for (int i = 0; i < pred_s.size(); i++) {
    for (int j = 0; j < s.size(); j++) {
      double ds = (pred_s[i][j] - s[j]);
      double dd = (pred_d[i][j] - d[j]);
      double d = sqrt(ds * ds + dd * dd);
      if (d < dist) {
        dist = d;
      }
    }
  }
  if (dist < min_dist) {
    return 1.0 - exp(-(min_dist-dist)/min_dist);
  }
  return 0.f;
}

// Buffer Distance from other vehicles in front of the car using the last car vs ego point
double PositionSafetyCost2(const vector<vector<double>> &pred_s, const vector<vector<double>> &pred_d, const vector<double> &s, const vector<double> &d, const double min_dist) {
  double cost = 0.f;
  for (int i = 0; i < pred_s.size(); i++) {
    double car_s_curr = pred_s[i][0];
    double ego_s_curr = s[0];
    double car_s = pred_s[i][pred_s[i].size()-1];
    double ego_s = s[s.size()-1];
    double car_d = pred_d[i][pred_d[i].size()-1];
    double ego_d = d[d.size()-1];
    if (car_d > ego_d - 2 && car_d < ego_d + 2) {
      double delta = car_s - ego_s;
      if ( car_s_curr > ego_s_curr - 2 && delta < min_dist) {
        double c = Logistic(delta, 2, 0, -0.2);
        //fprintf(stderr, "%d:car_s[%f] ego_s[%f] delta[%f] cost[%f]\n", i, car_s, ego_s, delta, Logistic(delta, 2, 0, -0.2));
        if (c > cost) {
          cost = c;
        }
      }
    }
  }
  return cost;
}


// Buffer Distance from other vehicles in front of the car for all car vs ego points
double PositionSafetyCost(const vector<vector<double>> &pred_s, const vector<vector<double>> &pred_d, const vector<double> &s, const vector<double> &d, const double min_dist) {
  double dist = 1e6;
  for (int i = 0; i < pred_s.size(); i++) {
    for (int j = 0; j < s.size(); j++) {
      double delta_s = (pred_s[i][j] - s[j]);
      double delta_d = abs(pred_d[i][j] - d[j]);
      if (delta_d < 2.f /* in same path */ && delta_s < min_dist) {
        if (delta_s < dist) {
          dist = delta_s;
        }
      }
    }
  }
  if (dist < min_dist) {
    return Logistic(dist, 2, -20, -0.1);
  }
  return 0.f;
}

// Near center of the road
double PositionComfortCost(const vector<double> &d, const double lane_center) {
  double total = 0.f;
  for (int i = 0; i < d.size(); i++) {
    // center of lane is 0, 2 meters left or right is ~ 0
    total += (1 / (1 + exp(-pow(d[i]-lane_center,2))) - 0.5) * 2;
  }
  double cost = total / d.size();
  return cost;
}

// Keep us close to the speed limit
double VelocityEffeciencyCost(const vector<double> &s, const double target_speed,
                    const double dt) {
  // vector<double> vel;
  // GenerateVelocity(vel, s, dt);
  // double end_speed = vel[vel.size() - 1];
  double end_speed = (s[s.size() - 1] - s[s.size() - 2]) / dt;
  return Logistic2(abs(target_speed - end_speed) / target_speed);
}

// Keep us from exceeding the speed limit
double VelocityLegalityCost(const vector<double> &s, const double target_speed,
                    const double dt) {
  // vector<double> vel;
  // GenerateVelocity(vel, s, dt);
  // double end_speed = vel[vel.size() - 1];
  double end_speed = (s[s.size() - 1] - s[s.size() - 2]) / dt;
  if (end_speed > target_speed || end_speed < 0) {
    return 1.f;
  }
  return 0.f;
}

// Keep us close to the same speed of the traffic around us
double VelocitySafetyCost(const vector<double> &s) {
  return 0.f;
}

double NormalAccCost(const vector<double> &d, const double max_accel, const double dt) {
  vector<double> vel;
  GenerateDerivative(vel, d, dt);
  vector<double> acc;
  GenerateDerivative(acc, vel, dt);
  double max_val = 0.f;
  for (int i = 0; i < acc.size(); i++) {
    double acc_mag = abs(acc[acc.size()]);
    if (max_val < acc_mag) {
      max_val = acc_mag;
    }
  }
  if (max_val >= max_accel) {
    return 1.f;
  }
  return 0.f;
}

// Only consider accelerations the car can execute
double AccelerationFeasibilityCost(const vector<double> &s, const vector<double> &d, const double max_accel, const double dt) {
  vector<double> vel;
  GenerateVelMag(vel, s, d, dt);
  vector<double> acc;
  GenerateDerivative(acc, vel, dt);
  double max_val = 0.f;
  for (int i = 0; i < acc.size(); i++) {
    double acc_mag = abs(acc[acc.size()]);
    if (max_val < acc_mag) {
      max_val = acc_mag;
    }
  }
  if (max_val >= max_accel) {
    return 1.0 - exp(-(max_val - max_accel)/max_val);
  }
  return 0.f;
}

// Avoid Jerk
double AccelerationComfortCost(const vector<double> &s, const vector<double> &d, const double max_jerk, const double dt) {
  vector<double> vel;
  GenerateVelMag(vel, s, d, dt);
  vector<double> acc;
  GenerateDerivative(acc, vel, dt);
  vector<double> jerk;
  GenerateDerivative(jerk, acc, dt);
  double max_val = 0.f;
  for (int i = 0; i < jerk.size(); i++) {
    double jerk_mag = abs(jerk[jerk.size()]);
    if (max_val < jerk_mag) {
      max_val = jerk_mag;
    }
  }
  if (max_val >= max_jerk) {
    return 1.0 - exp(-(max_val - max_jerk)/max_val);
  }
  return 0.f;
}

#endif // COST_H
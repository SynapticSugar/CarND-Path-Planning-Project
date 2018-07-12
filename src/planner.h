#ifndef PLANNER_H
#define PLANNER_H

#include "helper.h"
#include <vector>

using namespace std;

constexpr double const kMAX_ACCEL = 4.0;
constexpr double const kMAX_JERK = 4.0;
constexpr double const kMPH2MPS = 0.44704;
constexpr double const kSEC_PER_STEP = 0.01;
constexpr int    const kSTEPS = 50;
constexpr double const kVDELTA = 0.02;
constexpr double const kCOLLISION_DIST = 1.5; // TODO edit cost fcn so we dont go outside lane on curves
constexpr double const kBUFFER_DIST = 1.0;
constexpr double const kVCONTROL = 0.1;

//                         1    2     3    4    5
typedef enum { READY = 0, KL, PLCL, PLCR, LCL, LCR } planner_state_t;

struct Traj {
  planner_state_t s;   // The state this trajectory belongs to
  double cost[10];      // Individual costs
  bool collision;      // True if this path results in a collision
  double total_cost;   // The total cost
  double velocity;     // Resultant velocity in m/s
  vector<double> path; // Spline of consecutive x,y points
  int target_lane;     // The intended lane to move to
  double desired_d;
};

class PlannerState {
public:
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  double end_path_s;
  double end_path_d;
  vector<double> *map_waypoints_x;
  vector<double> *map_waypoints_y;
  vector<double> *map_waypoints_s;
  vector<double> *map_waypoints_dx;
  vector<double> *map_waypoints_dy;
  vector<double> *previous_path_x;
  vector<double> *previous_path_y;
  vector<vector<double>> *sensor_fusion;
  PlannerState();
  virtual ~PlannerState();
};

class Planner {
public:
  planner_state_t current_state;
  double speed_limit;   // m/s
  int desired_lane;     // leftmost is lane 0
  int target_lane;      // The target for a lane change
  double desired_vel;   // m/s
  double lane_speed[3]; // m/s
  PlannerState v_ctx;

  /**
    * Constructor
    */
  Planner();
  Planner(double speed_limit, int lane);

  /**
  * Destructor
  */
  virtual ~Planner();
  void UpdateLaneSpeeds(const int steps, const double s,
                        const vector<vector<double>> &sensor_fusion);
  void GenerateTrajectory(Traj &traj);
  void GeneratePLCLTrajectory(vector<Traj> &t);
  void GenerateKLTrajectory(vector<Traj> &t);
  void GenerateLCLTrajectory(vector<Traj> &t);
  void GeneratePLCRTrajectory(vector<Traj> &t);
  void GenerateLCRTrajectory(vector<Traj> &t);
  void UpdateSensorFusion(vector<vector<double>> &s, vector<vector<double>> &d);
  void GetFrenetTraj(const Traj &traj, vector<double> &s, vector<double> &d);
  void UpdateVelocity();
  bool TargetLaneReached();
  int LowestCost(vector<Traj> &t);
  double TotalCost(Traj &traj, const vector<vector<double>> &pred_s, const vector<vector<double>> &pred_d,
                          const vector<double> &s, const vector<double> &d);
  planner_state_t UpdateFSM(vector<Traj> &t, int &index);
  void GetTrajectory(vector<double> &next_x_vals, vector<double> &next_y_vals,
                     PlannerState &v);
};

#endif // PLANNER_H
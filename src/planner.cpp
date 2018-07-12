#include "planner.h"
#include "cost.h"
#include "spline.h"

using namespace std;

PlannerState::PlannerState() {}

PlannerState::~PlannerState() {}

/**
 * Initializes Planner
 */

Planner::Planner() {
  current_state = READY;
  this->speed_limit = 49.0 * kMPH2MPS; // MPH to mps
  this->desired_lane = 1;
  this->target_lane = 1;
  this->desired_vel = 0.0; // TODO: desired_vel
}

Planner::Planner(double speed_limit, int lane) {
  this->current_state = READY;
  this->speed_limit = speed_limit * kMPH2MPS; // MPH to mps
  this->desired_lane = lane;
  this->target_lane = lane;
  this->desired_vel = 0.0;
  this->lane_speed[lane] = this->speed_limit;
}

Planner::~Planner() {}

void Planner::UpdateVelocity() {

  vector<double> previous_path_x = *(this->v_ctx.previous_path_x);
  int prev_size =
      previous_path_x.size(); 

  bool too_close = false;
  double desired_d = 2 + 4 * desired_lane;
  vector<vector<double>> sf = *(v_ctx.sensor_fusion);
  double car_s = v_ctx.car_s;

  // move car to the end of the previous path
  if (prev_size > 0) {
    car_s = v_ctx.end_path_s;
  }

  for (int i = 0; i < sf.size(); i++) {
    float d = sf[i][6];
    if (d < desired_d + 2 &&
        d > desired_d - 2) { // anywhere in the desired lane width
      double vx = sf[i][3];
      double vy = sf[i][4];
      double check_speed = sqrt(vx * vx + vy * vy);
      double check_car_s = sf[i][5];

      check_car_s += (double)prev_size * kSEC_PER_STEP * check_speed;
      if (check_car_s > car_s &&
          check_car_s - car_s < 30) { // if it is in the track gap ahead
        desired_vel -= kVCONTROL;
        if (desired_vel < check_speed && check_car_s - car_s > desired_vel * 2) {
          desired_vel = check_speed;
        }
        too_close = true;
      }
    }
  }

  if (too_close == false && desired_vel < speed_limit) {
    desired_vel += kVCONTROL;
  }

  if (desired_vel > speed_limit) {
    desired_vel = speed_limit;
  }
}

void Planner::GenerateTrajectory(Traj &traj) {

  double speed = traj.velocity;
  vector<double> previous_path_x = *(this->v_ctx.previous_path_x);
  vector<double> previous_path_y = *(this->v_ctx.previous_path_y);

  int prev_size =
      previous_path_x.size(); // last list of path points from simulator

  double car_s = v_ctx.car_s;
  // move car to the end of the previous path
  if (prev_size > 0) {
    car_s = v_ctx.end_path_s;
  } else {
    prev_size = 0;
  }

  vector<double> ptsx;
  vector<double> ptsy;
  double ref_x = v_ctx.car_x;
  double ref_y = v_ctx.car_y;
  double ref_yaw = deg2rad(v_ctx.car_yaw);

  if (prev_size < 2) {
    // use the tangent of last and current
    double prev_car_x = v_ctx.car_x - cos(v_ctx.car_yaw);
    double prev_car_y = v_ctx.car_y - sin(v_ctx.car_yaw);
    ptsx.push_back(prev_car_x);
    ptsy.push_back(prev_car_y);
    ptsx.push_back(v_ctx.car_x);
    ptsy.push_back(v_ctx.car_y);
  } else {
    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];
    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    ptsx.push_back(ref_x_prev);
    ptsy.push_back(ref_y_prev);
    ptsx.push_back(ref_x + .001); // fix zero velocity car
    ptsy.push_back(ref_y);
  }

  vector<double> mwx = *(this->v_ctx.map_waypoints_x);
  vector<double> mwy = *(this->v_ctx.map_waypoints_y);
  vector<double> mws = *(this->v_ctx.map_waypoints_s);

  vector<double> next_wp0, next_wp1, next_wp2, next_wp3, next_wp4;

if (traj.s == LCL || traj.s == LCR) {
    // next_wp0 = getXY(car_s + 15, desired_d, mws, mwx, mwy);
    // next_wp1 = getXY(car_s + 30, desired_d, mws, mwx, mwy);
    // next_wp2 = getXY(car_s + 45, desired_d, mws, mwx, mwy);
    next_wp0 = parabolicGetXY(car_s + 25, traj.desired_d, mws, mwx, mwy);
    next_wp1 = parabolicGetXY(car_s + 50, traj.desired_d, mws, mwx, mwy);
    next_wp2 = parabolicGetXY(car_s + 75, traj.desired_d, mws, mwx, mwy);
}
// else if (desired_vel < 20) {
// //     next_wp0 = parabolicGetXY(car_s + 10, desired_d, mws, mwx, mwy);
// //     next_wp1 = parabolicGetXY(car_s + 20, desired_d, mws, mwx, mwy);
// //     next_wp2 = parabolicGetXY(car_s + 30, desired_d, mws, mwx, mwy);
// // }
else  {
    next_wp0 = parabolicGetXY(car_s + 10, traj.desired_d, mws, mwx, mwy);
    next_wp1 = parabolicGetXY(car_s + 20, traj.desired_d, mws, mwx, mwy);
    next_wp2 = parabolicGetXY(car_s + 30, traj.desired_d, mws, mwx, mwy);
}
    // next_wp3 = parabolicGetXY(car_s + 80, desired_d, mws, mwx, mwy);
    // next_wp4 = parabolicGetXY(car_s + 100, desired_d, mws, mwx, mwy);

  ptsx.push_back(next_wp0[0]);
  ptsy.push_back(next_wp0[1]);
  ptsx.push_back(next_wp1[0]);
  ptsy.push_back(next_wp1[1]);
  ptsx.push_back(next_wp2[0]);
  ptsy.push_back(next_wp2[1]);
  // ptsx.push_back(next_wp3[0]);
  // ptsy.push_back(next_wp3[1]);
  // ptsx.push_back(next_wp4[0]);
  // ptsy.push_back(next_wp4[1]);

  // Remap coordinates to car reference frame
  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    double psi = 0 - ref_yaw;
    ptsx[i] = shift_x * cos(psi) - shift_y * sin(psi);
    ptsy[i] = shift_x * sin(psi) + shift_y * cos(psi);
  }

  // for(int i = 0; i < ptsx.size(); i++) {
  //   fprintf(stderr, "[%f, %f]", ptsx[i], ptsy[i]);
  // }
  // fprintf(stderr, "\n");

  tk::spline spl;

  // quick and dirty sort spline points
  for (int i = 0; i < ptsx.size(); i++) {
    for (int j = i + 1; j < ptsx.size(); j++) {
      if (ptsx[j] < ptsx[i]) {
        double tmp = ptsx[j];
        ptsx[j] = ptsx[i];
        ptsx[i] = tmp;
        tmp = ptsy[j];
        ptsy[j] = ptsy[i];
        ptsy[i] = tmp;
      }
    }
  }

  spl.set_points(ptsx, ptsy);

  for (int i = 0; i < prev_size; i++) {
    traj.path.push_back(previous_path_x[i]);
    traj.path.push_back(previous_path_y[i]);
  }

  double target_x = 30.0;
  double target_y = spl(target_x);
  double target_dist = sqrt(target_x * target_x + target_y * target_y);
  double x_add_on = 0.0;
  double N = target_dist / (0.02 * speed);

  for (int i = 0; i <= kSTEPS - prev_size; i++) {
    double x_point = x_add_on + target_x / N;
    double y_point = spl(x_point);
    //fprintf(stderr, "point[%f, %f]\n", x_point, y_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    // Back to world coordiantes
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

    x_point += ref_x;
    y_point += ref_y;

    traj.path.push_back(x_point);
    traj.path.push_back(y_point);
  }
}

void Planner::GenerateKLTrajectory(vector<Traj> &t) {
  for (int i = -2; i < 3; i++) {
    // for (int j = -3; j < 4; j++) {
      Traj traj;
      traj.desired_d = 2 + 4 * desired_lane;// + .2 * j;
      double v = desired_vel + kVDELTA * i;
      traj.velocity = v;
      traj.s = KL;
      traj.target_lane = desired_lane;
      GenerateTrajectory(traj);
      t.push_back(traj);
    // }
  }
}

void Planner::GeneratePLCLTrajectory(vector<Traj> &t) {
  for (int i = -2; i < 3; i++) {
    // for (int j = -2; j < 3; j++) {
      Traj traj;
      traj.desired_d = 2 + 4 * desired_lane;// + .1 * j;
      double v = desired_vel + kVDELTA * i;
      traj.velocity = v;
      traj.s = PLCL;
      traj.target_lane = desired_lane - 1;
      GenerateTrajectory(traj);
      t.push_back(traj);
    // }
  }
}

void Planner::GeneratePLCRTrajectory(vector<Traj> &t) {
  for (int i = -2; i < 3; i++) {
    // for (int j = -2; j < 3; j++) {
      Traj traj;
      traj.desired_d = 2 + 4 * desired_lane;// + .1 * j;
      double v = desired_vel + kVDELTA * i;
      traj.velocity = v;
      traj.s = PLCR;
      traj.target_lane = desired_lane + 1;
      GenerateTrajectory(traj);
      t.push_back(traj);
    // }
  }
}

void Planner::GenerateLCLTrajectory(vector<Traj> &t) {
  for (int i = -2; i < 3; i++) {
    //for (int j = 0; j < 10; j++) {
      Traj traj;
      traj.desired_d = 2 + 4 * target_lane;// + .1 * j;
      double v = desired_vel + kVDELTA * i;
      traj.velocity = v;
      traj.s = LCL;
      traj.target_lane = target_lane;
      GenerateTrajectory(traj);
      t.push_back(traj);
    //}
  }
}

void Planner::GenerateLCRTrajectory(vector<Traj> &t) {
  for (int i = -2; i < 3; i++) {
    //for (int j = 0; j < 1; j++) {
      Traj traj;
      traj.desired_d = 2 + 4 * target_lane;// - .1 * j;
      double v = desired_vel + kVDELTA * i;
      traj.velocity = v;
      traj.s = LCR;
      traj.target_lane = target_lane;
      GenerateTrajectory(traj);
      t.push_back(traj);
    //}
  }
}

bool Planner::TargetLaneReached() {
  if (target_lane * 4 + 1 < v_ctx.car_d && target_lane * 4 + 3 > v_ctx.car_d) {
    return true;
  }
  return false;
}

void Planner::UpdateLaneSpeeds(const int steps, const double car_s,
                               const vector<vector<double>> &sensor_fusion) {
  for (int j = 0; j < 3; j++) {
    this->lane_speed[j] = this->speed_limit;
    for (int i = 0; i < sensor_fusion.size(); i++) {
      float d = sensor_fusion[i][6];
      // anywhere in the jth lane
      if (d < j * 4 + 4 && d > j * 4) {
        double vx = sensor_fusion[i][3];
        double vy = sensor_fusion[i][4];
        double speed = sqrt(vx * vx + vy * vy);
        double s = sensor_fusion[i][5];
        // move it forward in time to where we start the new trajectory
        //s += (double)steps * kSEC_PER_STEP * speed;
        if (s >= car_s - 5 && s < car_s + kSTEPS) {
          if (this->lane_speed[j] > speed) {
            this->lane_speed[j] = speed;
          }
        }
      }
    }
  }
}

planner_state_t Planner::UpdateFSM(vector<Traj> &t, int &index) {

  if (this->current_state == READY) {
    this->current_state = KL;
  }

  switch (this->current_state) {
  case KL: {
    target_lane = desired_lane;
    GenerateKLTrajectory(t);
    // Check availible state transitions KL PLCL PLCR
    if (desired_lane > 0)
      GeneratePLCLTrajectory(t);
    if (desired_lane < 2)
      GeneratePLCRTrajectory(t);
    break;
  }

  case PLCL: {
    // Check availible state transitions KL PLCL LCL
    target_lane = desired_lane - 1;
    GeneratePLCLTrajectory(t);
    GenerateLCLTrajectory(t);
    GenerateKLTrajectory(t);
    break;
  }

  case PLCR: {
    // Check availible state transitions KL PLCR LCR
    target_lane = desired_lane + 1;
    GeneratePLCRTrajectory(t);
    GenerateLCRTrajectory(t);
    GenerateKLTrajectory(t);
    break;
  }

  case LCL: {
    // Check availible state transitions KL LCL
    if (TargetLaneReached()) {
      desired_lane = target_lane;
      GenerateKLTrajectory(t);
    } else {
      GenerateLCLTrajectory(t);
      GenerateKLTrajectory(t);
    }
    break;
  }

  case LCR: {
    // Check availible state transitions KL LCR
    if (TargetLaneReached()) {
      desired_lane = target_lane;
      GenerateKLTrajectory(t);
    } else {
      GenerateLCRTrajectory(t);
      GenerateKLTrajectory(t);
    }
    break;
  }
  }

  index = LowestCost(t);
  this->current_state = t[index].s;

  return this->current_state;
}

void Planner::UpdateSensorFusion(vector<vector<double>> &pred_s, vector<vector<double>> &pred_d) {
  vector<vector<double>> sf = *(v_ctx.sensor_fusion);
  double prev_size = v_ctx.previous_path_x->size();
  for (int i = 0; i < sf.size(); i++) {
    double d = sf[i][6];
    double vx = sf[i][3];
    double vy = sf[i][4];
    double speed = sqrt(vx * vx + vy * vy);
    double s_off = sf[i][5];
    vector<double> vec_d;
    vector<double> vec_s;
    for ( int j = 0; j < kSTEPS; j++) {
      vec_d.push_back(d); // assume stays in lane
      vec_s.push_back(s_off + i * kSEC_PER_STEP * speed);
    }
    pred_s.push_back(vec_s);
    pred_d.push_back(vec_d);
  }
}

double Planner::TotalCost(Traj &traj, const vector<vector<double>> &pred_s, const vector<vector<double>> &pred_d,
                          const vector<double> &s, const vector<double> &d) {
  double val = 0.f;
  int final_lane = desired_lane;
  if (traj.s == LCL || traj.s == LCR ) {
    final_lane = traj.target_lane;
  }
  traj.cost[0] = InefficiencyCost(speed_limit, traj.target_lane, final_lane,
                         lane_speed) * 3;
  traj.cost[1] = InefficiencyCost(speed_limit, traj.target_lane, final_lane,
                         lane_speed) * 3;
  traj.cost[2] = VelocityEffeciencyCost(s, speed_limit, kSEC_PER_STEP) * .1; //.1
  traj.cost[3] = VelocityLegalityCost(s, speed_limit, kSEC_PER_STEP) * .1; //.1
  traj.cost[4] = PositionComfortCost(d, final_lane * 4 + 2) * 2; // .5
  traj.cost[5] = PositionFeasibilityCost(pred_s, pred_d, s, d, kCOLLISION_DIST) * 10;
  traj.cost[6] = PositionSafetyCost2(pred_s, pred_d, s, d, kBUFFER_DIST);
  traj.cost[7] = AccelerationFeasibilityCost(s, d, kMAX_ACCEL, kSEC_PER_STEP) * 1;
  traj.cost[8] = AccelerationComfortCost(s, d, kMAX_JERK, kSEC_PER_STEP) * 1;
  //traj.cost[9] = NormalAccCost(d, kMAX_ACCEL, kSEC_PER_STEP) * 1;
  for (int i = 0; i < 10; i++) {
    val += traj.cost[i];
  }
  //fprintf(stderr, "val[%f] ", val);
  return val;
}

void Planner::GetFrenetTraj(const Traj &traj, vector<double> &s,
                            vector<double> &d) {
  for (int i = 0; i < traj.path.size() / 2; i++) {
    vector<double> xy =
        getFrenet(traj.path[i * 2], traj.path[i * 2 + 1], deg2rad(v_ctx.car_yaw),
                  *(this->v_ctx.map_waypoints_x), *(this->v_ctx.map_waypoints_y));
    s.push_back(xy[0]);
    d.push_back(xy[1]);
  }
}

int Planner::LowestCost(vector<Traj> &t) {

  int index = 0;
  double lowest_cost = 1e2;
  vector<vector<double>> sfs;
  vector<vector<double>> sfd;

  // predict other vehicle positions
  UpdateSensorFusion(sfs, sfd);

  // Calculate each trajectory cost and assign lowest index
  for (int i = 0; i < t.size(); i++) {
    vector<double> s;
    vector<double> d;
    GetFrenetTraj(t[i], s, d);
    double c = TotalCost(t[i], sfs, sfd, s, d);
    t[i].total_cost = c;
    if (c < lowest_cost) {
      lowest_cost = c;
      index = i;
    }
  }

  // DEBUG
  vector<vector<double>> eff;
  for (int j = 0; j < 6; j++) {
    vector<double> x;
    for (int i = 0; i < t.size(); i++) {
      if (t[i].s == j) {
        x.push_back(t[i].total_cost);
      }
    }
    eff.push_back(x);
  }

  double costs[6];
  for (int i = 1; i < 6; i++) {
    double low = 1e2;
    for (int j = 0; j < eff[i].size(); j++) {
      if (eff[i][j] < low)
        low = eff[i][j];
    }
    costs[i] = low;
  }
  for (int i = 0; i < 10; i++) {
    fprintf(stderr, "%d[%.3f] ", i, t[index].cost[i]);
  }
  fprintf(stderr, "state[%d] cost[%f] %.2f\n", t[index].s, t[index].total_cost, t[index].desired_d);
  // fprintf(stderr,"costs 1[%.3f] 2[%.3f] 3[%.3f] 4[%.3f] 5[%.3f] state[%d]\n", costs[1], costs[2], costs[3], costs[4], costs[5], t[index].s);
  // fprintf(stderr, "lowest cost[%f] index[%d] state[%d]\n", lowest_cost, index, t[index].s);
  return index;
}

void Planner::GetTrajectory(vector<double> &next_x_vals,
                            vector<double> &next_y_vals, PlannerState &v_ctx) {

  this->v_ctx = v_ctx;
  vector<vector<double>> sf = *(this->v_ctx.sensor_fusion);
  vector<double> previous_path_x = *(this->v_ctx.previous_path_x);
  vector<double> previous_path_y = *(this->v_ctx.previous_path_y);

  // last list of path points from simulator
  int prev_size = previous_path_x.size();

  UpdateLaneSpeeds(prev_size, v_ctx.car_s, sf);
//  fprintf(stderr, "lane[%f, %f, %f] ", lane_speed[0], lane_speed[1], lane_speed[2]);
  UpdateVelocity(); // Adaptive Cruise control sets the target velocity

  int index = 0;
  vector<Traj> t;
  UpdateFSM(t, index);

  // Update velocity step with actual lowest cost trajectory velocity
  desired_vel = t[index].velocity;

  for (int i = 0; i < 50; i++) {
    next_x_vals.push_back(t[index].path[i * 2]);
    next_y_vals.push_back(t[index].path[i * 2 + 1]);
  }
}

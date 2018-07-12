#ifndef HELPER_H
#define HELPER_H

#include <cmath>
#include <vector>

namespace {
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {

  double closestLen = 100000; // large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); i++) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x, y, map_x, map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {

  int closestWaypoint = ClosestWaypoint(x, y, maps_x, maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y - y), (map_x - x));

  double angle = fabs(theta - heading);
  angle = min(2 * pi() - angle, angle);

  if (angle > pi() / 4) {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading =
      atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x, y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if (next_wp == 0) {
    prev_wp = maps_x.size() - 1;
  }

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  // see if d value is positive or negative by comparing it to a center point

  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++) {
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1]);
  }

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// ADDITIONAL HELPER FUNCTIONS Taken from https://github.com/Paionir/Path-planning/blob/master/src/main.cpp

double calcPoly (std::vector<double> coeffs, double t) 
{
  // This functions calculates the value of a polynomial at time t and returns its value

  //INPUT:
  // a vector of all coefficients for the polynomial sorted from lowest degree to highest
  // the time t at which evaluate the polynomial

  //OUTPUT:
  // the value of the polynomial at time t
  
  double pol = 0.;
  for (int i = 0; i < coeffs.size(); i++)
  {
    pol += coeffs[i] * pow(t, i);
  }
  return pol;
}


std::vector<double> parabolicInterpol(std::vector<double> X, std::vector<double> Y, int center, double ds, double d) 
{
  // This functions interpolates a 2nd grade polynomial between 3 waypoints X,Y and then uses ds and d to estimate the x,y position
  // of a point between the center waypoint and the next

  //INPUT:
  // vector of X coordinates of 3 waypoints
  // vector of Y coordinates of 3 waypoints
  // index of the central waypoint
  // arc lenght along the parabola ds
  // coordinate d 

  //OUTPUT:
  // a vector of (x,y) coordinate for point (ds,d)



  // transform to reference system of center point
  double x0 = X[center-1] - X[center]; 
  double x1 = X[center] - X[center];
  double x2 = X[center+1] - X[center];
  
  double y0 = Y[center-1] - Y[center];
  double y1 = Y[center] - Y[center];
  double y2 = Y[center+1] - Y[center];
    
  double den_X = (x0-x1)*(x0-x2)*(x1-x2);
  double den_y = (y0-y1)*(y0-y2)*(y1-y2);
  double disc_x = (x0-x1)*(x1-x2);
  double disc_y = (y0-y1)*(y1-y2);
  bool rotate = false;
  
  if (disc_x <= 0 )  
  {
    //rotate reference system, so that (x,y) -> (-y,x)
 
  double tx0 = -y0;
  double tx1 = -y1;
  double tx2 = -y2;
  
  y0 = x0;
  y1 = x1;
  y2 = x2;

  x0 = tx0;
  x1 = tx1;
  x2 = tx2;

  std::vector<double> TX;

  for (int i =0;i<X.size();i++)
  {
    TX.push_back(-Y[i]);
    Y[i]=X[i];
    X[i]=TX[i];
  }

  rotate = true;

  }

  // Calculate 3 parameters of the parabola passing by the 3 waypoints y=ax^2+bx+c
  double den = (x0-x1)*(x0-x2)*(x1-x2);
  double a = ( x2*(y1-y0) + x1*(y0-y2) + x0*(y2-y1) )/den;
  double b = ( x2*x2*(y0-y1) + x1*x1*(y2-y0) +x0*x0*(y1-y2) )/den;
  double c = ( x1*x2*(x1-x2)*y0 + x2*x0*(x2-x0)*y1 +x0*x1*(x0-x1)*y2 )/den;
  
  
  double sum = 0.;
  int idx = 0;

  double X1 = X[center]-X[center]; // transform to reference system of center point
  double X2 = X[center+abs(ds)/ds]-X[center]; // second integration limit is the previous or successive point of center, according to ds sign    
  
  double h = (X2-X1)/50.;

  // the arc lenght of a parabola is the definite integral of sqrt(1+f'(x)^2) in dx
  double u1 = 2.*a*X1 +b; // helper variable 
  double g1 = u1*sqrt(1+u1*u1) + log(abs(sqrt(1+u1*u1) + u1)); // primitive of sqrt(1+f'(x)^2) calculated in X1
  
  double xe2=X1;

  // EVALUATE xe2 at which the arc lenght equals |ds| with 1e-11 tolerance or with 10000000 max iterations, whatever happens first
  while (( abs(abs(ds) - sum) > 1e-11) && (idx < 10000000))
  {
    
    xe2 += h;
    double u2 = 2.*a*xe2 + b;
    double g2 = (u2*sqrt(1+u2*u2) + log(abs(sqrt(1+u2*u2) + u2))); // primitive of sqrt(1+f'(x)^2) calculated in xe2
    
    sum = abs((g2 - g1)/(4.*a)); // arc lenght from X1 to xe2
    if (sum > abs(ds) ) // if arc lenght is greater than |ds| go back one step and divide h by 2
    {
      xe2 -= h;  
      h = h/2.;  
    }
    idx++;
  }
  double xp = xe2;
  double yp = calcPoly({c,b,a},xp);
  double heading = atan2(2.*a*xp + b, 1.); //calculate heading of parabola at point (xp, yp=2axp+b)
 
  // transform back to global reference system
  xp += X[center];
  yp += Y[center];

  if (rotate)
  {
    //rotate back
    double txp= xp;
    xp = yp;
    yp = -txp;


    if (x1-x0 > 0.)
    {
      heading = heading + pi();
    }

    // add d offset using heading
    
    xp += d * cos(heading);
    yp += d * sin(heading);


  } else {
      

    if (x1-x0 < 0.)
    {
      heading = heading + pi();
     }
    heading = heading-pi()/2.;
    xp += d * cos(heading);
    yp += d * sin(heading);  
  }

  return{xp,yp};
}


vector<double> parabolicGetXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  // This functions transform from s,d coordinates to global x,y coordinates using a waypoint maps of the highway
  // Instead of a linear interpolation, it uses two parabolic interpolation and then calculates a weighted mean. 
  // The first interpolation is made using the previous waypoint and the immidiately successive and previous waypoint,
  // the second interpolation is made using the previous waypoint and the immidiately 2 successive waypoints.
  // Then a weighted mean of the two points is calculated using, as weights, the inverse of the squared distance from the 
  // previous waypoint and the next waypoint

  //INPUT:
  // s coordinate
  // d coordinate
  // s values of waypoints
  // x values of waypoints
  // y values of waypoints

  //OUTPUT:
  // a vector of (x,y) coordinate for point (s,d)

  double max_s = 6945.554; //max s value for waypoints
  while (s > max_s)
  {
    s -= max_s;
  }

  int prev_wp = -1;

  // find the previous waypoint

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

 
  int cubic_num; //helper index of the previous waypoint

  vector<double> X; // X coordinates of nearest waypoints used for interpolation
  vector<double> Y; // Y coordinates of nearest waypoints used for interpolation

  // fill X and Y with 1 previous waypoint and 2 successive waypoints,
  // if previous waypoint is 0 then start from the last waypoint in the map

  if (prev_wp >=1) 
  {
    cubic_num = 1;
    for (int i = -1; i < 3; i++)
    {
      X.push_back( maps_x[(prev_wp + i)%maps_x.size()] );
      Y.push_back( maps_y[(prev_wp + i)%maps_x.size()] );
    }
  } 
  else 
  {
    cubic_num = 1;
    for (int i = maps_x.size() -1 ; i < maps_x.size() + 3; i++)
    {
      X.push_back( maps_x[i%maps_x.size()] );
      Y.push_back( maps_y[i%maps_x.size()] );
    }
  }

  double ds_p = s - maps_s[prev_wp]; //distance in s from previous waypoint

  std::vector<double> XYp = parabolicInterpol(X,Y, cubic_num, ds_p, d); // calculate x,y using the previous waypoint as the central waypoint for interpolation
  
  double ds_s; // distance in s from the next waypoint
  if (prev_wp == maps_s.size() - 1 )
  {
    ds_s = s - max_s;  
  }
  else
  {
    ds_s = s - maps_s[(prev_wp+1)];  
  }
  

  std::vector<double> XYs = parabolicInterpol(X,Y, cubic_num+1, ds_s, d); // calculate x,y using the next waypoint as the central waypoint for interpolation

  // calculate the weighted mean of the two interpolations using the inverse sqaure of the distance from previous and next waypoint
  int n_exp=-2;
  double p1 = pow(ds_p,n_exp);
  double p2 = pow(ds_s,n_exp);
  double norm =p1+p2;
  double x = (XYp[0]*p1 + XYs[0]*p2)/(norm);
  double y = (XYp[1]*p1 + XYs[1]*p2)/(norm);  
 
  return {x,y};
   
}  


}

#endif // HELPER_H
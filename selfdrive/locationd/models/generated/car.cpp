#include "car.h"

namespace {
#define DIM 8
#define EDIM 8
#define MEDIM 8
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 5.991464547107981;

/******************************************************************************
 *                       Code generated with sympy 1.8                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3006024959811024317) {
   out_3006024959811024317[0] = delta_x[0] + nom_x[0];
   out_3006024959811024317[1] = delta_x[1] + nom_x[1];
   out_3006024959811024317[2] = delta_x[2] + nom_x[2];
   out_3006024959811024317[3] = delta_x[3] + nom_x[3];
   out_3006024959811024317[4] = delta_x[4] + nom_x[4];
   out_3006024959811024317[5] = delta_x[5] + nom_x[5];
   out_3006024959811024317[6] = delta_x[6] + nom_x[6];
   out_3006024959811024317[7] = delta_x[7] + nom_x[7];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6749470672304064626) {
   out_6749470672304064626[0] = -nom_x[0] + true_x[0];
   out_6749470672304064626[1] = -nom_x[1] + true_x[1];
   out_6749470672304064626[2] = -nom_x[2] + true_x[2];
   out_6749470672304064626[3] = -nom_x[3] + true_x[3];
   out_6749470672304064626[4] = -nom_x[4] + true_x[4];
   out_6749470672304064626[5] = -nom_x[5] + true_x[5];
   out_6749470672304064626[6] = -nom_x[6] + true_x[6];
   out_6749470672304064626[7] = -nom_x[7] + true_x[7];
}
void H_mod_fun(double *state, double *out_1876413921371897227) {
   out_1876413921371897227[0] = 1.0;
   out_1876413921371897227[1] = 0.0;
   out_1876413921371897227[2] = 0.0;
   out_1876413921371897227[3] = 0.0;
   out_1876413921371897227[4] = 0.0;
   out_1876413921371897227[5] = 0.0;
   out_1876413921371897227[6] = 0.0;
   out_1876413921371897227[7] = 0.0;
   out_1876413921371897227[8] = 0.0;
   out_1876413921371897227[9] = 1.0;
   out_1876413921371897227[10] = 0.0;
   out_1876413921371897227[11] = 0.0;
   out_1876413921371897227[12] = 0.0;
   out_1876413921371897227[13] = 0.0;
   out_1876413921371897227[14] = 0.0;
   out_1876413921371897227[15] = 0.0;
   out_1876413921371897227[16] = 0.0;
   out_1876413921371897227[17] = 0.0;
   out_1876413921371897227[18] = 1.0;
   out_1876413921371897227[19] = 0.0;
   out_1876413921371897227[20] = 0.0;
   out_1876413921371897227[21] = 0.0;
   out_1876413921371897227[22] = 0.0;
   out_1876413921371897227[23] = 0.0;
   out_1876413921371897227[24] = 0.0;
   out_1876413921371897227[25] = 0.0;
   out_1876413921371897227[26] = 0.0;
   out_1876413921371897227[27] = 1.0;
   out_1876413921371897227[28] = 0.0;
   out_1876413921371897227[29] = 0.0;
   out_1876413921371897227[30] = 0.0;
   out_1876413921371897227[31] = 0.0;
   out_1876413921371897227[32] = 0.0;
   out_1876413921371897227[33] = 0.0;
   out_1876413921371897227[34] = 0.0;
   out_1876413921371897227[35] = 0.0;
   out_1876413921371897227[36] = 1.0;
   out_1876413921371897227[37] = 0.0;
   out_1876413921371897227[38] = 0.0;
   out_1876413921371897227[39] = 0.0;
   out_1876413921371897227[40] = 0.0;
   out_1876413921371897227[41] = 0.0;
   out_1876413921371897227[42] = 0.0;
   out_1876413921371897227[43] = 0.0;
   out_1876413921371897227[44] = 0.0;
   out_1876413921371897227[45] = 1.0;
   out_1876413921371897227[46] = 0.0;
   out_1876413921371897227[47] = 0.0;
   out_1876413921371897227[48] = 0.0;
   out_1876413921371897227[49] = 0.0;
   out_1876413921371897227[50] = 0.0;
   out_1876413921371897227[51] = 0.0;
   out_1876413921371897227[52] = 0.0;
   out_1876413921371897227[53] = 0.0;
   out_1876413921371897227[54] = 1.0;
   out_1876413921371897227[55] = 0.0;
   out_1876413921371897227[56] = 0.0;
   out_1876413921371897227[57] = 0.0;
   out_1876413921371897227[58] = 0.0;
   out_1876413921371897227[59] = 0.0;
   out_1876413921371897227[60] = 0.0;
   out_1876413921371897227[61] = 0.0;
   out_1876413921371897227[62] = 0.0;
   out_1876413921371897227[63] = 1.0;
}
void f_fun(double *state, double dt, double *out_7267003510127361778) {
   out_7267003510127361778[0] = state[0];
   out_7267003510127361778[1] = state[1];
   out_7267003510127361778[2] = state[2];
   out_7267003510127361778[3] = state[3];
   out_7267003510127361778[4] = state[4];
   out_7267003510127361778[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7267003510127361778[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7267003510127361778[7] = state[7];
}
void F_fun(double *state, double dt, double *out_4507925919255265520) {
   out_4507925919255265520[0] = 1;
   out_4507925919255265520[1] = 0;
   out_4507925919255265520[2] = 0;
   out_4507925919255265520[3] = 0;
   out_4507925919255265520[4] = 0;
   out_4507925919255265520[5] = 0;
   out_4507925919255265520[6] = 0;
   out_4507925919255265520[7] = 0;
   out_4507925919255265520[8] = 0;
   out_4507925919255265520[9] = 1;
   out_4507925919255265520[10] = 0;
   out_4507925919255265520[11] = 0;
   out_4507925919255265520[12] = 0;
   out_4507925919255265520[13] = 0;
   out_4507925919255265520[14] = 0;
   out_4507925919255265520[15] = 0;
   out_4507925919255265520[16] = 0;
   out_4507925919255265520[17] = 0;
   out_4507925919255265520[18] = 1;
   out_4507925919255265520[19] = 0;
   out_4507925919255265520[20] = 0;
   out_4507925919255265520[21] = 0;
   out_4507925919255265520[22] = 0;
   out_4507925919255265520[23] = 0;
   out_4507925919255265520[24] = 0;
   out_4507925919255265520[25] = 0;
   out_4507925919255265520[26] = 0;
   out_4507925919255265520[27] = 1;
   out_4507925919255265520[28] = 0;
   out_4507925919255265520[29] = 0;
   out_4507925919255265520[30] = 0;
   out_4507925919255265520[31] = 0;
   out_4507925919255265520[32] = 0;
   out_4507925919255265520[33] = 0;
   out_4507925919255265520[34] = 0;
   out_4507925919255265520[35] = 0;
   out_4507925919255265520[36] = 1;
   out_4507925919255265520[37] = 0;
   out_4507925919255265520[38] = 0;
   out_4507925919255265520[39] = 0;
   out_4507925919255265520[40] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4507925919255265520[41] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4507925919255265520[42] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4507925919255265520[43] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4507925919255265520[44] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4507925919255265520[45] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4507925919255265520[46] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4507925919255265520[47] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4507925919255265520[48] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4507925919255265520[49] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4507925919255265520[50] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4507925919255265520[51] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4507925919255265520[52] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4507925919255265520[53] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4507925919255265520[54] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4507925919255265520[55] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4507925919255265520[56] = 0;
   out_4507925919255265520[57] = 0;
   out_4507925919255265520[58] = 0;
   out_4507925919255265520[59] = 0;
   out_4507925919255265520[60] = 0;
   out_4507925919255265520[61] = 0;
   out_4507925919255265520[62] = 0;
   out_4507925919255265520[63] = 1;
}
void h_25(double *state, double *unused, double *out_1777083439914440830) {
   out_1777083439914440830[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5796497272403601369) {
   out_5796497272403601369[0] = 0;
   out_5796497272403601369[1] = 0;
   out_5796497272403601369[2] = 0;
   out_5796497272403601369[3] = 0;
   out_5796497272403601369[4] = 0;
   out_5796497272403601369[5] = 0;
   out_5796497272403601369[6] = 1;
   out_5796497272403601369[7] = 0;
}
void h_24(double *state, double *unused, double *out_1016139534714745614) {
   out_1016139534714745614[0] = state[4];
   out_1016139534714745614[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2660528953614973373) {
   out_2660528953614973373[0] = 0;
   out_2660528953614973373[1] = 0;
   out_2660528953614973373[2] = 0;
   out_2660528953614973373[3] = 0;
   out_2660528953614973373[4] = 1;
   out_2660528953614973373[5] = 0;
   out_2660528953614973373[6] = 0;
   out_2660528953614973373[7] = 0;
   out_2660528953614973373[8] = 0;
   out_2660528953614973373[9] = 0;
   out_2660528953614973373[10] = 0;
   out_2660528953614973373[11] = 0;
   out_2660528953614973373[12] = 0;
   out_2660528953614973373[13] = 1;
   out_2660528953614973373[14] = 0;
   out_2660528953614973373[15] = 0;
}
void h_30(double *state, double *unused, double *out_326452047285557663) {
   out_326452047285557663[0] = state[4];
}
void H_30(double *state, double *unused, double *out_8215759021529950039) {
   out_8215759021529950039[0] = 0;
   out_8215759021529950039[1] = 0;
   out_8215759021529950039[2] = 0;
   out_8215759021529950039[3] = 0;
   out_8215759021529950039[4] = 1;
   out_8215759021529950039[5] = 0;
   out_8215759021529950039[6] = 0;
   out_8215759021529950039[7] = 0;
}
void h_26(double *state, double *unused, double *out_4739965471432785060) {
   out_4739965471432785060[0] = state[7];
}
void H_26(double *state, double *unused, double *out_56703755723585689) {
   out_56703755723585689[0] = 0;
   out_56703755723585689[1] = 0;
   out_56703755723585689[2] = 0;
   out_56703755723585689[3] = 0;
   out_56703755723585689[4] = 0;
   out_56703755723585689[5] = 0;
   out_56703755723585689[6] = 0;
   out_56703755723585689[7] = 1;
}
void h_27(double *state, double *unused, double *out_6778756707575286475) {
   out_6778756707575286475[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5104983626382207223) {
   out_5104983626382207223[0] = 0;
   out_5104983626382207223[1] = 0;
   out_5104983626382207223[2] = 0;
   out_5104983626382207223[3] = 1;
   out_5104983626382207223[4] = 0;
   out_5104983626382207223[5] = 0;
   out_5104983626382207223[6] = 0;
   out_5104983626382207223[7] = 0;
}
void h_29(double *state, double *unused, double *out_5328125314946403308) {
   out_5328125314946403308[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7641200407674413864) {
   out_7641200407674413864[0] = 0;
   out_7641200407674413864[1] = 1;
   out_7641200407674413864[2] = 0;
   out_7641200407674413864[3] = 0;
   out_7641200407674413864[4] = 0;
   out_7641200407674413864[5] = 0;
   out_7641200407674413864[6] = 0;
   out_7641200407674413864[7] = 0;
}
void h_28(double *state, double *unused, double *out_5225526501588038331) {
   out_5225526501588038331[0] = state[5];
   out_5225526501588038331[1] = state[6];
}
void H_28(double *state, double *unused, double *out_4370025258873715249) {
   out_4370025258873715249[0] = 0;
   out_4370025258873715249[1] = 0;
   out_4370025258873715249[2] = 0;
   out_4370025258873715249[3] = 0;
   out_4370025258873715249[4] = 0;
   out_4370025258873715249[5] = 1;
   out_4370025258873715249[6] = 0;
   out_4370025258873715249[7] = 0;
   out_4370025258873715249[8] = 0;
   out_4370025258873715249[9] = 0;
   out_4370025258873715249[10] = 0;
   out_4370025258873715249[11] = 0;
   out_4370025258873715249[12] = 0;
   out_4370025258873715249[13] = 0;
   out_4370025258873715249[14] = 1;
   out_4370025258873715249[15] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3006024959811024317) {
  err_fun(nom_x, delta_x, out_3006024959811024317);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6749470672304064626) {
  inv_err_fun(nom_x, true_x, out_6749470672304064626);
}
void car_H_mod_fun(double *state, double *out_1876413921371897227) {
  H_mod_fun(state, out_1876413921371897227);
}
void car_f_fun(double *state, double dt, double *out_7267003510127361778) {
  f_fun(state,  dt, out_7267003510127361778);
}
void car_F_fun(double *state, double dt, double *out_4507925919255265520) {
  F_fun(state,  dt, out_4507925919255265520);
}
void car_h_25(double *state, double *unused, double *out_1777083439914440830) {
  h_25(state, unused, out_1777083439914440830);
}
void car_H_25(double *state, double *unused, double *out_5796497272403601369) {
  H_25(state, unused, out_5796497272403601369);
}
void car_h_24(double *state, double *unused, double *out_1016139534714745614) {
  h_24(state, unused, out_1016139534714745614);
}
void car_H_24(double *state, double *unused, double *out_2660528953614973373) {
  H_24(state, unused, out_2660528953614973373);
}
void car_h_30(double *state, double *unused, double *out_326452047285557663) {
  h_30(state, unused, out_326452047285557663);
}
void car_H_30(double *state, double *unused, double *out_8215759021529950039) {
  H_30(state, unused, out_8215759021529950039);
}
void car_h_26(double *state, double *unused, double *out_4739965471432785060) {
  h_26(state, unused, out_4739965471432785060);
}
void car_H_26(double *state, double *unused, double *out_56703755723585689) {
  H_26(state, unused, out_56703755723585689);
}
void car_h_27(double *state, double *unused, double *out_6778756707575286475) {
  h_27(state, unused, out_6778756707575286475);
}
void car_H_27(double *state, double *unused, double *out_5104983626382207223) {
  H_27(state, unused, out_5104983626382207223);
}
void car_h_29(double *state, double *unused, double *out_5328125314946403308) {
  h_29(state, unused, out_5328125314946403308);
}
void car_H_29(double *state, double *unused, double *out_7641200407674413864) {
  H_29(state, unused, out_7641200407674413864);
}
void car_h_28(double *state, double *unused, double *out_5225526501588038331) {
  h_28(state, unused, out_5225526501588038331);
}
void car_H_28(double *state, double *unused, double *out_4370025258873715249) {
  H_28(state, unused, out_4370025258873715249);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);

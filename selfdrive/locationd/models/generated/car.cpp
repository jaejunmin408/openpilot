#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
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
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_3753227418625007558) {
   out_3753227418625007558[0] = delta_x[0] + nom_x[0];
   out_3753227418625007558[1] = delta_x[1] + nom_x[1];
   out_3753227418625007558[2] = delta_x[2] + nom_x[2];
   out_3753227418625007558[3] = delta_x[3] + nom_x[3];
   out_3753227418625007558[4] = delta_x[4] + nom_x[4];
   out_3753227418625007558[5] = delta_x[5] + nom_x[5];
   out_3753227418625007558[6] = delta_x[6] + nom_x[6];
   out_3753227418625007558[7] = delta_x[7] + nom_x[7];
   out_3753227418625007558[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6246023284876268363) {
   out_6246023284876268363[0] = -nom_x[0] + true_x[0];
   out_6246023284876268363[1] = -nom_x[1] + true_x[1];
   out_6246023284876268363[2] = -nom_x[2] + true_x[2];
   out_6246023284876268363[3] = -nom_x[3] + true_x[3];
   out_6246023284876268363[4] = -nom_x[4] + true_x[4];
   out_6246023284876268363[5] = -nom_x[5] + true_x[5];
   out_6246023284876268363[6] = -nom_x[6] + true_x[6];
   out_6246023284876268363[7] = -nom_x[7] + true_x[7];
   out_6246023284876268363[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_5326220687443925213) {
   out_5326220687443925213[0] = 1.0;
   out_5326220687443925213[1] = 0.0;
   out_5326220687443925213[2] = 0.0;
   out_5326220687443925213[3] = 0.0;
   out_5326220687443925213[4] = 0.0;
   out_5326220687443925213[5] = 0.0;
   out_5326220687443925213[6] = 0.0;
   out_5326220687443925213[7] = 0.0;
   out_5326220687443925213[8] = 0.0;
   out_5326220687443925213[9] = 0.0;
   out_5326220687443925213[10] = 1.0;
   out_5326220687443925213[11] = 0.0;
   out_5326220687443925213[12] = 0.0;
   out_5326220687443925213[13] = 0.0;
   out_5326220687443925213[14] = 0.0;
   out_5326220687443925213[15] = 0.0;
   out_5326220687443925213[16] = 0.0;
   out_5326220687443925213[17] = 0.0;
   out_5326220687443925213[18] = 0.0;
   out_5326220687443925213[19] = 0.0;
   out_5326220687443925213[20] = 1.0;
   out_5326220687443925213[21] = 0.0;
   out_5326220687443925213[22] = 0.0;
   out_5326220687443925213[23] = 0.0;
   out_5326220687443925213[24] = 0.0;
   out_5326220687443925213[25] = 0.0;
   out_5326220687443925213[26] = 0.0;
   out_5326220687443925213[27] = 0.0;
   out_5326220687443925213[28] = 0.0;
   out_5326220687443925213[29] = 0.0;
   out_5326220687443925213[30] = 1.0;
   out_5326220687443925213[31] = 0.0;
   out_5326220687443925213[32] = 0.0;
   out_5326220687443925213[33] = 0.0;
   out_5326220687443925213[34] = 0.0;
   out_5326220687443925213[35] = 0.0;
   out_5326220687443925213[36] = 0.0;
   out_5326220687443925213[37] = 0.0;
   out_5326220687443925213[38] = 0.0;
   out_5326220687443925213[39] = 0.0;
   out_5326220687443925213[40] = 1.0;
   out_5326220687443925213[41] = 0.0;
   out_5326220687443925213[42] = 0.0;
   out_5326220687443925213[43] = 0.0;
   out_5326220687443925213[44] = 0.0;
   out_5326220687443925213[45] = 0.0;
   out_5326220687443925213[46] = 0.0;
   out_5326220687443925213[47] = 0.0;
   out_5326220687443925213[48] = 0.0;
   out_5326220687443925213[49] = 0.0;
   out_5326220687443925213[50] = 1.0;
   out_5326220687443925213[51] = 0.0;
   out_5326220687443925213[52] = 0.0;
   out_5326220687443925213[53] = 0.0;
   out_5326220687443925213[54] = 0.0;
   out_5326220687443925213[55] = 0.0;
   out_5326220687443925213[56] = 0.0;
   out_5326220687443925213[57] = 0.0;
   out_5326220687443925213[58] = 0.0;
   out_5326220687443925213[59] = 0.0;
   out_5326220687443925213[60] = 1.0;
   out_5326220687443925213[61] = 0.0;
   out_5326220687443925213[62] = 0.0;
   out_5326220687443925213[63] = 0.0;
   out_5326220687443925213[64] = 0.0;
   out_5326220687443925213[65] = 0.0;
   out_5326220687443925213[66] = 0.0;
   out_5326220687443925213[67] = 0.0;
   out_5326220687443925213[68] = 0.0;
   out_5326220687443925213[69] = 0.0;
   out_5326220687443925213[70] = 1.0;
   out_5326220687443925213[71] = 0.0;
   out_5326220687443925213[72] = 0.0;
   out_5326220687443925213[73] = 0.0;
   out_5326220687443925213[74] = 0.0;
   out_5326220687443925213[75] = 0.0;
   out_5326220687443925213[76] = 0.0;
   out_5326220687443925213[77] = 0.0;
   out_5326220687443925213[78] = 0.0;
   out_5326220687443925213[79] = 0.0;
   out_5326220687443925213[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_8658820599897161958) {
   out_8658820599897161958[0] = state[0];
   out_8658820599897161958[1] = state[1];
   out_8658820599897161958[2] = state[2];
   out_8658820599897161958[3] = state[3];
   out_8658820599897161958[4] = state[4];
   out_8658820599897161958[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_8658820599897161958[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_8658820599897161958[7] = state[7];
   out_8658820599897161958[8] = state[8];
}
void F_fun(double *state, double dt, double *out_3017086349094288292) {
   out_3017086349094288292[0] = 1;
   out_3017086349094288292[1] = 0;
   out_3017086349094288292[2] = 0;
   out_3017086349094288292[3] = 0;
   out_3017086349094288292[4] = 0;
   out_3017086349094288292[5] = 0;
   out_3017086349094288292[6] = 0;
   out_3017086349094288292[7] = 0;
   out_3017086349094288292[8] = 0;
   out_3017086349094288292[9] = 0;
   out_3017086349094288292[10] = 1;
   out_3017086349094288292[11] = 0;
   out_3017086349094288292[12] = 0;
   out_3017086349094288292[13] = 0;
   out_3017086349094288292[14] = 0;
   out_3017086349094288292[15] = 0;
   out_3017086349094288292[16] = 0;
   out_3017086349094288292[17] = 0;
   out_3017086349094288292[18] = 0;
   out_3017086349094288292[19] = 0;
   out_3017086349094288292[20] = 1;
   out_3017086349094288292[21] = 0;
   out_3017086349094288292[22] = 0;
   out_3017086349094288292[23] = 0;
   out_3017086349094288292[24] = 0;
   out_3017086349094288292[25] = 0;
   out_3017086349094288292[26] = 0;
   out_3017086349094288292[27] = 0;
   out_3017086349094288292[28] = 0;
   out_3017086349094288292[29] = 0;
   out_3017086349094288292[30] = 1;
   out_3017086349094288292[31] = 0;
   out_3017086349094288292[32] = 0;
   out_3017086349094288292[33] = 0;
   out_3017086349094288292[34] = 0;
   out_3017086349094288292[35] = 0;
   out_3017086349094288292[36] = 0;
   out_3017086349094288292[37] = 0;
   out_3017086349094288292[38] = 0;
   out_3017086349094288292[39] = 0;
   out_3017086349094288292[40] = 1;
   out_3017086349094288292[41] = 0;
   out_3017086349094288292[42] = 0;
   out_3017086349094288292[43] = 0;
   out_3017086349094288292[44] = 0;
   out_3017086349094288292[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_3017086349094288292[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_3017086349094288292[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3017086349094288292[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_3017086349094288292[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_3017086349094288292[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_3017086349094288292[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_3017086349094288292[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_3017086349094288292[53] = -9.8100000000000005*dt;
   out_3017086349094288292[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_3017086349094288292[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_3017086349094288292[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3017086349094288292[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3017086349094288292[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_3017086349094288292[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_3017086349094288292[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_3017086349094288292[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_3017086349094288292[62] = 0;
   out_3017086349094288292[63] = 0;
   out_3017086349094288292[64] = 0;
   out_3017086349094288292[65] = 0;
   out_3017086349094288292[66] = 0;
   out_3017086349094288292[67] = 0;
   out_3017086349094288292[68] = 0;
   out_3017086349094288292[69] = 0;
   out_3017086349094288292[70] = 1;
   out_3017086349094288292[71] = 0;
   out_3017086349094288292[72] = 0;
   out_3017086349094288292[73] = 0;
   out_3017086349094288292[74] = 0;
   out_3017086349094288292[75] = 0;
   out_3017086349094288292[76] = 0;
   out_3017086349094288292[77] = 0;
   out_3017086349094288292[78] = 0;
   out_3017086349094288292[79] = 0;
   out_3017086349094288292[80] = 1;
}
void h_25(double *state, double *unused, double *out_7506814967177167178) {
   out_7506814967177167178[0] = state[6];
}
void H_25(double *state, double *unused, double *out_5308046335548562329) {
   out_5308046335548562329[0] = 0;
   out_5308046335548562329[1] = 0;
   out_5308046335548562329[2] = 0;
   out_5308046335548562329[3] = 0;
   out_5308046335548562329[4] = 0;
   out_5308046335548562329[5] = 0;
   out_5308046335548562329[6] = 1;
   out_5308046335548562329[7] = 0;
   out_5308046335548562329[8] = 0;
}
void h_24(double *state, double *unused, double *out_4376945197385028814) {
   out_4376945197385028814[0] = state[4];
   out_4376945197385028814[1] = state[5];
}
void H_24(double *state, double *unused, double *out_1754685561347889600) {
   out_1754685561347889600[0] = 0;
   out_1754685561347889600[1] = 0;
   out_1754685561347889600[2] = 0;
   out_1754685561347889600[3] = 0;
   out_1754685561347889600[4] = 1;
   out_1754685561347889600[5] = 0;
   out_1754685561347889600[6] = 0;
   out_1754685561347889600[7] = 0;
   out_1754685561347889600[8] = 0;
   out_1754685561347889600[9] = 0;
   out_1754685561347889600[10] = 0;
   out_1754685561347889600[11] = 0;
   out_1754685561347889600[12] = 0;
   out_1754685561347889600[13] = 0;
   out_1754685561347889600[14] = 1;
   out_1754685561347889600[15] = 0;
   out_1754685561347889600[16] = 0;
   out_1754685561347889600[17] = 0;
}
void h_30(double *state, double *unused, double *out_7231620904892661289) {
   out_7231620904892661289[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7826379294055810956) {
   out_7826379294055810956[0] = 0;
   out_7826379294055810956[1] = 0;
   out_7826379294055810956[2] = 0;
   out_7826379294055810956[3] = 0;
   out_7826379294055810956[4] = 1;
   out_7826379294055810956[5] = 0;
   out_7826379294055810956[6] = 0;
   out_7826379294055810956[7] = 0;
   out_7826379294055810956[8] = 0;
}
void h_26(double *state, double *unused, double *out_1746536873551828834) {
   out_1746536873551828834[0] = state[7];
}
void H_26(double *state, double *unused, double *out_1566543016674506105) {
   out_1566543016674506105[0] = 0;
   out_1566543016674506105[1] = 0;
   out_1566543016674506105[2] = 0;
   out_1566543016674506105[3] = 0;
   out_1566543016674506105[4] = 0;
   out_1566543016674506105[5] = 0;
   out_1566543016674506105[6] = 0;
   out_1566543016674506105[7] = 1;
   out_1566543016674506105[8] = 0;
}
void h_27(double *state, double *unused, double *out_8335523785723686787) {
   out_8335523785723686787[0] = state[3];
}
void H_27(double *state, double *unused, double *out_5651615982255386045) {
   out_5651615982255386045[0] = 0;
   out_5651615982255386045[1] = 0;
   out_5651615982255386045[2] = 0;
   out_5651615982255386045[3] = 1;
   out_5651615982255386045[4] = 0;
   out_5651615982255386045[5] = 0;
   out_5651615982255386045[6] = 0;
   out_5651615982255386045[7] = 0;
   out_5651615982255386045[8] = 0;
}
void h_29(double *state, double *unused, double *out_8178337117372557642) {
   out_8178337117372557642[0] = state[1];
}
void H_29(double *state, double *unused, double *out_8336610638370203140) {
   out_8336610638370203140[0] = 0;
   out_8336610638370203140[1] = 1;
   out_8336610638370203140[2] = 0;
   out_8336610638370203140[3] = 0;
   out_8336610638370203140[4] = 0;
   out_8336610638370203140[5] = 0;
   out_8336610638370203140[6] = 0;
   out_8336610638370203140[7] = 0;
   out_8336610638370203140[8] = 0;
}
void h_28(double *state, double *unused, double *out_3597377736216872857) {
   out_3597377736216872857[0] = state[0];
}
void H_28(double *state, double *unused, double *out_3254211621300672566) {
   out_3254211621300672566[0] = 1;
   out_3254211621300672566[1] = 0;
   out_3254211621300672566[2] = 0;
   out_3254211621300672566[3] = 0;
   out_3254211621300672566[4] = 0;
   out_3254211621300672566[5] = 0;
   out_3254211621300672566[6] = 0;
   out_3254211621300672566[7] = 0;
   out_3254211621300672566[8] = 0;
}
void h_31(double *state, double *unused, double *out_303599010191181208) {
   out_303599010191181208[0] = state[8];
}
void H_31(double *state, double *unused, double *out_940334914441154629) {
   out_940334914441154629[0] = 0;
   out_940334914441154629[1] = 0;
   out_940334914441154629[2] = 0;
   out_940334914441154629[3] = 0;
   out_940334914441154629[4] = 0;
   out_940334914441154629[5] = 0;
   out_940334914441154629[6] = 0;
   out_940334914441154629[7] = 0;
   out_940334914441154629[8] = 1;
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
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_3753227418625007558) {
  err_fun(nom_x, delta_x, out_3753227418625007558);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6246023284876268363) {
  inv_err_fun(nom_x, true_x, out_6246023284876268363);
}
void car_H_mod_fun(double *state, double *out_5326220687443925213) {
  H_mod_fun(state, out_5326220687443925213);
}
void car_f_fun(double *state, double dt, double *out_8658820599897161958) {
  f_fun(state,  dt, out_8658820599897161958);
}
void car_F_fun(double *state, double dt, double *out_3017086349094288292) {
  F_fun(state,  dt, out_3017086349094288292);
}
void car_h_25(double *state, double *unused, double *out_7506814967177167178) {
  h_25(state, unused, out_7506814967177167178);
}
void car_H_25(double *state, double *unused, double *out_5308046335548562329) {
  H_25(state, unused, out_5308046335548562329);
}
void car_h_24(double *state, double *unused, double *out_4376945197385028814) {
  h_24(state, unused, out_4376945197385028814);
}
void car_H_24(double *state, double *unused, double *out_1754685561347889600) {
  H_24(state, unused, out_1754685561347889600);
}
void car_h_30(double *state, double *unused, double *out_7231620904892661289) {
  h_30(state, unused, out_7231620904892661289);
}
void car_H_30(double *state, double *unused, double *out_7826379294055810956) {
  H_30(state, unused, out_7826379294055810956);
}
void car_h_26(double *state, double *unused, double *out_1746536873551828834) {
  h_26(state, unused, out_1746536873551828834);
}
void car_H_26(double *state, double *unused, double *out_1566543016674506105) {
  H_26(state, unused, out_1566543016674506105);
}
void car_h_27(double *state, double *unused, double *out_8335523785723686787) {
  h_27(state, unused, out_8335523785723686787);
}
void car_H_27(double *state, double *unused, double *out_5651615982255386045) {
  H_27(state, unused, out_5651615982255386045);
}
void car_h_29(double *state, double *unused, double *out_8178337117372557642) {
  h_29(state, unused, out_8178337117372557642);
}
void car_H_29(double *state, double *unused, double *out_8336610638370203140) {
  H_29(state, unused, out_8336610638370203140);
}
void car_h_28(double *state, double *unused, double *out_3597377736216872857) {
  h_28(state, unused, out_3597377736216872857);
}
void car_H_28(double *state, double *unused, double *out_3254211621300672566) {
  H_28(state, unused, out_3254211621300672566);
}
void car_h_31(double *state, double *unused, double *out_303599010191181208) {
  h_31(state, unused, out_303599010191181208);
}
void car_H_31(double *state, double *unused, double *out_940334914441154629) {
  H_31(state, unused, out_940334914441154629);
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
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
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
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
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

ekf_lib_init(car)

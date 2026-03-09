#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3753227418625007558);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6246023284876268363);
void car_H_mod_fun(double *state, double *out_5326220687443925213);
void car_f_fun(double *state, double dt, double *out_8658820599897161958);
void car_F_fun(double *state, double dt, double *out_3017086349094288292);
void car_h_25(double *state, double *unused, double *out_7506814967177167178);
void car_H_25(double *state, double *unused, double *out_5308046335548562329);
void car_h_24(double *state, double *unused, double *out_4376945197385028814);
void car_H_24(double *state, double *unused, double *out_1754685561347889600);
void car_h_30(double *state, double *unused, double *out_7231620904892661289);
void car_H_30(double *state, double *unused, double *out_7826379294055810956);
void car_h_26(double *state, double *unused, double *out_1746536873551828834);
void car_H_26(double *state, double *unused, double *out_1566543016674506105);
void car_h_27(double *state, double *unused, double *out_8335523785723686787);
void car_H_27(double *state, double *unused, double *out_5651615982255386045);
void car_h_29(double *state, double *unused, double *out_8178337117372557642);
void car_H_29(double *state, double *unused, double *out_8336610638370203140);
void car_h_28(double *state, double *unused, double *out_3597377736216872857);
void car_H_28(double *state, double *unused, double *out_3254211621300672566);
void car_h_31(double *state, double *unused, double *out_303599010191181208);
void car_H_31(double *state, double *unused, double *out_940334914441154629);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
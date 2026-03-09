#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2103793852715952646);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7486425640275390002);
void pose_H_mod_fun(double *state, double *out_3511508283555031915);
void pose_f_fun(double *state, double dt, double *out_4315192145077905717);
void pose_F_fun(double *state, double dt, double *out_5323785267198531003);
void pose_h_4(double *state, double *unused, double *out_1791351456807006336);
void pose_H_4(double *state, double *unused, double *out_5432853382502789427);
void pose_h_10(double *state, double *unused, double *out_505052022544704925);
void pose_H_10(double *state, double *unused, double *out_5103853662812526892);
void pose_h_13(double *state, double *unused, double *out_8955651205854686458);
void pose_H_13(double *state, double *unused, double *out_2220579557170456626);
void pose_h_14(double *state, double *unused, double *out_1905155691463244381);
void pose_H_14(double *state, double *unused, double *out_1469612526163304898);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
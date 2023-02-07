#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_3006024959811024317);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6749470672304064626);
void car_H_mod_fun(double *state, double *out_1876413921371897227);
void car_f_fun(double *state, double dt, double *out_7267003510127361778);
void car_F_fun(double *state, double dt, double *out_4507925919255265520);
void car_h_25(double *state, double *unused, double *out_1777083439914440830);
void car_H_25(double *state, double *unused, double *out_5796497272403601369);
void car_h_24(double *state, double *unused, double *out_1016139534714745614);
void car_H_24(double *state, double *unused, double *out_2660528953614973373);
void car_h_30(double *state, double *unused, double *out_326452047285557663);
void car_H_30(double *state, double *unused, double *out_8215759021529950039);
void car_h_26(double *state, double *unused, double *out_4739965471432785060);
void car_H_26(double *state, double *unused, double *out_56703755723585689);
void car_h_27(double *state, double *unused, double *out_6778756707575286475);
void car_H_27(double *state, double *unused, double *out_5104983626382207223);
void car_h_29(double *state, double *unused, double *out_5328125314946403308);
void car_H_29(double *state, double *unused, double *out_7641200407674413864);
void car_h_28(double *state, double *unused, double *out_5225526501588038331);
void car_H_28(double *state, double *unused, double *out_4370025258873715249);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}
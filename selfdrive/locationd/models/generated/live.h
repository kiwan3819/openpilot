#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_3982672328206977086);
void live_err_fun(double *nom_x, double *delta_x, double *out_7065534367796289506);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_479709996386351187);
void live_H_mod_fun(double *state, double *out_4572680402772579634);
void live_f_fun(double *state, double dt, double *out_7823197626812359551);
void live_F_fun(double *state, double dt, double *out_350263754613447254);
void live_h_4(double *state, double *unused, double *out_1228675592078714749);
void live_H_4(double *state, double *unused, double *out_6543490681844391960);
void live_h_9(double *state, double *unused, double *out_5078662012341720751);
void live_H_9(double *state, double *unused, double *out_743728253420055510);
void live_h_10(double *state, double *unused, double *out_854738547976412628);
void live_H_10(double *state, double *unused, double *out_2209412934872969411);
void live_h_12(double *state, double *unused, double *out_4011620689457812030);
void live_H_12(double *state, double *unused, double *out_5521995014822426660);
void live_h_31(double *state, double *unused, double *out_5651320870718150641);
void live_H_31(double *state, double *unused, double *out_1221528758512583544);
void live_h_32(double *state, double *unused, double *out_3573880032566032592);
void live_H_32(double *state, double *unused, double *out_3045663839531044115);
void live_h_13(double *state, double *unused, double *out_1970098613952177986);
void live_H_13(double *state, double *unused, double *out_1393358276815853877);
void live_h_14(double *state, double *unused, double *out_5078662012341720751);
void live_H_14(double *state, double *unused, double *out_743728253420055510);
void live_h_33(double *state, double *unused, double *out_1290343346391299525);
void live_H_33(double *state, double *unused, double *out_4372085763151441148);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}
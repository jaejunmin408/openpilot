#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_2103793852715952646) {
   out_2103793852715952646[0] = delta_x[0] + nom_x[0];
   out_2103793852715952646[1] = delta_x[1] + nom_x[1];
   out_2103793852715952646[2] = delta_x[2] + nom_x[2];
   out_2103793852715952646[3] = delta_x[3] + nom_x[3];
   out_2103793852715952646[4] = delta_x[4] + nom_x[4];
   out_2103793852715952646[5] = delta_x[5] + nom_x[5];
   out_2103793852715952646[6] = delta_x[6] + nom_x[6];
   out_2103793852715952646[7] = delta_x[7] + nom_x[7];
   out_2103793852715952646[8] = delta_x[8] + nom_x[8];
   out_2103793852715952646[9] = delta_x[9] + nom_x[9];
   out_2103793852715952646[10] = delta_x[10] + nom_x[10];
   out_2103793852715952646[11] = delta_x[11] + nom_x[11];
   out_2103793852715952646[12] = delta_x[12] + nom_x[12];
   out_2103793852715952646[13] = delta_x[13] + nom_x[13];
   out_2103793852715952646[14] = delta_x[14] + nom_x[14];
   out_2103793852715952646[15] = delta_x[15] + nom_x[15];
   out_2103793852715952646[16] = delta_x[16] + nom_x[16];
   out_2103793852715952646[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7486425640275390002) {
   out_7486425640275390002[0] = -nom_x[0] + true_x[0];
   out_7486425640275390002[1] = -nom_x[1] + true_x[1];
   out_7486425640275390002[2] = -nom_x[2] + true_x[2];
   out_7486425640275390002[3] = -nom_x[3] + true_x[3];
   out_7486425640275390002[4] = -nom_x[4] + true_x[4];
   out_7486425640275390002[5] = -nom_x[5] + true_x[5];
   out_7486425640275390002[6] = -nom_x[6] + true_x[6];
   out_7486425640275390002[7] = -nom_x[7] + true_x[7];
   out_7486425640275390002[8] = -nom_x[8] + true_x[8];
   out_7486425640275390002[9] = -nom_x[9] + true_x[9];
   out_7486425640275390002[10] = -nom_x[10] + true_x[10];
   out_7486425640275390002[11] = -nom_x[11] + true_x[11];
   out_7486425640275390002[12] = -nom_x[12] + true_x[12];
   out_7486425640275390002[13] = -nom_x[13] + true_x[13];
   out_7486425640275390002[14] = -nom_x[14] + true_x[14];
   out_7486425640275390002[15] = -nom_x[15] + true_x[15];
   out_7486425640275390002[16] = -nom_x[16] + true_x[16];
   out_7486425640275390002[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_3511508283555031915) {
   out_3511508283555031915[0] = 1.0;
   out_3511508283555031915[1] = 0.0;
   out_3511508283555031915[2] = 0.0;
   out_3511508283555031915[3] = 0.0;
   out_3511508283555031915[4] = 0.0;
   out_3511508283555031915[5] = 0.0;
   out_3511508283555031915[6] = 0.0;
   out_3511508283555031915[7] = 0.0;
   out_3511508283555031915[8] = 0.0;
   out_3511508283555031915[9] = 0.0;
   out_3511508283555031915[10] = 0.0;
   out_3511508283555031915[11] = 0.0;
   out_3511508283555031915[12] = 0.0;
   out_3511508283555031915[13] = 0.0;
   out_3511508283555031915[14] = 0.0;
   out_3511508283555031915[15] = 0.0;
   out_3511508283555031915[16] = 0.0;
   out_3511508283555031915[17] = 0.0;
   out_3511508283555031915[18] = 0.0;
   out_3511508283555031915[19] = 1.0;
   out_3511508283555031915[20] = 0.0;
   out_3511508283555031915[21] = 0.0;
   out_3511508283555031915[22] = 0.0;
   out_3511508283555031915[23] = 0.0;
   out_3511508283555031915[24] = 0.0;
   out_3511508283555031915[25] = 0.0;
   out_3511508283555031915[26] = 0.0;
   out_3511508283555031915[27] = 0.0;
   out_3511508283555031915[28] = 0.0;
   out_3511508283555031915[29] = 0.0;
   out_3511508283555031915[30] = 0.0;
   out_3511508283555031915[31] = 0.0;
   out_3511508283555031915[32] = 0.0;
   out_3511508283555031915[33] = 0.0;
   out_3511508283555031915[34] = 0.0;
   out_3511508283555031915[35] = 0.0;
   out_3511508283555031915[36] = 0.0;
   out_3511508283555031915[37] = 0.0;
   out_3511508283555031915[38] = 1.0;
   out_3511508283555031915[39] = 0.0;
   out_3511508283555031915[40] = 0.0;
   out_3511508283555031915[41] = 0.0;
   out_3511508283555031915[42] = 0.0;
   out_3511508283555031915[43] = 0.0;
   out_3511508283555031915[44] = 0.0;
   out_3511508283555031915[45] = 0.0;
   out_3511508283555031915[46] = 0.0;
   out_3511508283555031915[47] = 0.0;
   out_3511508283555031915[48] = 0.0;
   out_3511508283555031915[49] = 0.0;
   out_3511508283555031915[50] = 0.0;
   out_3511508283555031915[51] = 0.0;
   out_3511508283555031915[52] = 0.0;
   out_3511508283555031915[53] = 0.0;
   out_3511508283555031915[54] = 0.0;
   out_3511508283555031915[55] = 0.0;
   out_3511508283555031915[56] = 0.0;
   out_3511508283555031915[57] = 1.0;
   out_3511508283555031915[58] = 0.0;
   out_3511508283555031915[59] = 0.0;
   out_3511508283555031915[60] = 0.0;
   out_3511508283555031915[61] = 0.0;
   out_3511508283555031915[62] = 0.0;
   out_3511508283555031915[63] = 0.0;
   out_3511508283555031915[64] = 0.0;
   out_3511508283555031915[65] = 0.0;
   out_3511508283555031915[66] = 0.0;
   out_3511508283555031915[67] = 0.0;
   out_3511508283555031915[68] = 0.0;
   out_3511508283555031915[69] = 0.0;
   out_3511508283555031915[70] = 0.0;
   out_3511508283555031915[71] = 0.0;
   out_3511508283555031915[72] = 0.0;
   out_3511508283555031915[73] = 0.0;
   out_3511508283555031915[74] = 0.0;
   out_3511508283555031915[75] = 0.0;
   out_3511508283555031915[76] = 1.0;
   out_3511508283555031915[77] = 0.0;
   out_3511508283555031915[78] = 0.0;
   out_3511508283555031915[79] = 0.0;
   out_3511508283555031915[80] = 0.0;
   out_3511508283555031915[81] = 0.0;
   out_3511508283555031915[82] = 0.0;
   out_3511508283555031915[83] = 0.0;
   out_3511508283555031915[84] = 0.0;
   out_3511508283555031915[85] = 0.0;
   out_3511508283555031915[86] = 0.0;
   out_3511508283555031915[87] = 0.0;
   out_3511508283555031915[88] = 0.0;
   out_3511508283555031915[89] = 0.0;
   out_3511508283555031915[90] = 0.0;
   out_3511508283555031915[91] = 0.0;
   out_3511508283555031915[92] = 0.0;
   out_3511508283555031915[93] = 0.0;
   out_3511508283555031915[94] = 0.0;
   out_3511508283555031915[95] = 1.0;
   out_3511508283555031915[96] = 0.0;
   out_3511508283555031915[97] = 0.0;
   out_3511508283555031915[98] = 0.0;
   out_3511508283555031915[99] = 0.0;
   out_3511508283555031915[100] = 0.0;
   out_3511508283555031915[101] = 0.0;
   out_3511508283555031915[102] = 0.0;
   out_3511508283555031915[103] = 0.0;
   out_3511508283555031915[104] = 0.0;
   out_3511508283555031915[105] = 0.0;
   out_3511508283555031915[106] = 0.0;
   out_3511508283555031915[107] = 0.0;
   out_3511508283555031915[108] = 0.0;
   out_3511508283555031915[109] = 0.0;
   out_3511508283555031915[110] = 0.0;
   out_3511508283555031915[111] = 0.0;
   out_3511508283555031915[112] = 0.0;
   out_3511508283555031915[113] = 0.0;
   out_3511508283555031915[114] = 1.0;
   out_3511508283555031915[115] = 0.0;
   out_3511508283555031915[116] = 0.0;
   out_3511508283555031915[117] = 0.0;
   out_3511508283555031915[118] = 0.0;
   out_3511508283555031915[119] = 0.0;
   out_3511508283555031915[120] = 0.0;
   out_3511508283555031915[121] = 0.0;
   out_3511508283555031915[122] = 0.0;
   out_3511508283555031915[123] = 0.0;
   out_3511508283555031915[124] = 0.0;
   out_3511508283555031915[125] = 0.0;
   out_3511508283555031915[126] = 0.0;
   out_3511508283555031915[127] = 0.0;
   out_3511508283555031915[128] = 0.0;
   out_3511508283555031915[129] = 0.0;
   out_3511508283555031915[130] = 0.0;
   out_3511508283555031915[131] = 0.0;
   out_3511508283555031915[132] = 0.0;
   out_3511508283555031915[133] = 1.0;
   out_3511508283555031915[134] = 0.0;
   out_3511508283555031915[135] = 0.0;
   out_3511508283555031915[136] = 0.0;
   out_3511508283555031915[137] = 0.0;
   out_3511508283555031915[138] = 0.0;
   out_3511508283555031915[139] = 0.0;
   out_3511508283555031915[140] = 0.0;
   out_3511508283555031915[141] = 0.0;
   out_3511508283555031915[142] = 0.0;
   out_3511508283555031915[143] = 0.0;
   out_3511508283555031915[144] = 0.0;
   out_3511508283555031915[145] = 0.0;
   out_3511508283555031915[146] = 0.0;
   out_3511508283555031915[147] = 0.0;
   out_3511508283555031915[148] = 0.0;
   out_3511508283555031915[149] = 0.0;
   out_3511508283555031915[150] = 0.0;
   out_3511508283555031915[151] = 0.0;
   out_3511508283555031915[152] = 1.0;
   out_3511508283555031915[153] = 0.0;
   out_3511508283555031915[154] = 0.0;
   out_3511508283555031915[155] = 0.0;
   out_3511508283555031915[156] = 0.0;
   out_3511508283555031915[157] = 0.0;
   out_3511508283555031915[158] = 0.0;
   out_3511508283555031915[159] = 0.0;
   out_3511508283555031915[160] = 0.0;
   out_3511508283555031915[161] = 0.0;
   out_3511508283555031915[162] = 0.0;
   out_3511508283555031915[163] = 0.0;
   out_3511508283555031915[164] = 0.0;
   out_3511508283555031915[165] = 0.0;
   out_3511508283555031915[166] = 0.0;
   out_3511508283555031915[167] = 0.0;
   out_3511508283555031915[168] = 0.0;
   out_3511508283555031915[169] = 0.0;
   out_3511508283555031915[170] = 0.0;
   out_3511508283555031915[171] = 1.0;
   out_3511508283555031915[172] = 0.0;
   out_3511508283555031915[173] = 0.0;
   out_3511508283555031915[174] = 0.0;
   out_3511508283555031915[175] = 0.0;
   out_3511508283555031915[176] = 0.0;
   out_3511508283555031915[177] = 0.0;
   out_3511508283555031915[178] = 0.0;
   out_3511508283555031915[179] = 0.0;
   out_3511508283555031915[180] = 0.0;
   out_3511508283555031915[181] = 0.0;
   out_3511508283555031915[182] = 0.0;
   out_3511508283555031915[183] = 0.0;
   out_3511508283555031915[184] = 0.0;
   out_3511508283555031915[185] = 0.0;
   out_3511508283555031915[186] = 0.0;
   out_3511508283555031915[187] = 0.0;
   out_3511508283555031915[188] = 0.0;
   out_3511508283555031915[189] = 0.0;
   out_3511508283555031915[190] = 1.0;
   out_3511508283555031915[191] = 0.0;
   out_3511508283555031915[192] = 0.0;
   out_3511508283555031915[193] = 0.0;
   out_3511508283555031915[194] = 0.0;
   out_3511508283555031915[195] = 0.0;
   out_3511508283555031915[196] = 0.0;
   out_3511508283555031915[197] = 0.0;
   out_3511508283555031915[198] = 0.0;
   out_3511508283555031915[199] = 0.0;
   out_3511508283555031915[200] = 0.0;
   out_3511508283555031915[201] = 0.0;
   out_3511508283555031915[202] = 0.0;
   out_3511508283555031915[203] = 0.0;
   out_3511508283555031915[204] = 0.0;
   out_3511508283555031915[205] = 0.0;
   out_3511508283555031915[206] = 0.0;
   out_3511508283555031915[207] = 0.0;
   out_3511508283555031915[208] = 0.0;
   out_3511508283555031915[209] = 1.0;
   out_3511508283555031915[210] = 0.0;
   out_3511508283555031915[211] = 0.0;
   out_3511508283555031915[212] = 0.0;
   out_3511508283555031915[213] = 0.0;
   out_3511508283555031915[214] = 0.0;
   out_3511508283555031915[215] = 0.0;
   out_3511508283555031915[216] = 0.0;
   out_3511508283555031915[217] = 0.0;
   out_3511508283555031915[218] = 0.0;
   out_3511508283555031915[219] = 0.0;
   out_3511508283555031915[220] = 0.0;
   out_3511508283555031915[221] = 0.0;
   out_3511508283555031915[222] = 0.0;
   out_3511508283555031915[223] = 0.0;
   out_3511508283555031915[224] = 0.0;
   out_3511508283555031915[225] = 0.0;
   out_3511508283555031915[226] = 0.0;
   out_3511508283555031915[227] = 0.0;
   out_3511508283555031915[228] = 1.0;
   out_3511508283555031915[229] = 0.0;
   out_3511508283555031915[230] = 0.0;
   out_3511508283555031915[231] = 0.0;
   out_3511508283555031915[232] = 0.0;
   out_3511508283555031915[233] = 0.0;
   out_3511508283555031915[234] = 0.0;
   out_3511508283555031915[235] = 0.0;
   out_3511508283555031915[236] = 0.0;
   out_3511508283555031915[237] = 0.0;
   out_3511508283555031915[238] = 0.0;
   out_3511508283555031915[239] = 0.0;
   out_3511508283555031915[240] = 0.0;
   out_3511508283555031915[241] = 0.0;
   out_3511508283555031915[242] = 0.0;
   out_3511508283555031915[243] = 0.0;
   out_3511508283555031915[244] = 0.0;
   out_3511508283555031915[245] = 0.0;
   out_3511508283555031915[246] = 0.0;
   out_3511508283555031915[247] = 1.0;
   out_3511508283555031915[248] = 0.0;
   out_3511508283555031915[249] = 0.0;
   out_3511508283555031915[250] = 0.0;
   out_3511508283555031915[251] = 0.0;
   out_3511508283555031915[252] = 0.0;
   out_3511508283555031915[253] = 0.0;
   out_3511508283555031915[254] = 0.0;
   out_3511508283555031915[255] = 0.0;
   out_3511508283555031915[256] = 0.0;
   out_3511508283555031915[257] = 0.0;
   out_3511508283555031915[258] = 0.0;
   out_3511508283555031915[259] = 0.0;
   out_3511508283555031915[260] = 0.0;
   out_3511508283555031915[261] = 0.0;
   out_3511508283555031915[262] = 0.0;
   out_3511508283555031915[263] = 0.0;
   out_3511508283555031915[264] = 0.0;
   out_3511508283555031915[265] = 0.0;
   out_3511508283555031915[266] = 1.0;
   out_3511508283555031915[267] = 0.0;
   out_3511508283555031915[268] = 0.0;
   out_3511508283555031915[269] = 0.0;
   out_3511508283555031915[270] = 0.0;
   out_3511508283555031915[271] = 0.0;
   out_3511508283555031915[272] = 0.0;
   out_3511508283555031915[273] = 0.0;
   out_3511508283555031915[274] = 0.0;
   out_3511508283555031915[275] = 0.0;
   out_3511508283555031915[276] = 0.0;
   out_3511508283555031915[277] = 0.0;
   out_3511508283555031915[278] = 0.0;
   out_3511508283555031915[279] = 0.0;
   out_3511508283555031915[280] = 0.0;
   out_3511508283555031915[281] = 0.0;
   out_3511508283555031915[282] = 0.0;
   out_3511508283555031915[283] = 0.0;
   out_3511508283555031915[284] = 0.0;
   out_3511508283555031915[285] = 1.0;
   out_3511508283555031915[286] = 0.0;
   out_3511508283555031915[287] = 0.0;
   out_3511508283555031915[288] = 0.0;
   out_3511508283555031915[289] = 0.0;
   out_3511508283555031915[290] = 0.0;
   out_3511508283555031915[291] = 0.0;
   out_3511508283555031915[292] = 0.0;
   out_3511508283555031915[293] = 0.0;
   out_3511508283555031915[294] = 0.0;
   out_3511508283555031915[295] = 0.0;
   out_3511508283555031915[296] = 0.0;
   out_3511508283555031915[297] = 0.0;
   out_3511508283555031915[298] = 0.0;
   out_3511508283555031915[299] = 0.0;
   out_3511508283555031915[300] = 0.0;
   out_3511508283555031915[301] = 0.0;
   out_3511508283555031915[302] = 0.0;
   out_3511508283555031915[303] = 0.0;
   out_3511508283555031915[304] = 1.0;
   out_3511508283555031915[305] = 0.0;
   out_3511508283555031915[306] = 0.0;
   out_3511508283555031915[307] = 0.0;
   out_3511508283555031915[308] = 0.0;
   out_3511508283555031915[309] = 0.0;
   out_3511508283555031915[310] = 0.0;
   out_3511508283555031915[311] = 0.0;
   out_3511508283555031915[312] = 0.0;
   out_3511508283555031915[313] = 0.0;
   out_3511508283555031915[314] = 0.0;
   out_3511508283555031915[315] = 0.0;
   out_3511508283555031915[316] = 0.0;
   out_3511508283555031915[317] = 0.0;
   out_3511508283555031915[318] = 0.0;
   out_3511508283555031915[319] = 0.0;
   out_3511508283555031915[320] = 0.0;
   out_3511508283555031915[321] = 0.0;
   out_3511508283555031915[322] = 0.0;
   out_3511508283555031915[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4315192145077905717) {
   out_4315192145077905717[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4315192145077905717[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4315192145077905717[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4315192145077905717[3] = dt*state[12] + state[3];
   out_4315192145077905717[4] = dt*state[13] + state[4];
   out_4315192145077905717[5] = dt*state[14] + state[5];
   out_4315192145077905717[6] = state[6];
   out_4315192145077905717[7] = state[7];
   out_4315192145077905717[8] = state[8];
   out_4315192145077905717[9] = state[9];
   out_4315192145077905717[10] = state[10];
   out_4315192145077905717[11] = state[11];
   out_4315192145077905717[12] = state[12];
   out_4315192145077905717[13] = state[13];
   out_4315192145077905717[14] = state[14];
   out_4315192145077905717[15] = state[15];
   out_4315192145077905717[16] = state[16];
   out_4315192145077905717[17] = state[17];
}
void F_fun(double *state, double dt, double *out_5323785267198531003) {
   out_5323785267198531003[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5323785267198531003[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5323785267198531003[2] = 0;
   out_5323785267198531003[3] = 0;
   out_5323785267198531003[4] = 0;
   out_5323785267198531003[5] = 0;
   out_5323785267198531003[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5323785267198531003[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5323785267198531003[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_5323785267198531003[9] = 0;
   out_5323785267198531003[10] = 0;
   out_5323785267198531003[11] = 0;
   out_5323785267198531003[12] = 0;
   out_5323785267198531003[13] = 0;
   out_5323785267198531003[14] = 0;
   out_5323785267198531003[15] = 0;
   out_5323785267198531003[16] = 0;
   out_5323785267198531003[17] = 0;
   out_5323785267198531003[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5323785267198531003[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5323785267198531003[20] = 0;
   out_5323785267198531003[21] = 0;
   out_5323785267198531003[22] = 0;
   out_5323785267198531003[23] = 0;
   out_5323785267198531003[24] = 0;
   out_5323785267198531003[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5323785267198531003[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_5323785267198531003[27] = 0;
   out_5323785267198531003[28] = 0;
   out_5323785267198531003[29] = 0;
   out_5323785267198531003[30] = 0;
   out_5323785267198531003[31] = 0;
   out_5323785267198531003[32] = 0;
   out_5323785267198531003[33] = 0;
   out_5323785267198531003[34] = 0;
   out_5323785267198531003[35] = 0;
   out_5323785267198531003[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5323785267198531003[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5323785267198531003[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5323785267198531003[39] = 0;
   out_5323785267198531003[40] = 0;
   out_5323785267198531003[41] = 0;
   out_5323785267198531003[42] = 0;
   out_5323785267198531003[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5323785267198531003[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_5323785267198531003[45] = 0;
   out_5323785267198531003[46] = 0;
   out_5323785267198531003[47] = 0;
   out_5323785267198531003[48] = 0;
   out_5323785267198531003[49] = 0;
   out_5323785267198531003[50] = 0;
   out_5323785267198531003[51] = 0;
   out_5323785267198531003[52] = 0;
   out_5323785267198531003[53] = 0;
   out_5323785267198531003[54] = 0;
   out_5323785267198531003[55] = 0;
   out_5323785267198531003[56] = 0;
   out_5323785267198531003[57] = 1;
   out_5323785267198531003[58] = 0;
   out_5323785267198531003[59] = 0;
   out_5323785267198531003[60] = 0;
   out_5323785267198531003[61] = 0;
   out_5323785267198531003[62] = 0;
   out_5323785267198531003[63] = 0;
   out_5323785267198531003[64] = 0;
   out_5323785267198531003[65] = 0;
   out_5323785267198531003[66] = dt;
   out_5323785267198531003[67] = 0;
   out_5323785267198531003[68] = 0;
   out_5323785267198531003[69] = 0;
   out_5323785267198531003[70] = 0;
   out_5323785267198531003[71] = 0;
   out_5323785267198531003[72] = 0;
   out_5323785267198531003[73] = 0;
   out_5323785267198531003[74] = 0;
   out_5323785267198531003[75] = 0;
   out_5323785267198531003[76] = 1;
   out_5323785267198531003[77] = 0;
   out_5323785267198531003[78] = 0;
   out_5323785267198531003[79] = 0;
   out_5323785267198531003[80] = 0;
   out_5323785267198531003[81] = 0;
   out_5323785267198531003[82] = 0;
   out_5323785267198531003[83] = 0;
   out_5323785267198531003[84] = 0;
   out_5323785267198531003[85] = dt;
   out_5323785267198531003[86] = 0;
   out_5323785267198531003[87] = 0;
   out_5323785267198531003[88] = 0;
   out_5323785267198531003[89] = 0;
   out_5323785267198531003[90] = 0;
   out_5323785267198531003[91] = 0;
   out_5323785267198531003[92] = 0;
   out_5323785267198531003[93] = 0;
   out_5323785267198531003[94] = 0;
   out_5323785267198531003[95] = 1;
   out_5323785267198531003[96] = 0;
   out_5323785267198531003[97] = 0;
   out_5323785267198531003[98] = 0;
   out_5323785267198531003[99] = 0;
   out_5323785267198531003[100] = 0;
   out_5323785267198531003[101] = 0;
   out_5323785267198531003[102] = 0;
   out_5323785267198531003[103] = 0;
   out_5323785267198531003[104] = dt;
   out_5323785267198531003[105] = 0;
   out_5323785267198531003[106] = 0;
   out_5323785267198531003[107] = 0;
   out_5323785267198531003[108] = 0;
   out_5323785267198531003[109] = 0;
   out_5323785267198531003[110] = 0;
   out_5323785267198531003[111] = 0;
   out_5323785267198531003[112] = 0;
   out_5323785267198531003[113] = 0;
   out_5323785267198531003[114] = 1;
   out_5323785267198531003[115] = 0;
   out_5323785267198531003[116] = 0;
   out_5323785267198531003[117] = 0;
   out_5323785267198531003[118] = 0;
   out_5323785267198531003[119] = 0;
   out_5323785267198531003[120] = 0;
   out_5323785267198531003[121] = 0;
   out_5323785267198531003[122] = 0;
   out_5323785267198531003[123] = 0;
   out_5323785267198531003[124] = 0;
   out_5323785267198531003[125] = 0;
   out_5323785267198531003[126] = 0;
   out_5323785267198531003[127] = 0;
   out_5323785267198531003[128] = 0;
   out_5323785267198531003[129] = 0;
   out_5323785267198531003[130] = 0;
   out_5323785267198531003[131] = 0;
   out_5323785267198531003[132] = 0;
   out_5323785267198531003[133] = 1;
   out_5323785267198531003[134] = 0;
   out_5323785267198531003[135] = 0;
   out_5323785267198531003[136] = 0;
   out_5323785267198531003[137] = 0;
   out_5323785267198531003[138] = 0;
   out_5323785267198531003[139] = 0;
   out_5323785267198531003[140] = 0;
   out_5323785267198531003[141] = 0;
   out_5323785267198531003[142] = 0;
   out_5323785267198531003[143] = 0;
   out_5323785267198531003[144] = 0;
   out_5323785267198531003[145] = 0;
   out_5323785267198531003[146] = 0;
   out_5323785267198531003[147] = 0;
   out_5323785267198531003[148] = 0;
   out_5323785267198531003[149] = 0;
   out_5323785267198531003[150] = 0;
   out_5323785267198531003[151] = 0;
   out_5323785267198531003[152] = 1;
   out_5323785267198531003[153] = 0;
   out_5323785267198531003[154] = 0;
   out_5323785267198531003[155] = 0;
   out_5323785267198531003[156] = 0;
   out_5323785267198531003[157] = 0;
   out_5323785267198531003[158] = 0;
   out_5323785267198531003[159] = 0;
   out_5323785267198531003[160] = 0;
   out_5323785267198531003[161] = 0;
   out_5323785267198531003[162] = 0;
   out_5323785267198531003[163] = 0;
   out_5323785267198531003[164] = 0;
   out_5323785267198531003[165] = 0;
   out_5323785267198531003[166] = 0;
   out_5323785267198531003[167] = 0;
   out_5323785267198531003[168] = 0;
   out_5323785267198531003[169] = 0;
   out_5323785267198531003[170] = 0;
   out_5323785267198531003[171] = 1;
   out_5323785267198531003[172] = 0;
   out_5323785267198531003[173] = 0;
   out_5323785267198531003[174] = 0;
   out_5323785267198531003[175] = 0;
   out_5323785267198531003[176] = 0;
   out_5323785267198531003[177] = 0;
   out_5323785267198531003[178] = 0;
   out_5323785267198531003[179] = 0;
   out_5323785267198531003[180] = 0;
   out_5323785267198531003[181] = 0;
   out_5323785267198531003[182] = 0;
   out_5323785267198531003[183] = 0;
   out_5323785267198531003[184] = 0;
   out_5323785267198531003[185] = 0;
   out_5323785267198531003[186] = 0;
   out_5323785267198531003[187] = 0;
   out_5323785267198531003[188] = 0;
   out_5323785267198531003[189] = 0;
   out_5323785267198531003[190] = 1;
   out_5323785267198531003[191] = 0;
   out_5323785267198531003[192] = 0;
   out_5323785267198531003[193] = 0;
   out_5323785267198531003[194] = 0;
   out_5323785267198531003[195] = 0;
   out_5323785267198531003[196] = 0;
   out_5323785267198531003[197] = 0;
   out_5323785267198531003[198] = 0;
   out_5323785267198531003[199] = 0;
   out_5323785267198531003[200] = 0;
   out_5323785267198531003[201] = 0;
   out_5323785267198531003[202] = 0;
   out_5323785267198531003[203] = 0;
   out_5323785267198531003[204] = 0;
   out_5323785267198531003[205] = 0;
   out_5323785267198531003[206] = 0;
   out_5323785267198531003[207] = 0;
   out_5323785267198531003[208] = 0;
   out_5323785267198531003[209] = 1;
   out_5323785267198531003[210] = 0;
   out_5323785267198531003[211] = 0;
   out_5323785267198531003[212] = 0;
   out_5323785267198531003[213] = 0;
   out_5323785267198531003[214] = 0;
   out_5323785267198531003[215] = 0;
   out_5323785267198531003[216] = 0;
   out_5323785267198531003[217] = 0;
   out_5323785267198531003[218] = 0;
   out_5323785267198531003[219] = 0;
   out_5323785267198531003[220] = 0;
   out_5323785267198531003[221] = 0;
   out_5323785267198531003[222] = 0;
   out_5323785267198531003[223] = 0;
   out_5323785267198531003[224] = 0;
   out_5323785267198531003[225] = 0;
   out_5323785267198531003[226] = 0;
   out_5323785267198531003[227] = 0;
   out_5323785267198531003[228] = 1;
   out_5323785267198531003[229] = 0;
   out_5323785267198531003[230] = 0;
   out_5323785267198531003[231] = 0;
   out_5323785267198531003[232] = 0;
   out_5323785267198531003[233] = 0;
   out_5323785267198531003[234] = 0;
   out_5323785267198531003[235] = 0;
   out_5323785267198531003[236] = 0;
   out_5323785267198531003[237] = 0;
   out_5323785267198531003[238] = 0;
   out_5323785267198531003[239] = 0;
   out_5323785267198531003[240] = 0;
   out_5323785267198531003[241] = 0;
   out_5323785267198531003[242] = 0;
   out_5323785267198531003[243] = 0;
   out_5323785267198531003[244] = 0;
   out_5323785267198531003[245] = 0;
   out_5323785267198531003[246] = 0;
   out_5323785267198531003[247] = 1;
   out_5323785267198531003[248] = 0;
   out_5323785267198531003[249] = 0;
   out_5323785267198531003[250] = 0;
   out_5323785267198531003[251] = 0;
   out_5323785267198531003[252] = 0;
   out_5323785267198531003[253] = 0;
   out_5323785267198531003[254] = 0;
   out_5323785267198531003[255] = 0;
   out_5323785267198531003[256] = 0;
   out_5323785267198531003[257] = 0;
   out_5323785267198531003[258] = 0;
   out_5323785267198531003[259] = 0;
   out_5323785267198531003[260] = 0;
   out_5323785267198531003[261] = 0;
   out_5323785267198531003[262] = 0;
   out_5323785267198531003[263] = 0;
   out_5323785267198531003[264] = 0;
   out_5323785267198531003[265] = 0;
   out_5323785267198531003[266] = 1;
   out_5323785267198531003[267] = 0;
   out_5323785267198531003[268] = 0;
   out_5323785267198531003[269] = 0;
   out_5323785267198531003[270] = 0;
   out_5323785267198531003[271] = 0;
   out_5323785267198531003[272] = 0;
   out_5323785267198531003[273] = 0;
   out_5323785267198531003[274] = 0;
   out_5323785267198531003[275] = 0;
   out_5323785267198531003[276] = 0;
   out_5323785267198531003[277] = 0;
   out_5323785267198531003[278] = 0;
   out_5323785267198531003[279] = 0;
   out_5323785267198531003[280] = 0;
   out_5323785267198531003[281] = 0;
   out_5323785267198531003[282] = 0;
   out_5323785267198531003[283] = 0;
   out_5323785267198531003[284] = 0;
   out_5323785267198531003[285] = 1;
   out_5323785267198531003[286] = 0;
   out_5323785267198531003[287] = 0;
   out_5323785267198531003[288] = 0;
   out_5323785267198531003[289] = 0;
   out_5323785267198531003[290] = 0;
   out_5323785267198531003[291] = 0;
   out_5323785267198531003[292] = 0;
   out_5323785267198531003[293] = 0;
   out_5323785267198531003[294] = 0;
   out_5323785267198531003[295] = 0;
   out_5323785267198531003[296] = 0;
   out_5323785267198531003[297] = 0;
   out_5323785267198531003[298] = 0;
   out_5323785267198531003[299] = 0;
   out_5323785267198531003[300] = 0;
   out_5323785267198531003[301] = 0;
   out_5323785267198531003[302] = 0;
   out_5323785267198531003[303] = 0;
   out_5323785267198531003[304] = 1;
   out_5323785267198531003[305] = 0;
   out_5323785267198531003[306] = 0;
   out_5323785267198531003[307] = 0;
   out_5323785267198531003[308] = 0;
   out_5323785267198531003[309] = 0;
   out_5323785267198531003[310] = 0;
   out_5323785267198531003[311] = 0;
   out_5323785267198531003[312] = 0;
   out_5323785267198531003[313] = 0;
   out_5323785267198531003[314] = 0;
   out_5323785267198531003[315] = 0;
   out_5323785267198531003[316] = 0;
   out_5323785267198531003[317] = 0;
   out_5323785267198531003[318] = 0;
   out_5323785267198531003[319] = 0;
   out_5323785267198531003[320] = 0;
   out_5323785267198531003[321] = 0;
   out_5323785267198531003[322] = 0;
   out_5323785267198531003[323] = 1;
}
void h_4(double *state, double *unused, double *out_1791351456807006336) {
   out_1791351456807006336[0] = state[6] + state[9];
   out_1791351456807006336[1] = state[7] + state[10];
   out_1791351456807006336[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_5432853382502789427) {
   out_5432853382502789427[0] = 0;
   out_5432853382502789427[1] = 0;
   out_5432853382502789427[2] = 0;
   out_5432853382502789427[3] = 0;
   out_5432853382502789427[4] = 0;
   out_5432853382502789427[5] = 0;
   out_5432853382502789427[6] = 1;
   out_5432853382502789427[7] = 0;
   out_5432853382502789427[8] = 0;
   out_5432853382502789427[9] = 1;
   out_5432853382502789427[10] = 0;
   out_5432853382502789427[11] = 0;
   out_5432853382502789427[12] = 0;
   out_5432853382502789427[13] = 0;
   out_5432853382502789427[14] = 0;
   out_5432853382502789427[15] = 0;
   out_5432853382502789427[16] = 0;
   out_5432853382502789427[17] = 0;
   out_5432853382502789427[18] = 0;
   out_5432853382502789427[19] = 0;
   out_5432853382502789427[20] = 0;
   out_5432853382502789427[21] = 0;
   out_5432853382502789427[22] = 0;
   out_5432853382502789427[23] = 0;
   out_5432853382502789427[24] = 0;
   out_5432853382502789427[25] = 1;
   out_5432853382502789427[26] = 0;
   out_5432853382502789427[27] = 0;
   out_5432853382502789427[28] = 1;
   out_5432853382502789427[29] = 0;
   out_5432853382502789427[30] = 0;
   out_5432853382502789427[31] = 0;
   out_5432853382502789427[32] = 0;
   out_5432853382502789427[33] = 0;
   out_5432853382502789427[34] = 0;
   out_5432853382502789427[35] = 0;
   out_5432853382502789427[36] = 0;
   out_5432853382502789427[37] = 0;
   out_5432853382502789427[38] = 0;
   out_5432853382502789427[39] = 0;
   out_5432853382502789427[40] = 0;
   out_5432853382502789427[41] = 0;
   out_5432853382502789427[42] = 0;
   out_5432853382502789427[43] = 0;
   out_5432853382502789427[44] = 1;
   out_5432853382502789427[45] = 0;
   out_5432853382502789427[46] = 0;
   out_5432853382502789427[47] = 1;
   out_5432853382502789427[48] = 0;
   out_5432853382502789427[49] = 0;
   out_5432853382502789427[50] = 0;
   out_5432853382502789427[51] = 0;
   out_5432853382502789427[52] = 0;
   out_5432853382502789427[53] = 0;
}
void h_10(double *state, double *unused, double *out_505052022544704925) {
   out_505052022544704925[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_505052022544704925[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_505052022544704925[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5103853662812526892) {
   out_5103853662812526892[0] = 0;
   out_5103853662812526892[1] = 9.8100000000000005*cos(state[1]);
   out_5103853662812526892[2] = 0;
   out_5103853662812526892[3] = 0;
   out_5103853662812526892[4] = -state[8];
   out_5103853662812526892[5] = state[7];
   out_5103853662812526892[6] = 0;
   out_5103853662812526892[7] = state[5];
   out_5103853662812526892[8] = -state[4];
   out_5103853662812526892[9] = 0;
   out_5103853662812526892[10] = 0;
   out_5103853662812526892[11] = 0;
   out_5103853662812526892[12] = 1;
   out_5103853662812526892[13] = 0;
   out_5103853662812526892[14] = 0;
   out_5103853662812526892[15] = 1;
   out_5103853662812526892[16] = 0;
   out_5103853662812526892[17] = 0;
   out_5103853662812526892[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5103853662812526892[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5103853662812526892[20] = 0;
   out_5103853662812526892[21] = state[8];
   out_5103853662812526892[22] = 0;
   out_5103853662812526892[23] = -state[6];
   out_5103853662812526892[24] = -state[5];
   out_5103853662812526892[25] = 0;
   out_5103853662812526892[26] = state[3];
   out_5103853662812526892[27] = 0;
   out_5103853662812526892[28] = 0;
   out_5103853662812526892[29] = 0;
   out_5103853662812526892[30] = 0;
   out_5103853662812526892[31] = 1;
   out_5103853662812526892[32] = 0;
   out_5103853662812526892[33] = 0;
   out_5103853662812526892[34] = 1;
   out_5103853662812526892[35] = 0;
   out_5103853662812526892[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5103853662812526892[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5103853662812526892[38] = 0;
   out_5103853662812526892[39] = -state[7];
   out_5103853662812526892[40] = state[6];
   out_5103853662812526892[41] = 0;
   out_5103853662812526892[42] = state[4];
   out_5103853662812526892[43] = -state[3];
   out_5103853662812526892[44] = 0;
   out_5103853662812526892[45] = 0;
   out_5103853662812526892[46] = 0;
   out_5103853662812526892[47] = 0;
   out_5103853662812526892[48] = 0;
   out_5103853662812526892[49] = 0;
   out_5103853662812526892[50] = 1;
   out_5103853662812526892[51] = 0;
   out_5103853662812526892[52] = 0;
   out_5103853662812526892[53] = 1;
}
void h_13(double *state, double *unused, double *out_8955651205854686458) {
   out_8955651205854686458[0] = state[3];
   out_8955651205854686458[1] = state[4];
   out_8955651205854686458[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2220579557170456626) {
   out_2220579557170456626[0] = 0;
   out_2220579557170456626[1] = 0;
   out_2220579557170456626[2] = 0;
   out_2220579557170456626[3] = 1;
   out_2220579557170456626[4] = 0;
   out_2220579557170456626[5] = 0;
   out_2220579557170456626[6] = 0;
   out_2220579557170456626[7] = 0;
   out_2220579557170456626[8] = 0;
   out_2220579557170456626[9] = 0;
   out_2220579557170456626[10] = 0;
   out_2220579557170456626[11] = 0;
   out_2220579557170456626[12] = 0;
   out_2220579557170456626[13] = 0;
   out_2220579557170456626[14] = 0;
   out_2220579557170456626[15] = 0;
   out_2220579557170456626[16] = 0;
   out_2220579557170456626[17] = 0;
   out_2220579557170456626[18] = 0;
   out_2220579557170456626[19] = 0;
   out_2220579557170456626[20] = 0;
   out_2220579557170456626[21] = 0;
   out_2220579557170456626[22] = 1;
   out_2220579557170456626[23] = 0;
   out_2220579557170456626[24] = 0;
   out_2220579557170456626[25] = 0;
   out_2220579557170456626[26] = 0;
   out_2220579557170456626[27] = 0;
   out_2220579557170456626[28] = 0;
   out_2220579557170456626[29] = 0;
   out_2220579557170456626[30] = 0;
   out_2220579557170456626[31] = 0;
   out_2220579557170456626[32] = 0;
   out_2220579557170456626[33] = 0;
   out_2220579557170456626[34] = 0;
   out_2220579557170456626[35] = 0;
   out_2220579557170456626[36] = 0;
   out_2220579557170456626[37] = 0;
   out_2220579557170456626[38] = 0;
   out_2220579557170456626[39] = 0;
   out_2220579557170456626[40] = 0;
   out_2220579557170456626[41] = 1;
   out_2220579557170456626[42] = 0;
   out_2220579557170456626[43] = 0;
   out_2220579557170456626[44] = 0;
   out_2220579557170456626[45] = 0;
   out_2220579557170456626[46] = 0;
   out_2220579557170456626[47] = 0;
   out_2220579557170456626[48] = 0;
   out_2220579557170456626[49] = 0;
   out_2220579557170456626[50] = 0;
   out_2220579557170456626[51] = 0;
   out_2220579557170456626[52] = 0;
   out_2220579557170456626[53] = 0;
}
void h_14(double *state, double *unused, double *out_1905155691463244381) {
   out_1905155691463244381[0] = state[6];
   out_1905155691463244381[1] = state[7];
   out_1905155691463244381[2] = state[8];
}
void H_14(double *state, double *unused, double *out_1469612526163304898) {
   out_1469612526163304898[0] = 0;
   out_1469612526163304898[1] = 0;
   out_1469612526163304898[2] = 0;
   out_1469612526163304898[3] = 0;
   out_1469612526163304898[4] = 0;
   out_1469612526163304898[5] = 0;
   out_1469612526163304898[6] = 1;
   out_1469612526163304898[7] = 0;
   out_1469612526163304898[8] = 0;
   out_1469612526163304898[9] = 0;
   out_1469612526163304898[10] = 0;
   out_1469612526163304898[11] = 0;
   out_1469612526163304898[12] = 0;
   out_1469612526163304898[13] = 0;
   out_1469612526163304898[14] = 0;
   out_1469612526163304898[15] = 0;
   out_1469612526163304898[16] = 0;
   out_1469612526163304898[17] = 0;
   out_1469612526163304898[18] = 0;
   out_1469612526163304898[19] = 0;
   out_1469612526163304898[20] = 0;
   out_1469612526163304898[21] = 0;
   out_1469612526163304898[22] = 0;
   out_1469612526163304898[23] = 0;
   out_1469612526163304898[24] = 0;
   out_1469612526163304898[25] = 1;
   out_1469612526163304898[26] = 0;
   out_1469612526163304898[27] = 0;
   out_1469612526163304898[28] = 0;
   out_1469612526163304898[29] = 0;
   out_1469612526163304898[30] = 0;
   out_1469612526163304898[31] = 0;
   out_1469612526163304898[32] = 0;
   out_1469612526163304898[33] = 0;
   out_1469612526163304898[34] = 0;
   out_1469612526163304898[35] = 0;
   out_1469612526163304898[36] = 0;
   out_1469612526163304898[37] = 0;
   out_1469612526163304898[38] = 0;
   out_1469612526163304898[39] = 0;
   out_1469612526163304898[40] = 0;
   out_1469612526163304898[41] = 0;
   out_1469612526163304898[42] = 0;
   out_1469612526163304898[43] = 0;
   out_1469612526163304898[44] = 1;
   out_1469612526163304898[45] = 0;
   out_1469612526163304898[46] = 0;
   out_1469612526163304898[47] = 0;
   out_1469612526163304898[48] = 0;
   out_1469612526163304898[49] = 0;
   out_1469612526163304898[50] = 0;
   out_1469612526163304898[51] = 0;
   out_1469612526163304898[52] = 0;
   out_1469612526163304898[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_2103793852715952646) {
  err_fun(nom_x, delta_x, out_2103793852715952646);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7486425640275390002) {
  inv_err_fun(nom_x, true_x, out_7486425640275390002);
}
void pose_H_mod_fun(double *state, double *out_3511508283555031915) {
  H_mod_fun(state, out_3511508283555031915);
}
void pose_f_fun(double *state, double dt, double *out_4315192145077905717) {
  f_fun(state,  dt, out_4315192145077905717);
}
void pose_F_fun(double *state, double dt, double *out_5323785267198531003) {
  F_fun(state,  dt, out_5323785267198531003);
}
void pose_h_4(double *state, double *unused, double *out_1791351456807006336) {
  h_4(state, unused, out_1791351456807006336);
}
void pose_H_4(double *state, double *unused, double *out_5432853382502789427) {
  H_4(state, unused, out_5432853382502789427);
}
void pose_h_10(double *state, double *unused, double *out_505052022544704925) {
  h_10(state, unused, out_505052022544704925);
}
void pose_H_10(double *state, double *unused, double *out_5103853662812526892) {
  H_10(state, unused, out_5103853662812526892);
}
void pose_h_13(double *state, double *unused, double *out_8955651205854686458) {
  h_13(state, unused, out_8955651205854686458);
}
void pose_H_13(double *state, double *unused, double *out_2220579557170456626) {
  H_13(state, unused, out_2220579557170456626);
}
void pose_h_14(double *state, double *unused, double *out_1905155691463244381) {
  h_14(state, unused, out_1905155691463244381);
}
void pose_H_14(double *state, double *unused, double *out_1469612526163304898) {
  H_14(state, unused, out_1469612526163304898);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)

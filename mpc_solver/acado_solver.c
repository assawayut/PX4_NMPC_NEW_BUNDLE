/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation( void )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 9 + 8];

acadoWorkspace.state[117] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[118] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[119] = acadoVariables.u[lRun1 * 3 + 2];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 9] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 9 + 9];
acadoWorkspace.d[lRun1 * 9 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 9 + 10];
acadoWorkspace.d[lRun1 * 9 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 9 + 11];
acadoWorkspace.d[lRun1 * 9 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 9 + 12];
acadoWorkspace.d[lRun1 * 9 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 9 + 13];
acadoWorkspace.d[lRun1 * 9 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 9 + 14];
acadoWorkspace.d[lRun1 * 9 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 9 + 15];
acadoWorkspace.d[lRun1 * 9 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 9 + 16];
acadoWorkspace.d[lRun1 * 9 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 9 + 17];

acadoWorkspace.evGx[lRun1 * 81] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 81 + 1] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 81 + 2] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 81 + 3] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 81 + 4] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 81 + 5] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 81 + 6] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 81 + 7] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 81 + 8] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 81 + 9] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 81 + 10] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 81 + 11] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 81 + 12] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 81 + 13] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 81 + 14] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 81 + 15] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 81 + 16] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 81 + 17] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 81 + 18] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 81 + 19] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 81 + 20] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 81 + 21] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 81 + 22] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 81 + 23] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 81 + 24] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 81 + 25] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 81 + 26] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 81 + 27] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 81 + 28] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 81 + 29] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 81 + 30] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 81 + 31] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 81 + 32] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 81 + 33] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 81 + 34] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 81 + 35] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 81 + 36] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 81 + 37] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 81 + 38] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 81 + 39] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 81 + 40] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 81 + 41] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 81 + 42] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 81 + 43] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 81 + 44] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 81 + 45] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 81 + 46] = acadoWorkspace.state[55];
acadoWorkspace.evGx[lRun1 * 81 + 47] = acadoWorkspace.state[56];
acadoWorkspace.evGx[lRun1 * 81 + 48] = acadoWorkspace.state[57];
acadoWorkspace.evGx[lRun1 * 81 + 49] = acadoWorkspace.state[58];
acadoWorkspace.evGx[lRun1 * 81 + 50] = acadoWorkspace.state[59];
acadoWorkspace.evGx[lRun1 * 81 + 51] = acadoWorkspace.state[60];
acadoWorkspace.evGx[lRun1 * 81 + 52] = acadoWorkspace.state[61];
acadoWorkspace.evGx[lRun1 * 81 + 53] = acadoWorkspace.state[62];
acadoWorkspace.evGx[lRun1 * 81 + 54] = acadoWorkspace.state[63];
acadoWorkspace.evGx[lRun1 * 81 + 55] = acadoWorkspace.state[64];
acadoWorkspace.evGx[lRun1 * 81 + 56] = acadoWorkspace.state[65];
acadoWorkspace.evGx[lRun1 * 81 + 57] = acadoWorkspace.state[66];
acadoWorkspace.evGx[lRun1 * 81 + 58] = acadoWorkspace.state[67];
acadoWorkspace.evGx[lRun1 * 81 + 59] = acadoWorkspace.state[68];
acadoWorkspace.evGx[lRun1 * 81 + 60] = acadoWorkspace.state[69];
acadoWorkspace.evGx[lRun1 * 81 + 61] = acadoWorkspace.state[70];
acadoWorkspace.evGx[lRun1 * 81 + 62] = acadoWorkspace.state[71];
acadoWorkspace.evGx[lRun1 * 81 + 63] = acadoWorkspace.state[72];
acadoWorkspace.evGx[lRun1 * 81 + 64] = acadoWorkspace.state[73];
acadoWorkspace.evGx[lRun1 * 81 + 65] = acadoWorkspace.state[74];
acadoWorkspace.evGx[lRun1 * 81 + 66] = acadoWorkspace.state[75];
acadoWorkspace.evGx[lRun1 * 81 + 67] = acadoWorkspace.state[76];
acadoWorkspace.evGx[lRun1 * 81 + 68] = acadoWorkspace.state[77];
acadoWorkspace.evGx[lRun1 * 81 + 69] = acadoWorkspace.state[78];
acadoWorkspace.evGx[lRun1 * 81 + 70] = acadoWorkspace.state[79];
acadoWorkspace.evGx[lRun1 * 81 + 71] = acadoWorkspace.state[80];
acadoWorkspace.evGx[lRun1 * 81 + 72] = acadoWorkspace.state[81];
acadoWorkspace.evGx[lRun1 * 81 + 73] = acadoWorkspace.state[82];
acadoWorkspace.evGx[lRun1 * 81 + 74] = acadoWorkspace.state[83];
acadoWorkspace.evGx[lRun1 * 81 + 75] = acadoWorkspace.state[84];
acadoWorkspace.evGx[lRun1 * 81 + 76] = acadoWorkspace.state[85];
acadoWorkspace.evGx[lRun1 * 81 + 77] = acadoWorkspace.state[86];
acadoWorkspace.evGx[lRun1 * 81 + 78] = acadoWorkspace.state[87];
acadoWorkspace.evGx[lRun1 * 81 + 79] = acadoWorkspace.state[88];
acadoWorkspace.evGx[lRun1 * 81 + 80] = acadoWorkspace.state[89];

acadoWorkspace.evGu[lRun1 * 27] = acadoWorkspace.state[90];
acadoWorkspace.evGu[lRun1 * 27 + 1] = acadoWorkspace.state[91];
acadoWorkspace.evGu[lRun1 * 27 + 2] = acadoWorkspace.state[92];
acadoWorkspace.evGu[lRun1 * 27 + 3] = acadoWorkspace.state[93];
acadoWorkspace.evGu[lRun1 * 27 + 4] = acadoWorkspace.state[94];
acadoWorkspace.evGu[lRun1 * 27 + 5] = acadoWorkspace.state[95];
acadoWorkspace.evGu[lRun1 * 27 + 6] = acadoWorkspace.state[96];
acadoWorkspace.evGu[lRun1 * 27 + 7] = acadoWorkspace.state[97];
acadoWorkspace.evGu[lRun1 * 27 + 8] = acadoWorkspace.state[98];
acadoWorkspace.evGu[lRun1 * 27 + 9] = acadoWorkspace.state[99];
acadoWorkspace.evGu[lRun1 * 27 + 10] = acadoWorkspace.state[100];
acadoWorkspace.evGu[lRun1 * 27 + 11] = acadoWorkspace.state[101];
acadoWorkspace.evGu[lRun1 * 27 + 12] = acadoWorkspace.state[102];
acadoWorkspace.evGu[lRun1 * 27 + 13] = acadoWorkspace.state[103];
acadoWorkspace.evGu[lRun1 * 27 + 14] = acadoWorkspace.state[104];
acadoWorkspace.evGu[lRun1 * 27 + 15] = acadoWorkspace.state[105];
acadoWorkspace.evGu[lRun1 * 27 + 16] = acadoWorkspace.state[106];
acadoWorkspace.evGu[lRun1 * 27 + 17] = acadoWorkspace.state[107];
acadoWorkspace.evGu[lRun1 * 27 + 18] = acadoWorkspace.state[108];
acadoWorkspace.evGu[lRun1 * 27 + 19] = acadoWorkspace.state[109];
acadoWorkspace.evGu[lRun1 * 27 + 20] = acadoWorkspace.state[110];
acadoWorkspace.evGu[lRun1 * 27 + 21] = acadoWorkspace.state[111];
acadoWorkspace.evGu[lRun1 * 27 + 22] = acadoWorkspace.state[112];
acadoWorkspace.evGu[lRun1 * 27 + 23] = acadoWorkspace.state[113];
acadoWorkspace.evGu[lRun1 * 27 + 24] = acadoWorkspace.state[114];
acadoWorkspace.evGu[lRun1 * 27 + 25] = acadoWorkspace.state[115];
acadoWorkspace.evGu[lRun1 * 27 + 26] = acadoWorkspace.state[116];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = u[0];
out[10] = u[1];
out[11] = u[2];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ2[60] = +tmpObjS[60];
tmpQ2[61] = +tmpObjS[61];
tmpQ2[62] = +tmpObjS[62];
tmpQ2[63] = +tmpObjS[63];
tmpQ2[64] = +tmpObjS[64];
tmpQ2[65] = +tmpObjS[65];
tmpQ2[66] = +tmpObjS[66];
tmpQ2[67] = +tmpObjS[67];
tmpQ2[68] = +tmpObjS[68];
tmpQ2[69] = +tmpObjS[69];
tmpQ2[70] = +tmpObjS[70];
tmpQ2[71] = +tmpObjS[71];
tmpQ2[72] = +tmpObjS[72];
tmpQ2[73] = +tmpObjS[73];
tmpQ2[74] = +tmpObjS[74];
tmpQ2[75] = +tmpObjS[75];
tmpQ2[76] = +tmpObjS[76];
tmpQ2[77] = +tmpObjS[77];
tmpQ2[78] = +tmpObjS[78];
tmpQ2[79] = +tmpObjS[79];
tmpQ2[80] = +tmpObjS[80];
tmpQ2[81] = +tmpObjS[81];
tmpQ2[82] = +tmpObjS[82];
tmpQ2[83] = +tmpObjS[83];
tmpQ2[84] = +tmpObjS[84];
tmpQ2[85] = +tmpObjS[85];
tmpQ2[86] = +tmpObjS[86];
tmpQ2[87] = +tmpObjS[87];
tmpQ2[88] = +tmpObjS[88];
tmpQ2[89] = +tmpObjS[89];
tmpQ2[90] = +tmpObjS[90];
tmpQ2[91] = +tmpObjS[91];
tmpQ2[92] = +tmpObjS[92];
tmpQ2[93] = +tmpObjS[93];
tmpQ2[94] = +tmpObjS[94];
tmpQ2[95] = +tmpObjS[95];
tmpQ2[96] = +tmpObjS[96];
tmpQ2[97] = +tmpObjS[97];
tmpQ2[98] = +tmpObjS[98];
tmpQ2[99] = +tmpObjS[99];
tmpQ2[100] = +tmpObjS[100];
tmpQ2[101] = +tmpObjS[101];
tmpQ2[102] = +tmpObjS[102];
tmpQ2[103] = +tmpObjS[103];
tmpQ2[104] = +tmpObjS[104];
tmpQ2[105] = +tmpObjS[105];
tmpQ2[106] = +tmpObjS[106];
tmpQ2[107] = +tmpObjS[107];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = + tmpQ2[12];
tmpQ1[10] = + tmpQ2[13];
tmpQ1[11] = + tmpQ2[14];
tmpQ1[12] = + tmpQ2[15];
tmpQ1[13] = + tmpQ2[16];
tmpQ1[14] = + tmpQ2[17];
tmpQ1[15] = + tmpQ2[18];
tmpQ1[16] = + tmpQ2[19];
tmpQ1[17] = + tmpQ2[20];
tmpQ1[18] = + tmpQ2[24];
tmpQ1[19] = + tmpQ2[25];
tmpQ1[20] = + tmpQ2[26];
tmpQ1[21] = + tmpQ2[27];
tmpQ1[22] = + tmpQ2[28];
tmpQ1[23] = + tmpQ2[29];
tmpQ1[24] = + tmpQ2[30];
tmpQ1[25] = + tmpQ2[31];
tmpQ1[26] = + tmpQ2[32];
tmpQ1[27] = + tmpQ2[36];
tmpQ1[28] = + tmpQ2[37];
tmpQ1[29] = + tmpQ2[38];
tmpQ1[30] = + tmpQ2[39];
tmpQ1[31] = + tmpQ2[40];
tmpQ1[32] = + tmpQ2[41];
tmpQ1[33] = + tmpQ2[42];
tmpQ1[34] = + tmpQ2[43];
tmpQ1[35] = + tmpQ2[44];
tmpQ1[36] = + tmpQ2[48];
tmpQ1[37] = + tmpQ2[49];
tmpQ1[38] = + tmpQ2[50];
tmpQ1[39] = + tmpQ2[51];
tmpQ1[40] = + tmpQ2[52];
tmpQ1[41] = + tmpQ2[53];
tmpQ1[42] = + tmpQ2[54];
tmpQ1[43] = + tmpQ2[55];
tmpQ1[44] = + tmpQ2[56];
tmpQ1[45] = + tmpQ2[60];
tmpQ1[46] = + tmpQ2[61];
tmpQ1[47] = + tmpQ2[62];
tmpQ1[48] = + tmpQ2[63];
tmpQ1[49] = + tmpQ2[64];
tmpQ1[50] = + tmpQ2[65];
tmpQ1[51] = + tmpQ2[66];
tmpQ1[52] = + tmpQ2[67];
tmpQ1[53] = + tmpQ2[68];
tmpQ1[54] = + tmpQ2[72];
tmpQ1[55] = + tmpQ2[73];
tmpQ1[56] = + tmpQ2[74];
tmpQ1[57] = + tmpQ2[75];
tmpQ1[58] = + tmpQ2[76];
tmpQ1[59] = + tmpQ2[77];
tmpQ1[60] = + tmpQ2[78];
tmpQ1[61] = + tmpQ2[79];
tmpQ1[62] = + tmpQ2[80];
tmpQ1[63] = + tmpQ2[84];
tmpQ1[64] = + tmpQ2[85];
tmpQ1[65] = + tmpQ2[86];
tmpQ1[66] = + tmpQ2[87];
tmpQ1[67] = + tmpQ2[88];
tmpQ1[68] = + tmpQ2[89];
tmpQ1[69] = + tmpQ2[90];
tmpQ1[70] = + tmpQ2[91];
tmpQ1[71] = + tmpQ2[92];
tmpQ1[72] = + tmpQ2[96];
tmpQ1[73] = + tmpQ2[97];
tmpQ1[74] = + tmpQ2[98];
tmpQ1[75] = + tmpQ2[99];
tmpQ1[76] = + tmpQ2[100];
tmpQ1[77] = + tmpQ2[101];
tmpQ1[78] = + tmpQ2[102];
tmpQ1[79] = + tmpQ2[103];
tmpQ1[80] = + tmpQ2[104];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[108];
tmpR2[1] = +tmpObjS[109];
tmpR2[2] = +tmpObjS[110];
tmpR2[3] = +tmpObjS[111];
tmpR2[4] = +tmpObjS[112];
tmpR2[5] = +tmpObjS[113];
tmpR2[6] = +tmpObjS[114];
tmpR2[7] = +tmpObjS[115];
tmpR2[8] = +tmpObjS[116];
tmpR2[9] = +tmpObjS[117];
tmpR2[10] = +tmpObjS[118];
tmpR2[11] = +tmpObjS[119];
tmpR2[12] = +tmpObjS[120];
tmpR2[13] = +tmpObjS[121];
tmpR2[14] = +tmpObjS[122];
tmpR2[15] = +tmpObjS[123];
tmpR2[16] = +tmpObjS[124];
tmpR2[17] = +tmpObjS[125];
tmpR2[18] = +tmpObjS[126];
tmpR2[19] = +tmpObjS[127];
tmpR2[20] = +tmpObjS[128];
tmpR2[21] = +tmpObjS[129];
tmpR2[22] = +tmpObjS[130];
tmpR2[23] = +tmpObjS[131];
tmpR2[24] = +tmpObjS[132];
tmpR2[25] = +tmpObjS[133];
tmpR2[26] = +tmpObjS[134];
tmpR2[27] = +tmpObjS[135];
tmpR2[28] = +tmpObjS[136];
tmpR2[29] = +tmpObjS[137];
tmpR2[30] = +tmpObjS[138];
tmpR2[31] = +tmpObjS[139];
tmpR2[32] = +tmpObjS[140];
tmpR2[33] = +tmpObjS[141];
tmpR2[34] = +tmpObjS[142];
tmpR2[35] = +tmpObjS[143];
tmpR1[0] = + tmpR2[9];
tmpR1[1] = + tmpR2[10];
tmpR1[2] = + tmpR2[11];
tmpR1[3] = + tmpR2[21];
tmpR1[4] = + tmpR2[22];
tmpR1[5] = + tmpR2[23];
tmpR1[6] = + tmpR2[33];
tmpR1[7] = + tmpR2[34];
tmpR1[8] = + tmpR2[35];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN2[49] = +tmpObjSEndTerm[49];
tmpQN2[50] = +tmpObjSEndTerm[50];
tmpQN2[51] = +tmpObjSEndTerm[51];
tmpQN2[52] = +tmpObjSEndTerm[52];
tmpQN2[53] = +tmpObjSEndTerm[53];
tmpQN2[54] = +tmpObjSEndTerm[54];
tmpQN2[55] = +tmpObjSEndTerm[55];
tmpQN2[56] = +tmpObjSEndTerm[56];
tmpQN2[57] = +tmpObjSEndTerm[57];
tmpQN2[58] = +tmpObjSEndTerm[58];
tmpQN2[59] = +tmpObjSEndTerm[59];
tmpQN2[60] = +tmpObjSEndTerm[60];
tmpQN2[61] = +tmpObjSEndTerm[61];
tmpQN2[62] = +tmpObjSEndTerm[62];
tmpQN2[63] = +tmpObjSEndTerm[63];
tmpQN2[64] = +tmpObjSEndTerm[64];
tmpQN2[65] = +tmpObjSEndTerm[65];
tmpQN2[66] = +tmpObjSEndTerm[66];
tmpQN2[67] = +tmpObjSEndTerm[67];
tmpQN2[68] = +tmpObjSEndTerm[68];
tmpQN2[69] = +tmpObjSEndTerm[69];
tmpQN2[70] = +tmpObjSEndTerm[70];
tmpQN2[71] = +tmpObjSEndTerm[71];
tmpQN2[72] = +tmpObjSEndTerm[72];
tmpQN2[73] = +tmpObjSEndTerm[73];
tmpQN2[74] = +tmpObjSEndTerm[74];
tmpQN2[75] = +tmpObjSEndTerm[75];
tmpQN2[76] = +tmpObjSEndTerm[76];
tmpQN2[77] = +tmpObjSEndTerm[77];
tmpQN2[78] = +tmpObjSEndTerm[78];
tmpQN2[79] = +tmpObjSEndTerm[79];
tmpQN2[80] = +tmpObjSEndTerm[80];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
tmpQN1[36] = + tmpQN2[36];
tmpQN1[37] = + tmpQN2[37];
tmpQN1[38] = + tmpQN2[38];
tmpQN1[39] = + tmpQN2[39];
tmpQN1[40] = + tmpQN2[40];
tmpQN1[41] = + tmpQN2[41];
tmpQN1[42] = + tmpQN2[42];
tmpQN1[43] = + tmpQN2[43];
tmpQN1[44] = + tmpQN2[44];
tmpQN1[45] = + tmpQN2[45];
tmpQN1[46] = + tmpQN2[46];
tmpQN1[47] = + tmpQN2[47];
tmpQN1[48] = + tmpQN2[48];
tmpQN1[49] = + tmpQN2[49];
tmpQN1[50] = + tmpQN2[50];
tmpQN1[51] = + tmpQN2[51];
tmpQN1[52] = + tmpQN2[52];
tmpQN1[53] = + tmpQN2[53];
tmpQN1[54] = + tmpQN2[54];
tmpQN1[55] = + tmpQN2[55];
tmpQN1[56] = + tmpQN2[56];
tmpQN1[57] = + tmpQN2[57];
tmpQN1[58] = + tmpQN2[58];
tmpQN1[59] = + tmpQN2[59];
tmpQN1[60] = + tmpQN2[60];
tmpQN1[61] = + tmpQN2[61];
tmpQN1[62] = + tmpQN2[62];
tmpQN1[63] = + tmpQN2[63];
tmpQN1[64] = + tmpQN2[64];
tmpQN1[65] = + tmpQN2[65];
tmpQN1[66] = + tmpQN2[66];
tmpQN1[67] = + tmpQN2[67];
tmpQN1[68] = + tmpQN2[68];
tmpQN1[69] = + tmpQN2[69];
tmpQN1[70] = + tmpQN2[70];
tmpQN1[71] = + tmpQN2[71];
tmpQN1[72] = + tmpQN2[72];
tmpQN1[73] = + tmpQN2[73];
tmpQN1[74] = + tmpQN2[74];
tmpQN1[75] = + tmpQN2[75];
tmpQN1[76] = + tmpQN2[76];
tmpQN1[77] = + tmpQN2[77];
tmpQN1[78] = + tmpQN2[78];
tmpQN1[79] = + tmpQN2[79];
tmpQN1[80] = + tmpQN2[80];
}

void acado_evaluateObjective( void )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 12] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 12 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 12 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 12 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 12 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 12 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 12 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 12 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 12 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 12 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 12 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 12 + 11] = acadoWorkspace.objValueOut[11];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 144 ]), &(acadoWorkspace.Q1[ runObj * 81 ]), &(acadoWorkspace.Q2[ runObj * 108 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 144 ]), &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 36 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[90];
acadoWorkspace.objValueIn[1] = acadoVariables.x[91];
acadoWorkspace.objValueIn[2] = acadoVariables.x[92];
acadoWorkspace.objValueIn[3] = acadoVariables.x[93];
acadoWorkspace.objValueIn[4] = acadoVariables.x[94];
acadoWorkspace.objValueIn[5] = acadoVariables.x[95];
acadoWorkspace.objValueIn[6] = acadoVariables.x[96];
acadoWorkspace.objValueIn[7] = acadoVariables.x[97];
acadoWorkspace.objValueIn[8] = acadoVariables.x[98];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8];
dNew[1] += + Gx1[9]*dOld[0] + Gx1[10]*dOld[1] + Gx1[11]*dOld[2] + Gx1[12]*dOld[3] + Gx1[13]*dOld[4] + Gx1[14]*dOld[5] + Gx1[15]*dOld[6] + Gx1[16]*dOld[7] + Gx1[17]*dOld[8];
dNew[2] += + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5] + Gx1[24]*dOld[6] + Gx1[25]*dOld[7] + Gx1[26]*dOld[8];
dNew[3] += + Gx1[27]*dOld[0] + Gx1[28]*dOld[1] + Gx1[29]*dOld[2] + Gx1[30]*dOld[3] + Gx1[31]*dOld[4] + Gx1[32]*dOld[5] + Gx1[33]*dOld[6] + Gx1[34]*dOld[7] + Gx1[35]*dOld[8];
dNew[4] += + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8];
dNew[5] += + Gx1[45]*dOld[0] + Gx1[46]*dOld[1] + Gx1[47]*dOld[2] + Gx1[48]*dOld[3] + Gx1[49]*dOld[4] + Gx1[50]*dOld[5] + Gx1[51]*dOld[6] + Gx1[52]*dOld[7] + Gx1[53]*dOld[8];
dNew[6] += + Gx1[54]*dOld[0] + Gx1[55]*dOld[1] + Gx1[56]*dOld[2] + Gx1[57]*dOld[3] + Gx1[58]*dOld[4] + Gx1[59]*dOld[5] + Gx1[60]*dOld[6] + Gx1[61]*dOld[7] + Gx1[62]*dOld[8];
dNew[7] += + Gx1[63]*dOld[0] + Gx1[64]*dOld[1] + Gx1[65]*dOld[2] + Gx1[66]*dOld[3] + Gx1[67]*dOld[4] + Gx1[68]*dOld[5] + Gx1[69]*dOld[6] + Gx1[70]*dOld[7] + Gx1[71]*dOld[8];
dNew[8] += + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8];
}

void acado_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
Gx2[64] = Gx1[64];
Gx2[65] = Gx1[65];
Gx2[66] = Gx1[66];
Gx2[67] = Gx1[67];
Gx2[68] = Gx1[68];
Gx2[69] = Gx1[69];
Gx2[70] = Gx1[70];
Gx2[71] = Gx1[71];
Gx2[72] = Gx1[72];
Gx2[73] = Gx1[73];
Gx2[74] = Gx1[74];
Gx2[75] = Gx1[75];
Gx2[76] = Gx1[76];
Gx2[77] = Gx1[77];
Gx2[78] = Gx1[78];
Gx2[79] = Gx1[79];
Gx2[80] = Gx1[80];
}

void acado_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[36] + Gx1[5]*Gx2[45] + Gx1[6]*Gx2[54] + Gx1[7]*Gx2[63] + Gx1[8]*Gx2[72];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[28] + Gx1[4]*Gx2[37] + Gx1[5]*Gx2[46] + Gx1[6]*Gx2[55] + Gx1[7]*Gx2[64] + Gx1[8]*Gx2[73];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[29] + Gx1[4]*Gx2[38] + Gx1[5]*Gx2[47] + Gx1[6]*Gx2[56] + Gx1[7]*Gx2[65] + Gx1[8]*Gx2[74];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[39] + Gx1[5]*Gx2[48] + Gx1[6]*Gx2[57] + Gx1[7]*Gx2[66] + Gx1[8]*Gx2[75];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[40] + Gx1[5]*Gx2[49] + Gx1[6]*Gx2[58] + Gx1[7]*Gx2[67] + Gx1[8]*Gx2[76];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[32] + Gx1[4]*Gx2[41] + Gx1[5]*Gx2[50] + Gx1[6]*Gx2[59] + Gx1[7]*Gx2[68] + Gx1[8]*Gx2[77];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[24] + Gx1[3]*Gx2[33] + Gx1[4]*Gx2[42] + Gx1[5]*Gx2[51] + Gx1[6]*Gx2[60] + Gx1[7]*Gx2[69] + Gx1[8]*Gx2[78];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[16] + Gx1[2]*Gx2[25] + Gx1[3]*Gx2[34] + Gx1[4]*Gx2[43] + Gx1[5]*Gx2[52] + Gx1[6]*Gx2[61] + Gx1[7]*Gx2[70] + Gx1[8]*Gx2[79];
Gx3[8] = + Gx1[0]*Gx2[8] + Gx1[1]*Gx2[17] + Gx1[2]*Gx2[26] + Gx1[3]*Gx2[35] + Gx1[4]*Gx2[44] + Gx1[5]*Gx2[53] + Gx1[6]*Gx2[62] + Gx1[7]*Gx2[71] + Gx1[8]*Gx2[80];
Gx3[9] = + Gx1[9]*Gx2[0] + Gx1[10]*Gx2[9] + Gx1[11]*Gx2[18] + Gx1[12]*Gx2[27] + Gx1[13]*Gx2[36] + Gx1[14]*Gx2[45] + Gx1[15]*Gx2[54] + Gx1[16]*Gx2[63] + Gx1[17]*Gx2[72];
Gx3[10] = + Gx1[9]*Gx2[1] + Gx1[10]*Gx2[10] + Gx1[11]*Gx2[19] + Gx1[12]*Gx2[28] + Gx1[13]*Gx2[37] + Gx1[14]*Gx2[46] + Gx1[15]*Gx2[55] + Gx1[16]*Gx2[64] + Gx1[17]*Gx2[73];
Gx3[11] = + Gx1[9]*Gx2[2] + Gx1[10]*Gx2[11] + Gx1[11]*Gx2[20] + Gx1[12]*Gx2[29] + Gx1[13]*Gx2[38] + Gx1[14]*Gx2[47] + Gx1[15]*Gx2[56] + Gx1[16]*Gx2[65] + Gx1[17]*Gx2[74];
Gx3[12] = + Gx1[9]*Gx2[3] + Gx1[10]*Gx2[12] + Gx1[11]*Gx2[21] + Gx1[12]*Gx2[30] + Gx1[13]*Gx2[39] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[57] + Gx1[16]*Gx2[66] + Gx1[17]*Gx2[75];
Gx3[13] = + Gx1[9]*Gx2[4] + Gx1[10]*Gx2[13] + Gx1[11]*Gx2[22] + Gx1[12]*Gx2[31] + Gx1[13]*Gx2[40] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[58] + Gx1[16]*Gx2[67] + Gx1[17]*Gx2[76];
Gx3[14] = + Gx1[9]*Gx2[5] + Gx1[10]*Gx2[14] + Gx1[11]*Gx2[23] + Gx1[12]*Gx2[32] + Gx1[13]*Gx2[41] + Gx1[14]*Gx2[50] + Gx1[15]*Gx2[59] + Gx1[16]*Gx2[68] + Gx1[17]*Gx2[77];
Gx3[15] = + Gx1[9]*Gx2[6] + Gx1[10]*Gx2[15] + Gx1[11]*Gx2[24] + Gx1[12]*Gx2[33] + Gx1[13]*Gx2[42] + Gx1[14]*Gx2[51] + Gx1[15]*Gx2[60] + Gx1[16]*Gx2[69] + Gx1[17]*Gx2[78];
Gx3[16] = + Gx1[9]*Gx2[7] + Gx1[10]*Gx2[16] + Gx1[11]*Gx2[25] + Gx1[12]*Gx2[34] + Gx1[13]*Gx2[43] + Gx1[14]*Gx2[52] + Gx1[15]*Gx2[61] + Gx1[16]*Gx2[70] + Gx1[17]*Gx2[79];
Gx3[17] = + Gx1[9]*Gx2[8] + Gx1[10]*Gx2[17] + Gx1[11]*Gx2[26] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[44] + Gx1[14]*Gx2[53] + Gx1[15]*Gx2[62] + Gx1[16]*Gx2[71] + Gx1[17]*Gx2[80];
Gx3[18] = + Gx1[18]*Gx2[0] + Gx1[19]*Gx2[9] + Gx1[20]*Gx2[18] + Gx1[21]*Gx2[27] + Gx1[22]*Gx2[36] + Gx1[23]*Gx2[45] + Gx1[24]*Gx2[54] + Gx1[25]*Gx2[63] + Gx1[26]*Gx2[72];
Gx3[19] = + Gx1[18]*Gx2[1] + Gx1[19]*Gx2[10] + Gx1[20]*Gx2[19] + Gx1[21]*Gx2[28] + Gx1[22]*Gx2[37] + Gx1[23]*Gx2[46] + Gx1[24]*Gx2[55] + Gx1[25]*Gx2[64] + Gx1[26]*Gx2[73];
Gx3[20] = + Gx1[18]*Gx2[2] + Gx1[19]*Gx2[11] + Gx1[20]*Gx2[20] + Gx1[21]*Gx2[29] + Gx1[22]*Gx2[38] + Gx1[23]*Gx2[47] + Gx1[24]*Gx2[56] + Gx1[25]*Gx2[65] + Gx1[26]*Gx2[74];
Gx3[21] = + Gx1[18]*Gx2[3] + Gx1[19]*Gx2[12] + Gx1[20]*Gx2[21] + Gx1[21]*Gx2[30] + Gx1[22]*Gx2[39] + Gx1[23]*Gx2[48] + Gx1[24]*Gx2[57] + Gx1[25]*Gx2[66] + Gx1[26]*Gx2[75];
Gx3[22] = + Gx1[18]*Gx2[4] + Gx1[19]*Gx2[13] + Gx1[20]*Gx2[22] + Gx1[21]*Gx2[31] + Gx1[22]*Gx2[40] + Gx1[23]*Gx2[49] + Gx1[24]*Gx2[58] + Gx1[25]*Gx2[67] + Gx1[26]*Gx2[76];
Gx3[23] = + Gx1[18]*Gx2[5] + Gx1[19]*Gx2[14] + Gx1[20]*Gx2[23] + Gx1[21]*Gx2[32] + Gx1[22]*Gx2[41] + Gx1[23]*Gx2[50] + Gx1[24]*Gx2[59] + Gx1[25]*Gx2[68] + Gx1[26]*Gx2[77];
Gx3[24] = + Gx1[18]*Gx2[6] + Gx1[19]*Gx2[15] + Gx1[20]*Gx2[24] + Gx1[21]*Gx2[33] + Gx1[22]*Gx2[42] + Gx1[23]*Gx2[51] + Gx1[24]*Gx2[60] + Gx1[25]*Gx2[69] + Gx1[26]*Gx2[78];
Gx3[25] = + Gx1[18]*Gx2[7] + Gx1[19]*Gx2[16] + Gx1[20]*Gx2[25] + Gx1[21]*Gx2[34] + Gx1[22]*Gx2[43] + Gx1[23]*Gx2[52] + Gx1[24]*Gx2[61] + Gx1[25]*Gx2[70] + Gx1[26]*Gx2[79];
Gx3[26] = + Gx1[18]*Gx2[8] + Gx1[19]*Gx2[17] + Gx1[20]*Gx2[26] + Gx1[21]*Gx2[35] + Gx1[22]*Gx2[44] + Gx1[23]*Gx2[53] + Gx1[24]*Gx2[62] + Gx1[25]*Gx2[71] + Gx1[26]*Gx2[80];
Gx3[27] = + Gx1[27]*Gx2[0] + Gx1[28]*Gx2[9] + Gx1[29]*Gx2[18] + Gx1[30]*Gx2[27] + Gx1[31]*Gx2[36] + Gx1[32]*Gx2[45] + Gx1[33]*Gx2[54] + Gx1[34]*Gx2[63] + Gx1[35]*Gx2[72];
Gx3[28] = + Gx1[27]*Gx2[1] + Gx1[28]*Gx2[10] + Gx1[29]*Gx2[19] + Gx1[30]*Gx2[28] + Gx1[31]*Gx2[37] + Gx1[32]*Gx2[46] + Gx1[33]*Gx2[55] + Gx1[34]*Gx2[64] + Gx1[35]*Gx2[73];
Gx3[29] = + Gx1[27]*Gx2[2] + Gx1[28]*Gx2[11] + Gx1[29]*Gx2[20] + Gx1[30]*Gx2[29] + Gx1[31]*Gx2[38] + Gx1[32]*Gx2[47] + Gx1[33]*Gx2[56] + Gx1[34]*Gx2[65] + Gx1[35]*Gx2[74];
Gx3[30] = + Gx1[27]*Gx2[3] + Gx1[28]*Gx2[12] + Gx1[29]*Gx2[21] + Gx1[30]*Gx2[30] + Gx1[31]*Gx2[39] + Gx1[32]*Gx2[48] + Gx1[33]*Gx2[57] + Gx1[34]*Gx2[66] + Gx1[35]*Gx2[75];
Gx3[31] = + Gx1[27]*Gx2[4] + Gx1[28]*Gx2[13] + Gx1[29]*Gx2[22] + Gx1[30]*Gx2[31] + Gx1[31]*Gx2[40] + Gx1[32]*Gx2[49] + Gx1[33]*Gx2[58] + Gx1[34]*Gx2[67] + Gx1[35]*Gx2[76];
Gx3[32] = + Gx1[27]*Gx2[5] + Gx1[28]*Gx2[14] + Gx1[29]*Gx2[23] + Gx1[30]*Gx2[32] + Gx1[31]*Gx2[41] + Gx1[32]*Gx2[50] + Gx1[33]*Gx2[59] + Gx1[34]*Gx2[68] + Gx1[35]*Gx2[77];
Gx3[33] = + Gx1[27]*Gx2[6] + Gx1[28]*Gx2[15] + Gx1[29]*Gx2[24] + Gx1[30]*Gx2[33] + Gx1[31]*Gx2[42] + Gx1[32]*Gx2[51] + Gx1[33]*Gx2[60] + Gx1[34]*Gx2[69] + Gx1[35]*Gx2[78];
Gx3[34] = + Gx1[27]*Gx2[7] + Gx1[28]*Gx2[16] + Gx1[29]*Gx2[25] + Gx1[30]*Gx2[34] + Gx1[31]*Gx2[43] + Gx1[32]*Gx2[52] + Gx1[33]*Gx2[61] + Gx1[34]*Gx2[70] + Gx1[35]*Gx2[79];
Gx3[35] = + Gx1[27]*Gx2[8] + Gx1[28]*Gx2[17] + Gx1[29]*Gx2[26] + Gx1[30]*Gx2[35] + Gx1[31]*Gx2[44] + Gx1[32]*Gx2[53] + Gx1[33]*Gx2[62] + Gx1[34]*Gx2[71] + Gx1[35]*Gx2[80];
Gx3[36] = + Gx1[36]*Gx2[0] + Gx1[37]*Gx2[9] + Gx1[38]*Gx2[18] + Gx1[39]*Gx2[27] + Gx1[40]*Gx2[36] + Gx1[41]*Gx2[45] + Gx1[42]*Gx2[54] + Gx1[43]*Gx2[63] + Gx1[44]*Gx2[72];
Gx3[37] = + Gx1[36]*Gx2[1] + Gx1[37]*Gx2[10] + Gx1[38]*Gx2[19] + Gx1[39]*Gx2[28] + Gx1[40]*Gx2[37] + Gx1[41]*Gx2[46] + Gx1[42]*Gx2[55] + Gx1[43]*Gx2[64] + Gx1[44]*Gx2[73];
Gx3[38] = + Gx1[36]*Gx2[2] + Gx1[37]*Gx2[11] + Gx1[38]*Gx2[20] + Gx1[39]*Gx2[29] + Gx1[40]*Gx2[38] + Gx1[41]*Gx2[47] + Gx1[42]*Gx2[56] + Gx1[43]*Gx2[65] + Gx1[44]*Gx2[74];
Gx3[39] = + Gx1[36]*Gx2[3] + Gx1[37]*Gx2[12] + Gx1[38]*Gx2[21] + Gx1[39]*Gx2[30] + Gx1[40]*Gx2[39] + Gx1[41]*Gx2[48] + Gx1[42]*Gx2[57] + Gx1[43]*Gx2[66] + Gx1[44]*Gx2[75];
Gx3[40] = + Gx1[36]*Gx2[4] + Gx1[37]*Gx2[13] + Gx1[38]*Gx2[22] + Gx1[39]*Gx2[31] + Gx1[40]*Gx2[40] + Gx1[41]*Gx2[49] + Gx1[42]*Gx2[58] + Gx1[43]*Gx2[67] + Gx1[44]*Gx2[76];
Gx3[41] = + Gx1[36]*Gx2[5] + Gx1[37]*Gx2[14] + Gx1[38]*Gx2[23] + Gx1[39]*Gx2[32] + Gx1[40]*Gx2[41] + Gx1[41]*Gx2[50] + Gx1[42]*Gx2[59] + Gx1[43]*Gx2[68] + Gx1[44]*Gx2[77];
Gx3[42] = + Gx1[36]*Gx2[6] + Gx1[37]*Gx2[15] + Gx1[38]*Gx2[24] + Gx1[39]*Gx2[33] + Gx1[40]*Gx2[42] + Gx1[41]*Gx2[51] + Gx1[42]*Gx2[60] + Gx1[43]*Gx2[69] + Gx1[44]*Gx2[78];
Gx3[43] = + Gx1[36]*Gx2[7] + Gx1[37]*Gx2[16] + Gx1[38]*Gx2[25] + Gx1[39]*Gx2[34] + Gx1[40]*Gx2[43] + Gx1[41]*Gx2[52] + Gx1[42]*Gx2[61] + Gx1[43]*Gx2[70] + Gx1[44]*Gx2[79];
Gx3[44] = + Gx1[36]*Gx2[8] + Gx1[37]*Gx2[17] + Gx1[38]*Gx2[26] + Gx1[39]*Gx2[35] + Gx1[40]*Gx2[44] + Gx1[41]*Gx2[53] + Gx1[42]*Gx2[62] + Gx1[43]*Gx2[71] + Gx1[44]*Gx2[80];
Gx3[45] = + Gx1[45]*Gx2[0] + Gx1[46]*Gx2[9] + Gx1[47]*Gx2[18] + Gx1[48]*Gx2[27] + Gx1[49]*Gx2[36] + Gx1[50]*Gx2[45] + Gx1[51]*Gx2[54] + Gx1[52]*Gx2[63] + Gx1[53]*Gx2[72];
Gx3[46] = + Gx1[45]*Gx2[1] + Gx1[46]*Gx2[10] + Gx1[47]*Gx2[19] + Gx1[48]*Gx2[28] + Gx1[49]*Gx2[37] + Gx1[50]*Gx2[46] + Gx1[51]*Gx2[55] + Gx1[52]*Gx2[64] + Gx1[53]*Gx2[73];
Gx3[47] = + Gx1[45]*Gx2[2] + Gx1[46]*Gx2[11] + Gx1[47]*Gx2[20] + Gx1[48]*Gx2[29] + Gx1[49]*Gx2[38] + Gx1[50]*Gx2[47] + Gx1[51]*Gx2[56] + Gx1[52]*Gx2[65] + Gx1[53]*Gx2[74];
Gx3[48] = + Gx1[45]*Gx2[3] + Gx1[46]*Gx2[12] + Gx1[47]*Gx2[21] + Gx1[48]*Gx2[30] + Gx1[49]*Gx2[39] + Gx1[50]*Gx2[48] + Gx1[51]*Gx2[57] + Gx1[52]*Gx2[66] + Gx1[53]*Gx2[75];
Gx3[49] = + Gx1[45]*Gx2[4] + Gx1[46]*Gx2[13] + Gx1[47]*Gx2[22] + Gx1[48]*Gx2[31] + Gx1[49]*Gx2[40] + Gx1[50]*Gx2[49] + Gx1[51]*Gx2[58] + Gx1[52]*Gx2[67] + Gx1[53]*Gx2[76];
Gx3[50] = + Gx1[45]*Gx2[5] + Gx1[46]*Gx2[14] + Gx1[47]*Gx2[23] + Gx1[48]*Gx2[32] + Gx1[49]*Gx2[41] + Gx1[50]*Gx2[50] + Gx1[51]*Gx2[59] + Gx1[52]*Gx2[68] + Gx1[53]*Gx2[77];
Gx3[51] = + Gx1[45]*Gx2[6] + Gx1[46]*Gx2[15] + Gx1[47]*Gx2[24] + Gx1[48]*Gx2[33] + Gx1[49]*Gx2[42] + Gx1[50]*Gx2[51] + Gx1[51]*Gx2[60] + Gx1[52]*Gx2[69] + Gx1[53]*Gx2[78];
Gx3[52] = + Gx1[45]*Gx2[7] + Gx1[46]*Gx2[16] + Gx1[47]*Gx2[25] + Gx1[48]*Gx2[34] + Gx1[49]*Gx2[43] + Gx1[50]*Gx2[52] + Gx1[51]*Gx2[61] + Gx1[52]*Gx2[70] + Gx1[53]*Gx2[79];
Gx3[53] = + Gx1[45]*Gx2[8] + Gx1[46]*Gx2[17] + Gx1[47]*Gx2[26] + Gx1[48]*Gx2[35] + Gx1[49]*Gx2[44] + Gx1[50]*Gx2[53] + Gx1[51]*Gx2[62] + Gx1[52]*Gx2[71] + Gx1[53]*Gx2[80];
Gx3[54] = + Gx1[54]*Gx2[0] + Gx1[55]*Gx2[9] + Gx1[56]*Gx2[18] + Gx1[57]*Gx2[27] + Gx1[58]*Gx2[36] + Gx1[59]*Gx2[45] + Gx1[60]*Gx2[54] + Gx1[61]*Gx2[63] + Gx1[62]*Gx2[72];
Gx3[55] = + Gx1[54]*Gx2[1] + Gx1[55]*Gx2[10] + Gx1[56]*Gx2[19] + Gx1[57]*Gx2[28] + Gx1[58]*Gx2[37] + Gx1[59]*Gx2[46] + Gx1[60]*Gx2[55] + Gx1[61]*Gx2[64] + Gx1[62]*Gx2[73];
Gx3[56] = + Gx1[54]*Gx2[2] + Gx1[55]*Gx2[11] + Gx1[56]*Gx2[20] + Gx1[57]*Gx2[29] + Gx1[58]*Gx2[38] + Gx1[59]*Gx2[47] + Gx1[60]*Gx2[56] + Gx1[61]*Gx2[65] + Gx1[62]*Gx2[74];
Gx3[57] = + Gx1[54]*Gx2[3] + Gx1[55]*Gx2[12] + Gx1[56]*Gx2[21] + Gx1[57]*Gx2[30] + Gx1[58]*Gx2[39] + Gx1[59]*Gx2[48] + Gx1[60]*Gx2[57] + Gx1[61]*Gx2[66] + Gx1[62]*Gx2[75];
Gx3[58] = + Gx1[54]*Gx2[4] + Gx1[55]*Gx2[13] + Gx1[56]*Gx2[22] + Gx1[57]*Gx2[31] + Gx1[58]*Gx2[40] + Gx1[59]*Gx2[49] + Gx1[60]*Gx2[58] + Gx1[61]*Gx2[67] + Gx1[62]*Gx2[76];
Gx3[59] = + Gx1[54]*Gx2[5] + Gx1[55]*Gx2[14] + Gx1[56]*Gx2[23] + Gx1[57]*Gx2[32] + Gx1[58]*Gx2[41] + Gx1[59]*Gx2[50] + Gx1[60]*Gx2[59] + Gx1[61]*Gx2[68] + Gx1[62]*Gx2[77];
Gx3[60] = + Gx1[54]*Gx2[6] + Gx1[55]*Gx2[15] + Gx1[56]*Gx2[24] + Gx1[57]*Gx2[33] + Gx1[58]*Gx2[42] + Gx1[59]*Gx2[51] + Gx1[60]*Gx2[60] + Gx1[61]*Gx2[69] + Gx1[62]*Gx2[78];
Gx3[61] = + Gx1[54]*Gx2[7] + Gx1[55]*Gx2[16] + Gx1[56]*Gx2[25] + Gx1[57]*Gx2[34] + Gx1[58]*Gx2[43] + Gx1[59]*Gx2[52] + Gx1[60]*Gx2[61] + Gx1[61]*Gx2[70] + Gx1[62]*Gx2[79];
Gx3[62] = + Gx1[54]*Gx2[8] + Gx1[55]*Gx2[17] + Gx1[56]*Gx2[26] + Gx1[57]*Gx2[35] + Gx1[58]*Gx2[44] + Gx1[59]*Gx2[53] + Gx1[60]*Gx2[62] + Gx1[61]*Gx2[71] + Gx1[62]*Gx2[80];
Gx3[63] = + Gx1[63]*Gx2[0] + Gx1[64]*Gx2[9] + Gx1[65]*Gx2[18] + Gx1[66]*Gx2[27] + Gx1[67]*Gx2[36] + Gx1[68]*Gx2[45] + Gx1[69]*Gx2[54] + Gx1[70]*Gx2[63] + Gx1[71]*Gx2[72];
Gx3[64] = + Gx1[63]*Gx2[1] + Gx1[64]*Gx2[10] + Gx1[65]*Gx2[19] + Gx1[66]*Gx2[28] + Gx1[67]*Gx2[37] + Gx1[68]*Gx2[46] + Gx1[69]*Gx2[55] + Gx1[70]*Gx2[64] + Gx1[71]*Gx2[73];
Gx3[65] = + Gx1[63]*Gx2[2] + Gx1[64]*Gx2[11] + Gx1[65]*Gx2[20] + Gx1[66]*Gx2[29] + Gx1[67]*Gx2[38] + Gx1[68]*Gx2[47] + Gx1[69]*Gx2[56] + Gx1[70]*Gx2[65] + Gx1[71]*Gx2[74];
Gx3[66] = + Gx1[63]*Gx2[3] + Gx1[64]*Gx2[12] + Gx1[65]*Gx2[21] + Gx1[66]*Gx2[30] + Gx1[67]*Gx2[39] + Gx1[68]*Gx2[48] + Gx1[69]*Gx2[57] + Gx1[70]*Gx2[66] + Gx1[71]*Gx2[75];
Gx3[67] = + Gx1[63]*Gx2[4] + Gx1[64]*Gx2[13] + Gx1[65]*Gx2[22] + Gx1[66]*Gx2[31] + Gx1[67]*Gx2[40] + Gx1[68]*Gx2[49] + Gx1[69]*Gx2[58] + Gx1[70]*Gx2[67] + Gx1[71]*Gx2[76];
Gx3[68] = + Gx1[63]*Gx2[5] + Gx1[64]*Gx2[14] + Gx1[65]*Gx2[23] + Gx1[66]*Gx2[32] + Gx1[67]*Gx2[41] + Gx1[68]*Gx2[50] + Gx1[69]*Gx2[59] + Gx1[70]*Gx2[68] + Gx1[71]*Gx2[77];
Gx3[69] = + Gx1[63]*Gx2[6] + Gx1[64]*Gx2[15] + Gx1[65]*Gx2[24] + Gx1[66]*Gx2[33] + Gx1[67]*Gx2[42] + Gx1[68]*Gx2[51] + Gx1[69]*Gx2[60] + Gx1[70]*Gx2[69] + Gx1[71]*Gx2[78];
Gx3[70] = + Gx1[63]*Gx2[7] + Gx1[64]*Gx2[16] + Gx1[65]*Gx2[25] + Gx1[66]*Gx2[34] + Gx1[67]*Gx2[43] + Gx1[68]*Gx2[52] + Gx1[69]*Gx2[61] + Gx1[70]*Gx2[70] + Gx1[71]*Gx2[79];
Gx3[71] = + Gx1[63]*Gx2[8] + Gx1[64]*Gx2[17] + Gx1[65]*Gx2[26] + Gx1[66]*Gx2[35] + Gx1[67]*Gx2[44] + Gx1[68]*Gx2[53] + Gx1[69]*Gx2[62] + Gx1[70]*Gx2[71] + Gx1[71]*Gx2[80];
Gx3[72] = + Gx1[72]*Gx2[0] + Gx1[73]*Gx2[9] + Gx1[74]*Gx2[18] + Gx1[75]*Gx2[27] + Gx1[76]*Gx2[36] + Gx1[77]*Gx2[45] + Gx1[78]*Gx2[54] + Gx1[79]*Gx2[63] + Gx1[80]*Gx2[72];
Gx3[73] = + Gx1[72]*Gx2[1] + Gx1[73]*Gx2[10] + Gx1[74]*Gx2[19] + Gx1[75]*Gx2[28] + Gx1[76]*Gx2[37] + Gx1[77]*Gx2[46] + Gx1[78]*Gx2[55] + Gx1[79]*Gx2[64] + Gx1[80]*Gx2[73];
Gx3[74] = + Gx1[72]*Gx2[2] + Gx1[73]*Gx2[11] + Gx1[74]*Gx2[20] + Gx1[75]*Gx2[29] + Gx1[76]*Gx2[38] + Gx1[77]*Gx2[47] + Gx1[78]*Gx2[56] + Gx1[79]*Gx2[65] + Gx1[80]*Gx2[74];
Gx3[75] = + Gx1[72]*Gx2[3] + Gx1[73]*Gx2[12] + Gx1[74]*Gx2[21] + Gx1[75]*Gx2[30] + Gx1[76]*Gx2[39] + Gx1[77]*Gx2[48] + Gx1[78]*Gx2[57] + Gx1[79]*Gx2[66] + Gx1[80]*Gx2[75];
Gx3[76] = + Gx1[72]*Gx2[4] + Gx1[73]*Gx2[13] + Gx1[74]*Gx2[22] + Gx1[75]*Gx2[31] + Gx1[76]*Gx2[40] + Gx1[77]*Gx2[49] + Gx1[78]*Gx2[58] + Gx1[79]*Gx2[67] + Gx1[80]*Gx2[76];
Gx3[77] = + Gx1[72]*Gx2[5] + Gx1[73]*Gx2[14] + Gx1[74]*Gx2[23] + Gx1[75]*Gx2[32] + Gx1[76]*Gx2[41] + Gx1[77]*Gx2[50] + Gx1[78]*Gx2[59] + Gx1[79]*Gx2[68] + Gx1[80]*Gx2[77];
Gx3[78] = + Gx1[72]*Gx2[6] + Gx1[73]*Gx2[15] + Gx1[74]*Gx2[24] + Gx1[75]*Gx2[33] + Gx1[76]*Gx2[42] + Gx1[77]*Gx2[51] + Gx1[78]*Gx2[60] + Gx1[79]*Gx2[69] + Gx1[80]*Gx2[78];
Gx3[79] = + Gx1[72]*Gx2[7] + Gx1[73]*Gx2[16] + Gx1[74]*Gx2[25] + Gx1[75]*Gx2[34] + Gx1[76]*Gx2[43] + Gx1[77]*Gx2[52] + Gx1[78]*Gx2[61] + Gx1[79]*Gx2[70] + Gx1[80]*Gx2[79];
Gx3[80] = + Gx1[72]*Gx2[8] + Gx1[73]*Gx2[17] + Gx1[74]*Gx2[26] + Gx1[75]*Gx2[35] + Gx1[76]*Gx2[44] + Gx1[77]*Gx2[53] + Gx1[78]*Gx2[62] + Gx1[79]*Gx2[71] + Gx1[80]*Gx2[80];
}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26];
Gu2[3] = + Gx1[9]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[15] + Gx1[15]*Gu1[18] + Gx1[16]*Gu1[21] + Gx1[17]*Gu1[24];
Gu2[4] = + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[19] + Gx1[16]*Gu1[22] + Gx1[17]*Gu1[25];
Gu2[5] = + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[8] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[23] + Gx1[17]*Gu1[26];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[21]*Gu1[9] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[24];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[16] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[25];
Gu2[8] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[14] + Gx1[23]*Gu1[17] + Gx1[24]*Gu1[20] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[26];
Gu2[9] = + Gx1[27]*Gu1[0] + Gx1[28]*Gu1[3] + Gx1[29]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[31]*Gu1[12] + Gx1[32]*Gu1[15] + Gx1[33]*Gu1[18] + Gx1[34]*Gu1[21] + Gx1[35]*Gu1[24];
Gu2[10] = + Gx1[27]*Gu1[1] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[31]*Gu1[13] + Gx1[32]*Gu1[16] + Gx1[33]*Gu1[19] + Gx1[34]*Gu1[22] + Gx1[35]*Gu1[25];
Gu2[11] = + Gx1[27]*Gu1[2] + Gx1[28]*Gu1[5] + Gx1[29]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[31]*Gu1[14] + Gx1[32]*Gu1[17] + Gx1[33]*Gu1[20] + Gx1[34]*Gu1[23] + Gx1[35]*Gu1[26];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[3] + Gx1[38]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[41]*Gu1[15] + Gx1[42]*Gu1[18] + Gx1[43]*Gu1[21] + Gx1[44]*Gu1[24];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[41]*Gu1[16] + Gx1[42]*Gu1[19] + Gx1[43]*Gu1[22] + Gx1[44]*Gu1[25];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[41]*Gu1[17] + Gx1[42]*Gu1[20] + Gx1[43]*Gu1[23] + Gx1[44]*Gu1[26];
Gu2[15] = + Gx1[45]*Gu1[0] + Gx1[46]*Gu1[3] + Gx1[47]*Gu1[6] + Gx1[48]*Gu1[9] + Gx1[49]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[51]*Gu1[18] + Gx1[52]*Gu1[21] + Gx1[53]*Gu1[24];
Gu2[16] = + Gx1[45]*Gu1[1] + Gx1[46]*Gu1[4] + Gx1[47]*Gu1[7] + Gx1[48]*Gu1[10] + Gx1[49]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[51]*Gu1[19] + Gx1[52]*Gu1[22] + Gx1[53]*Gu1[25];
Gu2[17] = + Gx1[45]*Gu1[2] + Gx1[46]*Gu1[5] + Gx1[47]*Gu1[8] + Gx1[48]*Gu1[11] + Gx1[49]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[51]*Gu1[20] + Gx1[52]*Gu1[23] + Gx1[53]*Gu1[26];
Gu2[18] = + Gx1[54]*Gu1[0] + Gx1[55]*Gu1[3] + Gx1[56]*Gu1[6] + Gx1[57]*Gu1[9] + Gx1[58]*Gu1[12] + Gx1[59]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[61]*Gu1[21] + Gx1[62]*Gu1[24];
Gu2[19] = + Gx1[54]*Gu1[1] + Gx1[55]*Gu1[4] + Gx1[56]*Gu1[7] + Gx1[57]*Gu1[10] + Gx1[58]*Gu1[13] + Gx1[59]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[61]*Gu1[22] + Gx1[62]*Gu1[25];
Gu2[20] = + Gx1[54]*Gu1[2] + Gx1[55]*Gu1[5] + Gx1[56]*Gu1[8] + Gx1[57]*Gu1[11] + Gx1[58]*Gu1[14] + Gx1[59]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[61]*Gu1[23] + Gx1[62]*Gu1[26];
Gu2[21] = + Gx1[63]*Gu1[0] + Gx1[64]*Gu1[3] + Gx1[65]*Gu1[6] + Gx1[66]*Gu1[9] + Gx1[67]*Gu1[12] + Gx1[68]*Gu1[15] + Gx1[69]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[71]*Gu1[24];
Gu2[22] = + Gx1[63]*Gu1[1] + Gx1[64]*Gu1[4] + Gx1[65]*Gu1[7] + Gx1[66]*Gu1[10] + Gx1[67]*Gu1[13] + Gx1[68]*Gu1[16] + Gx1[69]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[71]*Gu1[25];
Gu2[23] = + Gx1[63]*Gu1[2] + Gx1[64]*Gu1[5] + Gx1[65]*Gu1[8] + Gx1[66]*Gu1[11] + Gx1[67]*Gu1[14] + Gx1[68]*Gu1[17] + Gx1[69]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[71]*Gu1[26];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[3] + Gx1[74]*Gu1[6] + Gx1[75]*Gu1[9] + Gx1[76]*Gu1[12] + Gx1[77]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[79]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[7] + Gx1[75]*Gu1[10] + Gx1[76]*Gu1[13] + Gx1[77]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[79]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[11] + Gx1[76]*Gu1[14] + Gx1[77]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[79]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
}

void acado_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
acadoWorkspace.H[(iRow * 90) + (iCol * 3)] += + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 1)] += + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 2)] += + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3)] += + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 1)] += + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 2)] += + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3)] += + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 1)] += + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 2)] += + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
acadoWorkspace.H[(iRow * 90) + (iCol * 3)] = R11[0];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 1)] = R11[1];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 2)] = R11[2];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3)] = R11[3];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 1)] = R11[4];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 2)] = R11[5];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3)] = R11[6];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 1)] = R11[7];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 2)] = R11[8];
}

void acado_zeroBlockH11( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 90) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 1)] = 0.0000000000000000e+00;
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 2)] = 0.0000000000000000e+00;
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 90) + (iCol * 3)] = acadoWorkspace.H[(iCol * 90) + (iRow * 3)];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 90 + 30) + (iRow * 3)];
acadoWorkspace.H[(iRow * 90) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 90 + 60) + (iRow * 3)];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3)] = acadoWorkspace.H[(iCol * 90) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 90 + 30) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 90 + 30) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 90 + 60) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3)] = acadoWorkspace.H[(iCol * 90) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 90 + 30) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 90 + 60) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 90 + 60) + (iRow * 3 + 2)];
}

void acado_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7] + Gx1[8]*dOld[8];
dNew[1] = + Gx1[9]*dOld[0] + Gx1[10]*dOld[1] + Gx1[11]*dOld[2] + Gx1[12]*dOld[3] + Gx1[13]*dOld[4] + Gx1[14]*dOld[5] + Gx1[15]*dOld[6] + Gx1[16]*dOld[7] + Gx1[17]*dOld[8];
dNew[2] = + Gx1[18]*dOld[0] + Gx1[19]*dOld[1] + Gx1[20]*dOld[2] + Gx1[21]*dOld[3] + Gx1[22]*dOld[4] + Gx1[23]*dOld[5] + Gx1[24]*dOld[6] + Gx1[25]*dOld[7] + Gx1[26]*dOld[8];
dNew[3] = + Gx1[27]*dOld[0] + Gx1[28]*dOld[1] + Gx1[29]*dOld[2] + Gx1[30]*dOld[3] + Gx1[31]*dOld[4] + Gx1[32]*dOld[5] + Gx1[33]*dOld[6] + Gx1[34]*dOld[7] + Gx1[35]*dOld[8];
dNew[4] = + Gx1[36]*dOld[0] + Gx1[37]*dOld[1] + Gx1[38]*dOld[2] + Gx1[39]*dOld[3] + Gx1[40]*dOld[4] + Gx1[41]*dOld[5] + Gx1[42]*dOld[6] + Gx1[43]*dOld[7] + Gx1[44]*dOld[8];
dNew[5] = + Gx1[45]*dOld[0] + Gx1[46]*dOld[1] + Gx1[47]*dOld[2] + Gx1[48]*dOld[3] + Gx1[49]*dOld[4] + Gx1[50]*dOld[5] + Gx1[51]*dOld[6] + Gx1[52]*dOld[7] + Gx1[53]*dOld[8];
dNew[6] = + Gx1[54]*dOld[0] + Gx1[55]*dOld[1] + Gx1[56]*dOld[2] + Gx1[57]*dOld[3] + Gx1[58]*dOld[4] + Gx1[59]*dOld[5] + Gx1[60]*dOld[6] + Gx1[61]*dOld[7] + Gx1[62]*dOld[8];
dNew[7] = + Gx1[63]*dOld[0] + Gx1[64]*dOld[1] + Gx1[65]*dOld[2] + Gx1[66]*dOld[3] + Gx1[67]*dOld[4] + Gx1[68]*dOld[5] + Gx1[69]*dOld[6] + Gx1[70]*dOld[7] + Gx1[71]*dOld[8];
dNew[8] = + Gx1[72]*dOld[0] + Gx1[73]*dOld[1] + Gx1[74]*dOld[2] + Gx1[75]*dOld[3] + Gx1[76]*dOld[4] + Gx1[77]*dOld[5] + Gx1[78]*dOld[6] + Gx1[79]*dOld[7] + Gx1[80]*dOld[8];
}

void acado_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + acadoWorkspace.QN1[0]*dOld[0] + acadoWorkspace.QN1[1]*dOld[1] + acadoWorkspace.QN1[2]*dOld[2] + acadoWorkspace.QN1[3]*dOld[3] + acadoWorkspace.QN1[4]*dOld[4] + acadoWorkspace.QN1[5]*dOld[5] + acadoWorkspace.QN1[6]*dOld[6] + acadoWorkspace.QN1[7]*dOld[7] + acadoWorkspace.QN1[8]*dOld[8];
dNew[1] = + acadoWorkspace.QN1[9]*dOld[0] + acadoWorkspace.QN1[10]*dOld[1] + acadoWorkspace.QN1[11]*dOld[2] + acadoWorkspace.QN1[12]*dOld[3] + acadoWorkspace.QN1[13]*dOld[4] + acadoWorkspace.QN1[14]*dOld[5] + acadoWorkspace.QN1[15]*dOld[6] + acadoWorkspace.QN1[16]*dOld[7] + acadoWorkspace.QN1[17]*dOld[8];
dNew[2] = + acadoWorkspace.QN1[18]*dOld[0] + acadoWorkspace.QN1[19]*dOld[1] + acadoWorkspace.QN1[20]*dOld[2] + acadoWorkspace.QN1[21]*dOld[3] + acadoWorkspace.QN1[22]*dOld[4] + acadoWorkspace.QN1[23]*dOld[5] + acadoWorkspace.QN1[24]*dOld[6] + acadoWorkspace.QN1[25]*dOld[7] + acadoWorkspace.QN1[26]*dOld[8];
dNew[3] = + acadoWorkspace.QN1[27]*dOld[0] + acadoWorkspace.QN1[28]*dOld[1] + acadoWorkspace.QN1[29]*dOld[2] + acadoWorkspace.QN1[30]*dOld[3] + acadoWorkspace.QN1[31]*dOld[4] + acadoWorkspace.QN1[32]*dOld[5] + acadoWorkspace.QN1[33]*dOld[6] + acadoWorkspace.QN1[34]*dOld[7] + acadoWorkspace.QN1[35]*dOld[8];
dNew[4] = + acadoWorkspace.QN1[36]*dOld[0] + acadoWorkspace.QN1[37]*dOld[1] + acadoWorkspace.QN1[38]*dOld[2] + acadoWorkspace.QN1[39]*dOld[3] + acadoWorkspace.QN1[40]*dOld[4] + acadoWorkspace.QN1[41]*dOld[5] + acadoWorkspace.QN1[42]*dOld[6] + acadoWorkspace.QN1[43]*dOld[7] + acadoWorkspace.QN1[44]*dOld[8];
dNew[5] = + acadoWorkspace.QN1[45]*dOld[0] + acadoWorkspace.QN1[46]*dOld[1] + acadoWorkspace.QN1[47]*dOld[2] + acadoWorkspace.QN1[48]*dOld[3] + acadoWorkspace.QN1[49]*dOld[4] + acadoWorkspace.QN1[50]*dOld[5] + acadoWorkspace.QN1[51]*dOld[6] + acadoWorkspace.QN1[52]*dOld[7] + acadoWorkspace.QN1[53]*dOld[8];
dNew[6] = + acadoWorkspace.QN1[54]*dOld[0] + acadoWorkspace.QN1[55]*dOld[1] + acadoWorkspace.QN1[56]*dOld[2] + acadoWorkspace.QN1[57]*dOld[3] + acadoWorkspace.QN1[58]*dOld[4] + acadoWorkspace.QN1[59]*dOld[5] + acadoWorkspace.QN1[60]*dOld[6] + acadoWorkspace.QN1[61]*dOld[7] + acadoWorkspace.QN1[62]*dOld[8];
dNew[7] = + acadoWorkspace.QN1[63]*dOld[0] + acadoWorkspace.QN1[64]*dOld[1] + acadoWorkspace.QN1[65]*dOld[2] + acadoWorkspace.QN1[66]*dOld[3] + acadoWorkspace.QN1[67]*dOld[4] + acadoWorkspace.QN1[68]*dOld[5] + acadoWorkspace.QN1[69]*dOld[6] + acadoWorkspace.QN1[70]*dOld[7] + acadoWorkspace.QN1[71]*dOld[8];
dNew[8] = + acadoWorkspace.QN1[72]*dOld[0] + acadoWorkspace.QN1[73]*dOld[1] + acadoWorkspace.QN1[74]*dOld[2] + acadoWorkspace.QN1[75]*dOld[3] + acadoWorkspace.QN1[76]*dOld[4] + acadoWorkspace.QN1[77]*dOld[5] + acadoWorkspace.QN1[78]*dOld[6] + acadoWorkspace.QN1[79]*dOld[7] + acadoWorkspace.QN1[80]*dOld[8];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11];
RDy1[1] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5] + R2[18]*Dy1[6] + R2[19]*Dy1[7] + R2[20]*Dy1[8] + R2[21]*Dy1[9] + R2[22]*Dy1[10] + R2[23]*Dy1[11];
RDy1[2] = + R2[24]*Dy1[0] + R2[25]*Dy1[1] + R2[26]*Dy1[2] + R2[27]*Dy1[3] + R2[28]*Dy1[4] + R2[29]*Dy1[5] + R2[30]*Dy1[6] + R2[31]*Dy1[7] + R2[32]*Dy1[8] + R2[33]*Dy1[9] + R2[34]*Dy1[10] + R2[35]*Dy1[11];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11];
QDy1[1] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5] + Q2[18]*Dy1[6] + Q2[19]*Dy1[7] + Q2[20]*Dy1[8] + Q2[21]*Dy1[9] + Q2[22]*Dy1[10] + Q2[23]*Dy1[11];
QDy1[2] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7] + Q2[32]*Dy1[8] + Q2[33]*Dy1[9] + Q2[34]*Dy1[10] + Q2[35]*Dy1[11];
QDy1[3] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5] + Q2[42]*Dy1[6] + Q2[43]*Dy1[7] + Q2[44]*Dy1[8] + Q2[45]*Dy1[9] + Q2[46]*Dy1[10] + Q2[47]*Dy1[11];
QDy1[4] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5] + Q2[54]*Dy1[6] + Q2[55]*Dy1[7] + Q2[56]*Dy1[8] + Q2[57]*Dy1[9] + Q2[58]*Dy1[10] + Q2[59]*Dy1[11];
QDy1[5] = + Q2[60]*Dy1[0] + Q2[61]*Dy1[1] + Q2[62]*Dy1[2] + Q2[63]*Dy1[3] + Q2[64]*Dy1[4] + Q2[65]*Dy1[5] + Q2[66]*Dy1[6] + Q2[67]*Dy1[7] + Q2[68]*Dy1[8] + Q2[69]*Dy1[9] + Q2[70]*Dy1[10] + Q2[71]*Dy1[11];
QDy1[6] = + Q2[72]*Dy1[0] + Q2[73]*Dy1[1] + Q2[74]*Dy1[2] + Q2[75]*Dy1[3] + Q2[76]*Dy1[4] + Q2[77]*Dy1[5] + Q2[78]*Dy1[6] + Q2[79]*Dy1[7] + Q2[80]*Dy1[8] + Q2[81]*Dy1[9] + Q2[82]*Dy1[10] + Q2[83]*Dy1[11];
QDy1[7] = + Q2[84]*Dy1[0] + Q2[85]*Dy1[1] + Q2[86]*Dy1[2] + Q2[87]*Dy1[3] + Q2[88]*Dy1[4] + Q2[89]*Dy1[5] + Q2[90]*Dy1[6] + Q2[91]*Dy1[7] + Q2[92]*Dy1[8] + Q2[93]*Dy1[9] + Q2[94]*Dy1[10] + Q2[95]*Dy1[11];
QDy1[8] = + Q2[96]*Dy1[0] + Q2[97]*Dy1[1] + Q2[98]*Dy1[2] + Q2[99]*Dy1[3] + Q2[100]*Dy1[4] + Q2[101]*Dy1[5] + Q2[102]*Dy1[6] + Q2[103]*Dy1[7] + Q2[104]*Dy1[8] + Q2[105]*Dy1[9] + Q2[106]*Dy1[10] + Q2[107]*Dy1[11];
}

void acado_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[3]*QDy1[1] + E1[6]*QDy1[2] + E1[9]*QDy1[3] + E1[12]*QDy1[4] + E1[15]*QDy1[5] + E1[18]*QDy1[6] + E1[21]*QDy1[7] + E1[24]*QDy1[8];
U1[1] += + E1[1]*QDy1[0] + E1[4]*QDy1[1] + E1[7]*QDy1[2] + E1[10]*QDy1[3] + E1[13]*QDy1[4] + E1[16]*QDy1[5] + E1[19]*QDy1[6] + E1[22]*QDy1[7] + E1[25]*QDy1[8];
U1[2] += + E1[2]*QDy1[0] + E1[5]*QDy1[1] + E1[8]*QDy1[2] + E1[11]*QDy1[3] + E1[14]*QDy1[4] + E1[17]*QDy1[5] + E1[20]*QDy1[6] + E1[23]*QDy1[7] + E1[26]*QDy1[8];
}

void acado_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[3]*Gx1[9] + E1[6]*Gx1[18] + E1[9]*Gx1[27] + E1[12]*Gx1[36] + E1[15]*Gx1[45] + E1[18]*Gx1[54] + E1[21]*Gx1[63] + E1[24]*Gx1[72];
H101[1] += + E1[0]*Gx1[1] + E1[3]*Gx1[10] + E1[6]*Gx1[19] + E1[9]*Gx1[28] + E1[12]*Gx1[37] + E1[15]*Gx1[46] + E1[18]*Gx1[55] + E1[21]*Gx1[64] + E1[24]*Gx1[73];
H101[2] += + E1[0]*Gx1[2] + E1[3]*Gx1[11] + E1[6]*Gx1[20] + E1[9]*Gx1[29] + E1[12]*Gx1[38] + E1[15]*Gx1[47] + E1[18]*Gx1[56] + E1[21]*Gx1[65] + E1[24]*Gx1[74];
H101[3] += + E1[0]*Gx1[3] + E1[3]*Gx1[12] + E1[6]*Gx1[21] + E1[9]*Gx1[30] + E1[12]*Gx1[39] + E1[15]*Gx1[48] + E1[18]*Gx1[57] + E1[21]*Gx1[66] + E1[24]*Gx1[75];
H101[4] += + E1[0]*Gx1[4] + E1[3]*Gx1[13] + E1[6]*Gx1[22] + E1[9]*Gx1[31] + E1[12]*Gx1[40] + E1[15]*Gx1[49] + E1[18]*Gx1[58] + E1[21]*Gx1[67] + E1[24]*Gx1[76];
H101[5] += + E1[0]*Gx1[5] + E1[3]*Gx1[14] + E1[6]*Gx1[23] + E1[9]*Gx1[32] + E1[12]*Gx1[41] + E1[15]*Gx1[50] + E1[18]*Gx1[59] + E1[21]*Gx1[68] + E1[24]*Gx1[77];
H101[6] += + E1[0]*Gx1[6] + E1[3]*Gx1[15] + E1[6]*Gx1[24] + E1[9]*Gx1[33] + E1[12]*Gx1[42] + E1[15]*Gx1[51] + E1[18]*Gx1[60] + E1[21]*Gx1[69] + E1[24]*Gx1[78];
H101[7] += + E1[0]*Gx1[7] + E1[3]*Gx1[16] + E1[6]*Gx1[25] + E1[9]*Gx1[34] + E1[12]*Gx1[43] + E1[15]*Gx1[52] + E1[18]*Gx1[61] + E1[21]*Gx1[70] + E1[24]*Gx1[79];
H101[8] += + E1[0]*Gx1[8] + E1[3]*Gx1[17] + E1[6]*Gx1[26] + E1[9]*Gx1[35] + E1[12]*Gx1[44] + E1[15]*Gx1[53] + E1[18]*Gx1[62] + E1[21]*Gx1[71] + E1[24]*Gx1[80];
H101[9] += + E1[1]*Gx1[0] + E1[4]*Gx1[9] + E1[7]*Gx1[18] + E1[10]*Gx1[27] + E1[13]*Gx1[36] + E1[16]*Gx1[45] + E1[19]*Gx1[54] + E1[22]*Gx1[63] + E1[25]*Gx1[72];
H101[10] += + E1[1]*Gx1[1] + E1[4]*Gx1[10] + E1[7]*Gx1[19] + E1[10]*Gx1[28] + E1[13]*Gx1[37] + E1[16]*Gx1[46] + E1[19]*Gx1[55] + E1[22]*Gx1[64] + E1[25]*Gx1[73];
H101[11] += + E1[1]*Gx1[2] + E1[4]*Gx1[11] + E1[7]*Gx1[20] + E1[10]*Gx1[29] + E1[13]*Gx1[38] + E1[16]*Gx1[47] + E1[19]*Gx1[56] + E1[22]*Gx1[65] + E1[25]*Gx1[74];
H101[12] += + E1[1]*Gx1[3] + E1[4]*Gx1[12] + E1[7]*Gx1[21] + E1[10]*Gx1[30] + E1[13]*Gx1[39] + E1[16]*Gx1[48] + E1[19]*Gx1[57] + E1[22]*Gx1[66] + E1[25]*Gx1[75];
H101[13] += + E1[1]*Gx1[4] + E1[4]*Gx1[13] + E1[7]*Gx1[22] + E1[10]*Gx1[31] + E1[13]*Gx1[40] + E1[16]*Gx1[49] + E1[19]*Gx1[58] + E1[22]*Gx1[67] + E1[25]*Gx1[76];
H101[14] += + E1[1]*Gx1[5] + E1[4]*Gx1[14] + E1[7]*Gx1[23] + E1[10]*Gx1[32] + E1[13]*Gx1[41] + E1[16]*Gx1[50] + E1[19]*Gx1[59] + E1[22]*Gx1[68] + E1[25]*Gx1[77];
H101[15] += + E1[1]*Gx1[6] + E1[4]*Gx1[15] + E1[7]*Gx1[24] + E1[10]*Gx1[33] + E1[13]*Gx1[42] + E1[16]*Gx1[51] + E1[19]*Gx1[60] + E1[22]*Gx1[69] + E1[25]*Gx1[78];
H101[16] += + E1[1]*Gx1[7] + E1[4]*Gx1[16] + E1[7]*Gx1[25] + E1[10]*Gx1[34] + E1[13]*Gx1[43] + E1[16]*Gx1[52] + E1[19]*Gx1[61] + E1[22]*Gx1[70] + E1[25]*Gx1[79];
H101[17] += + E1[1]*Gx1[8] + E1[4]*Gx1[17] + E1[7]*Gx1[26] + E1[10]*Gx1[35] + E1[13]*Gx1[44] + E1[16]*Gx1[53] + E1[19]*Gx1[62] + E1[22]*Gx1[71] + E1[25]*Gx1[80];
H101[18] += + E1[2]*Gx1[0] + E1[5]*Gx1[9] + E1[8]*Gx1[18] + E1[11]*Gx1[27] + E1[14]*Gx1[36] + E1[17]*Gx1[45] + E1[20]*Gx1[54] + E1[23]*Gx1[63] + E1[26]*Gx1[72];
H101[19] += + E1[2]*Gx1[1] + E1[5]*Gx1[10] + E1[8]*Gx1[19] + E1[11]*Gx1[28] + E1[14]*Gx1[37] + E1[17]*Gx1[46] + E1[20]*Gx1[55] + E1[23]*Gx1[64] + E1[26]*Gx1[73];
H101[20] += + E1[2]*Gx1[2] + E1[5]*Gx1[11] + E1[8]*Gx1[20] + E1[11]*Gx1[29] + E1[14]*Gx1[38] + E1[17]*Gx1[47] + E1[20]*Gx1[56] + E1[23]*Gx1[65] + E1[26]*Gx1[74];
H101[21] += + E1[2]*Gx1[3] + E1[5]*Gx1[12] + E1[8]*Gx1[21] + E1[11]*Gx1[30] + E1[14]*Gx1[39] + E1[17]*Gx1[48] + E1[20]*Gx1[57] + E1[23]*Gx1[66] + E1[26]*Gx1[75];
H101[22] += + E1[2]*Gx1[4] + E1[5]*Gx1[13] + E1[8]*Gx1[22] + E1[11]*Gx1[31] + E1[14]*Gx1[40] + E1[17]*Gx1[49] + E1[20]*Gx1[58] + E1[23]*Gx1[67] + E1[26]*Gx1[76];
H101[23] += + E1[2]*Gx1[5] + E1[5]*Gx1[14] + E1[8]*Gx1[23] + E1[11]*Gx1[32] + E1[14]*Gx1[41] + E1[17]*Gx1[50] + E1[20]*Gx1[59] + E1[23]*Gx1[68] + E1[26]*Gx1[77];
H101[24] += + E1[2]*Gx1[6] + E1[5]*Gx1[15] + E1[8]*Gx1[24] + E1[11]*Gx1[33] + E1[14]*Gx1[42] + E1[17]*Gx1[51] + E1[20]*Gx1[60] + E1[23]*Gx1[69] + E1[26]*Gx1[78];
H101[25] += + E1[2]*Gx1[7] + E1[5]*Gx1[16] + E1[8]*Gx1[25] + E1[11]*Gx1[34] + E1[14]*Gx1[43] + E1[17]*Gx1[52] + E1[20]*Gx1[61] + E1[23]*Gx1[70] + E1[26]*Gx1[79];
H101[26] += + E1[2]*Gx1[8] + E1[5]*Gx1[17] + E1[8]*Gx1[26] + E1[11]*Gx1[35] + E1[14]*Gx1[44] + E1[17]*Gx1[53] + E1[20]*Gx1[62] + E1[23]*Gx1[71] + E1[26]*Gx1[80];
}

void acado_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 27; lCopy++) H101[ lCopy ] = 0; }
}

void acado_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2];
dNew[1] += + E1[3]*U1[0] + E1[4]*U1[1] + E1[5]*U1[2];
dNew[2] += + E1[6]*U1[0] + E1[7]*U1[1] + E1[8]*U1[2];
dNew[3] += + E1[9]*U1[0] + E1[10]*U1[1] + E1[11]*U1[2];
dNew[4] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2];
dNew[5] += + E1[15]*U1[0] + E1[16]*U1[1] + E1[17]*U1[2];
dNew[6] += + E1[18]*U1[0] + E1[19]*U1[1] + E1[20]*U1[2];
dNew[7] += + E1[21]*U1[0] + E1[22]*U1[1] + E1[23]*U1[2];
dNew[8] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2];
}

void acado_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
}

void acado_condensePrep( void )
{
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_moveGxT( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.T );
acado_multGxd( acadoWorkspace.d, &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.d[ 9 ]) );
acado_multGxGx( acadoWorkspace.T, acadoWorkspace.evGx, &(acadoWorkspace.evGx[ 81 ]) );

acado_multGxGu( acadoWorkspace.T, acadoWorkspace.E, &(acadoWorkspace.E[ 27 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.E[ 54 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 9 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.d[ 18 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGx[ 162 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.E[ 81 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.E[ 108 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.E[ 135 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.d[ 27 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGx[ 243 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.E[ 162 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 189 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.E[ 216 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 81 ]), &(acadoWorkspace.E[ 243 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 27 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.d[ 36 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGx[ 324 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.E[ 270 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.E[ 297 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 324 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.E[ 351 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.E[ 378 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.d[ 45 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGx[ 405 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.E[ 405 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.E[ 432 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.E[ 459 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.E[ 486 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.E[ 513 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 135 ]), &(acadoWorkspace.E[ 540 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.d[ 54 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGx[ 486 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.E[ 567 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 594 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.E[ 621 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.E[ 648 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.E[ 675 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.E[ 702 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.E[ 729 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.d[ 63 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGx[ 567 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.E[ 756 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.E[ 783 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.E[ 810 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.E[ 837 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 675 ]), &(acadoWorkspace.E[ 864 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 702 ]), &(acadoWorkspace.E[ 891 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 729 ]), &(acadoWorkspace.E[ 918 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.E[ 945 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 63 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.d[ 72 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.evGx[ 648 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.E[ 972 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.E[ 999 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.E[ 1026 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.E[ 1053 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.E[ 1080 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 891 ]), &(acadoWorkspace.E[ 1107 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 918 ]), &(acadoWorkspace.E[ 1134 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 945 ]), &(acadoWorkspace.E[ 1161 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.E[ 1188 ]) );

acado_moveGxT( &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.T );
acado_multGxd( &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.d[ 81 ]) );
acado_multGxGx( acadoWorkspace.T, &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGx[ 729 ]) );

acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.E[ 1215 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.E[ 1242 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.E[ 1269 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.E[ 1296 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.E[ 1323 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1107 ]), &(acadoWorkspace.E[ 1350 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1134 ]), &(acadoWorkspace.E[ 1377 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1161 ]), &(acadoWorkspace.E[ 1404 ]) );
acado_multGxGu( acadoWorkspace.T, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.E[ 1431 ]) );

acado_moveGuE( &(acadoWorkspace.evGu[ 243 ]), &(acadoWorkspace.E[ 1458 ]) );

acado_multGxGu( &(acadoWorkspace.Q1[ 81 ]), acadoWorkspace.E, acadoWorkspace.QE );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 135 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 459 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 513 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 594 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 621 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.E[ 675 ]), &(acadoWorkspace.QE[ 675 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.E[ 702 ]), &(acadoWorkspace.QE[ 702 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.E[ 729 ]), &(acadoWorkspace.QE[ 729 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QE[ 783 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.QE[ 837 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 891 ]), &(acadoWorkspace.QE[ 891 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 918 ]), &(acadoWorkspace.QE[ 918 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 945 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 999 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QE[ 1026 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.QE[ 1053 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 1107 ]), &(acadoWorkspace.QE[ 1107 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 1134 ]), &(acadoWorkspace.QE[ 1134 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 1161 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_multGxGu( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1215 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1242 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1269 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.QE[ 1323 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1350 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1377 ]), &(acadoWorkspace.QE[ 1377 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1404 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1431 ]), &(acadoWorkspace.QE[ 1431 ]) );
acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1458 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_zeroBlockH10( acadoWorkspace.H10 );
acado_multQETGx( acadoWorkspace.QE, acadoWorkspace.evGx, acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 27 ]), &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 81 ]), &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 162 ]), &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 270 ]), &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 405 ]), &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 567 ]), &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 756 ]), &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 972 ]), &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.H10 );
acado_multQETGx( &(acadoWorkspace.QE[ 1215 ]), &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.H10 );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 189 ]), &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 297 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 594 ]), &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 783 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 999 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1242 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 27 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 135 ]), &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 459 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 621 ]), &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 810 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1026 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1269 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 54 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 81 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 243 ]), &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.H10[ 81 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 351 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 81 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 486 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.H10[ 81 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.H10[ 81 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 837 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.H10[ 81 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1053 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 81 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 81 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 378 ]), &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 513 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 675 ]), &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1323 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 108 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 135 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.H10[ 135 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 702 ]), &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.H10[ 135 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 891 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.H10[ 135 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1107 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 135 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1350 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 135 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 162 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 729 ]), &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.H10[ 162 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 918 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.H10[ 162 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1134 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 162 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1377 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 162 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 189 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 945 ]), &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.H10[ 189 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1161 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 189 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1404 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 189 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 216 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1188 ]), &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.H10[ 216 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1431 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 216 ]) );
acado_zeroBlockH10( &(acadoWorkspace.H10[ 243 ]) );
acado_multQETGx( &(acadoWorkspace.QE[ 1458 ]), &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.H10[ 243 ]) );

acado_setBlockH11_R1( 0, 0, acadoWorkspace.R1 );
acado_setBlockH11( 0, 0, acadoWorkspace.E, acadoWorkspace.QE );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QE[ 27 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QE[ 81 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 162 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 270 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 405 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 567 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 756 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 972 ]) );
acado_setBlockH11( 0, 0, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1215 ]) );

acado_zeroBlockH11( 0, 1 );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 594 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 783 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 999 ]) );
acado_setBlockH11( 0, 1, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1242 ]) );

acado_zeroBlockH11( 0, 2 );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QE[ 135 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 459 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 621 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1026 ]) );
acado_setBlockH11( 0, 2, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1269 ]) );

acado_zeroBlockH11( 0, 3 );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 837 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1053 ]) );
acado_setBlockH11( 0, 3, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1296 ]) );

acado_zeroBlockH11( 0, 4 );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 513 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 675 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 0, 4, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1323 ]) );

acado_zeroBlockH11( 0, 5 );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 702 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 891 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1107 ]) );
acado_setBlockH11( 0, 5, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1350 ]) );

acado_zeroBlockH11( 0, 6 );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QE[ 729 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 918 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1134 ]) );
acado_setBlockH11( 0, 6, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1377 ]) );

acado_zeroBlockH11( 0, 7 );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_setBlockH11( 0, 7, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1404 ]) );

acado_zeroBlockH11( 0, 8 );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 0, 8, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 0, 9 );
acado_setBlockH11( 0, 9, &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 1, 1, &(acadoWorkspace.R1[ 9 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QE[ 54 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 108 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QE[ 189 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 297 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 432 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 594 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QE[ 783 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 999 ]) );
acado_setBlockH11( 1, 1, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1242 ]) );

acado_zeroBlockH11( 1, 2 );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QE[ 135 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 459 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 621 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 1026 ]) );
acado_setBlockH11( 1, 2, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1269 ]) );

acado_zeroBlockH11( 1, 3 );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QE[ 837 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 1053 ]) );
acado_setBlockH11( 1, 3, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1296 ]) );

acado_zeroBlockH11( 1, 4 );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 513 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 675 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 1, 4, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1323 ]) );

acado_zeroBlockH11( 1, 5 );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 702 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QE[ 891 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 1107 ]) );
acado_setBlockH11( 1, 5, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1350 ]) );

acado_zeroBlockH11( 1, 6 );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QE[ 729 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QE[ 918 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 1134 ]) );
acado_setBlockH11( 1, 6, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1377 ]) );

acado_zeroBlockH11( 1, 7 );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_setBlockH11( 1, 7, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1404 ]) );

acado_zeroBlockH11( 1, 8 );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 1, 8, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 1, 9 );
acado_setBlockH11( 1, 9, &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 2, 2, &(acadoWorkspace.R1[ 18 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QE[ 135 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 216 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 324 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 459 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 621 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 810 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QE[ 1026 ]) );
acado_setBlockH11( 2, 2, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1269 ]) );

acado_zeroBlockH11( 2, 3 );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 837 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QE[ 1053 ]) );
acado_setBlockH11( 2, 3, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1296 ]) );

acado_zeroBlockH11( 2, 4 );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 513 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 675 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 2, 4, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1323 ]) );

acado_zeroBlockH11( 2, 5 );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 702 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 891 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QE[ 1107 ]) );
acado_setBlockH11( 2, 5, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1350 ]) );

acado_zeroBlockH11( 2, 6 );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QE[ 729 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 918 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QE[ 1134 ]) );
acado_setBlockH11( 2, 6, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1377 ]) );

acado_zeroBlockH11( 2, 7 );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_setBlockH11( 2, 7, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1404 ]) );

acado_zeroBlockH11( 2, 8 );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 2, 8, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 2, 9 );
acado_setBlockH11( 2, 9, &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 3, 3, &(acadoWorkspace.R1[ 27 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QE[ 243 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QE[ 351 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 486 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 648 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.QE[ 837 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.QE[ 1053 ]) );
acado_setBlockH11( 3, 3, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1296 ]) );

acado_zeroBlockH11( 3, 4 );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 513 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 675 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 3, 4, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1323 ]) );

acado_zeroBlockH11( 3, 5 );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 702 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.QE[ 891 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.QE[ 1107 ]) );
acado_setBlockH11( 3, 5, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1350 ]) );

acado_zeroBlockH11( 3, 6 );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QE[ 729 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.QE[ 918 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.QE[ 1134 ]) );
acado_setBlockH11( 3, 6, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1377 ]) );

acado_zeroBlockH11( 3, 7 );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_setBlockH11( 3, 7, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1404 ]) );

acado_zeroBlockH11( 3, 8 );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 3, 8, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 3, 9 );
acado_setBlockH11( 3, 9, &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 4, 4, &(acadoWorkspace.R1[ 36 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QE[ 378 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 513 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 675 ]), &(acadoWorkspace.QE[ 675 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 864 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1080 ]) );
acado_setBlockH11( 4, 4, &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.QE[ 1323 ]) );

acado_zeroBlockH11( 4, 5 );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 675 ]), &(acadoWorkspace.QE[ 702 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 891 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1107 ]) );
acado_setBlockH11( 4, 5, &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.QE[ 1350 ]) );

acado_zeroBlockH11( 4, 6 );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 675 ]), &(acadoWorkspace.QE[ 729 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 918 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1134 ]) );
acado_setBlockH11( 4, 6, &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.QE[ 1377 ]) );

acado_zeroBlockH11( 4, 7 );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_setBlockH11( 4, 7, &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.QE[ 1404 ]) );

acado_zeroBlockH11( 4, 8 );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 4, 8, &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 4, 9 );
acado_setBlockH11( 4, 9, &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 5, 5, &(acadoWorkspace.R1[ 45 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QE[ 540 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 702 ]), &(acadoWorkspace.QE[ 702 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 891 ]), &(acadoWorkspace.QE[ 891 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1107 ]), &(acadoWorkspace.QE[ 1107 ]) );
acado_setBlockH11( 5, 5, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1350 ]) );

acado_zeroBlockH11( 5, 6 );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 702 ]), &(acadoWorkspace.QE[ 729 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 891 ]), &(acadoWorkspace.QE[ 918 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1107 ]), &(acadoWorkspace.QE[ 1134 ]) );
acado_setBlockH11( 5, 6, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1377 ]) );

acado_zeroBlockH11( 5, 7 );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 891 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1107 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_setBlockH11( 5, 7, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1404 ]) );

acado_zeroBlockH11( 5, 8 );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1107 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 5, 8, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 5, 9 );
acado_setBlockH11( 5, 9, &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 6, 6, &(acadoWorkspace.R1[ 54 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 729 ]), &(acadoWorkspace.QE[ 729 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 918 ]), &(acadoWorkspace.QE[ 918 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1134 ]), &(acadoWorkspace.QE[ 1134 ]) );
acado_setBlockH11( 6, 6, &(acadoWorkspace.E[ 1377 ]), &(acadoWorkspace.QE[ 1377 ]) );

acado_zeroBlockH11( 6, 7 );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 918 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1134 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_setBlockH11( 6, 7, &(acadoWorkspace.E[ 1377 ]), &(acadoWorkspace.QE[ 1404 ]) );

acado_zeroBlockH11( 6, 8 );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1134 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 6, 8, &(acadoWorkspace.E[ 1377 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 6, 9 );
acado_setBlockH11( 6, 9, &(acadoWorkspace.E[ 1377 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 7, 7, &(acadoWorkspace.R1[ 63 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 945 ]), &(acadoWorkspace.QE[ 945 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1161 ]), &(acadoWorkspace.QE[ 1161 ]) );
acado_setBlockH11( 7, 7, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1404 ]) );

acado_zeroBlockH11( 7, 8 );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1161 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 7, 8, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 7, 9 );
acado_setBlockH11( 7, 9, &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 8, 8, &(acadoWorkspace.R1[ 72 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QE[ 1188 ]) );
acado_setBlockH11( 8, 8, &(acadoWorkspace.E[ 1431 ]), &(acadoWorkspace.QE[ 1431 ]) );

acado_zeroBlockH11( 8, 9 );
acado_setBlockH11( 8, 9, &(acadoWorkspace.E[ 1431 ]), &(acadoWorkspace.QE[ 1458 ]) );

acado_setBlockH11_R1( 9, 9, &(acadoWorkspace.R1[ 81 ]) );
acado_setBlockH11( 9, 9, &(acadoWorkspace.E[ 1458 ]), &(acadoWorkspace.QE[ 1458 ]) );


acado_copyHTH( 1, 0 );
acado_copyHTH( 2, 0 );
acado_copyHTH( 2, 1 );
acado_copyHTH( 3, 0 );
acado_copyHTH( 3, 1 );
acado_copyHTH( 3, 2 );
acado_copyHTH( 4, 0 );
acado_copyHTH( 4, 1 );
acado_copyHTH( 4, 2 );
acado_copyHTH( 4, 3 );
acado_copyHTH( 5, 0 );
acado_copyHTH( 5, 1 );
acado_copyHTH( 5, 2 );
acado_copyHTH( 5, 3 );
acado_copyHTH( 5, 4 );
acado_copyHTH( 6, 0 );
acado_copyHTH( 6, 1 );
acado_copyHTH( 6, 2 );
acado_copyHTH( 6, 3 );
acado_copyHTH( 6, 4 );
acado_copyHTH( 6, 5 );
acado_copyHTH( 7, 0 );
acado_copyHTH( 7, 1 );
acado_copyHTH( 7, 2 );
acado_copyHTH( 7, 3 );
acado_copyHTH( 7, 4 );
acado_copyHTH( 7, 5 );
acado_copyHTH( 7, 6 );
acado_copyHTH( 8, 0 );
acado_copyHTH( 8, 1 );
acado_copyHTH( 8, 2 );
acado_copyHTH( 8, 3 );
acado_copyHTH( 8, 4 );
acado_copyHTH( 8, 5 );
acado_copyHTH( 8, 6 );
acado_copyHTH( 8, 7 );
acado_copyHTH( 9, 0 );
acado_copyHTH( 9, 1 );
acado_copyHTH( 9, 2 );
acado_copyHTH( 9, 3 );
acado_copyHTH( 9, 4 );
acado_copyHTH( 9, 5 );
acado_copyHTH( 9, 6 );
acado_copyHTH( 9, 7 );
acado_copyHTH( 9, 8 );

acado_multQ1d( &(acadoWorkspace.Q1[ 81 ]), acadoWorkspace.d, acadoWorkspace.Qd );
acado_multQ1d( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.d[ 9 ]), &(acadoWorkspace.Qd[ 9 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.d[ 18 ]), &(acadoWorkspace.Qd[ 18 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.d[ 27 ]), &(acadoWorkspace.Qd[ 27 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.d[ 36 ]), &(acadoWorkspace.Qd[ 36 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.d[ 45 ]), &(acadoWorkspace.Qd[ 45 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.d[ 54 ]), &(acadoWorkspace.Qd[ 54 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.d[ 63 ]), &(acadoWorkspace.Qd[ 63 ]) );
acado_multQ1d( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.d[ 72 ]), &(acadoWorkspace.Qd[ 72 ]) );
acado_multQN1d( acadoWorkspace.QN1, &(acadoWorkspace.d[ 81 ]), &(acadoWorkspace.Qd[ 81 ]) );

acado_macETSlu( acadoWorkspace.QE, acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 27 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 81 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 162 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 270 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 405 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 567 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 756 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 972 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 1215 ]), acadoWorkspace.g );
acado_macETSlu( &(acadoWorkspace.QE[ 54 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 108 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 189 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 297 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 432 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 594 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 783 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 999 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1242 ]), &(acadoWorkspace.g[ 3 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 135 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 216 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 324 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 459 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 621 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 810 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1026 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1269 ]), &(acadoWorkspace.g[ 6 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 243 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 351 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 486 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 648 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 837 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1053 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1296 ]), &(acadoWorkspace.g[ 9 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 378 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 513 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 675 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 864 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1080 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1323 ]), &(acadoWorkspace.g[ 12 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 540 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 702 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 891 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1107 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1350 ]), &(acadoWorkspace.g[ 15 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 729 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 918 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1134 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1377 ]), &(acadoWorkspace.g[ 18 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 945 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1161 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1404 ]), &(acadoWorkspace.g[ 21 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1188 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1431 ]), &(acadoWorkspace.g[ 24 ]) );
acado_macETSlu( &(acadoWorkspace.QE[ 1458 ]), &(acadoWorkspace.g[ 27 ]) );
acadoWorkspace.lb[0] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-1.0000000000000000e+01 - acadoVariables.u[29];
acadoWorkspace.ub[0] = (real_t)1.0000000000000000e+01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)1.0000000000000000e+01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)1.0000000000000000e+01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)1.0000000000000000e+01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)1.0000000000000000e+01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)1.0000000000000000e+01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)1.0000000000000000e+01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)1.0000000000000000e+01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)1.0000000000000000e+01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)1.0000000000000000e+01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)1.0000000000000000e+01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)1.0000000000000000e+01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)1.0000000000000000e+01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)1.0000000000000000e+01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)1.0000000000000000e+01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)1.0000000000000000e+01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)1.0000000000000000e+01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)1.0000000000000000e+01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)1.0000000000000000e+01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)1.0000000000000000e+01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)1.0000000000000000e+01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)1.0000000000000000e+01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)1.0000000000000000e+01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)1.0000000000000000e+01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)1.0000000000000000e+01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)1.0000000000000000e+01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)1.0000000000000000e+01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)1.0000000000000000e+01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)1.0000000000000000e+01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)1.0000000000000000e+01 - acadoVariables.u[29];

}

void acado_condenseFdb( void )
{
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];

acadoWorkspace.Dy[0] -= acadoVariables.y[0];
acadoWorkspace.Dy[1] -= acadoVariables.y[1];
acadoWorkspace.Dy[2] -= acadoVariables.y[2];
acadoWorkspace.Dy[3] -= acadoVariables.y[3];
acadoWorkspace.Dy[4] -= acadoVariables.y[4];
acadoWorkspace.Dy[5] -= acadoVariables.y[5];
acadoWorkspace.Dy[6] -= acadoVariables.y[6];
acadoWorkspace.Dy[7] -= acadoVariables.y[7];
acadoWorkspace.Dy[8] -= acadoVariables.y[8];
acadoWorkspace.Dy[9] -= acadoVariables.y[9];
acadoWorkspace.Dy[10] -= acadoVariables.y[10];
acadoWorkspace.Dy[11] -= acadoVariables.y[11];
acadoWorkspace.Dy[12] -= acadoVariables.y[12];
acadoWorkspace.Dy[13] -= acadoVariables.y[13];
acadoWorkspace.Dy[14] -= acadoVariables.y[14];
acadoWorkspace.Dy[15] -= acadoVariables.y[15];
acadoWorkspace.Dy[16] -= acadoVariables.y[16];
acadoWorkspace.Dy[17] -= acadoVariables.y[17];
acadoWorkspace.Dy[18] -= acadoVariables.y[18];
acadoWorkspace.Dy[19] -= acadoVariables.y[19];
acadoWorkspace.Dy[20] -= acadoVariables.y[20];
acadoWorkspace.Dy[21] -= acadoVariables.y[21];
acadoWorkspace.Dy[22] -= acadoVariables.y[22];
acadoWorkspace.Dy[23] -= acadoVariables.y[23];
acadoWorkspace.Dy[24] -= acadoVariables.y[24];
acadoWorkspace.Dy[25] -= acadoVariables.y[25];
acadoWorkspace.Dy[26] -= acadoVariables.y[26];
acadoWorkspace.Dy[27] -= acadoVariables.y[27];
acadoWorkspace.Dy[28] -= acadoVariables.y[28];
acadoWorkspace.Dy[29] -= acadoVariables.y[29];
acadoWorkspace.Dy[30] -= acadoVariables.y[30];
acadoWorkspace.Dy[31] -= acadoVariables.y[31];
acadoWorkspace.Dy[32] -= acadoVariables.y[32];
acadoWorkspace.Dy[33] -= acadoVariables.y[33];
acadoWorkspace.Dy[34] -= acadoVariables.y[34];
acadoWorkspace.Dy[35] -= acadoVariables.y[35];
acadoWorkspace.Dy[36] -= acadoVariables.y[36];
acadoWorkspace.Dy[37] -= acadoVariables.y[37];
acadoWorkspace.Dy[38] -= acadoVariables.y[38];
acadoWorkspace.Dy[39] -= acadoVariables.y[39];
acadoWorkspace.Dy[40] -= acadoVariables.y[40];
acadoWorkspace.Dy[41] -= acadoVariables.y[41];
acadoWorkspace.Dy[42] -= acadoVariables.y[42];
acadoWorkspace.Dy[43] -= acadoVariables.y[43];
acadoWorkspace.Dy[44] -= acadoVariables.y[44];
acadoWorkspace.Dy[45] -= acadoVariables.y[45];
acadoWorkspace.Dy[46] -= acadoVariables.y[46];
acadoWorkspace.Dy[47] -= acadoVariables.y[47];
acadoWorkspace.Dy[48] -= acadoVariables.y[48];
acadoWorkspace.Dy[49] -= acadoVariables.y[49];
acadoWorkspace.Dy[50] -= acadoVariables.y[50];
acadoWorkspace.Dy[51] -= acadoVariables.y[51];
acadoWorkspace.Dy[52] -= acadoVariables.y[52];
acadoWorkspace.Dy[53] -= acadoVariables.y[53];
acadoWorkspace.Dy[54] -= acadoVariables.y[54];
acadoWorkspace.Dy[55] -= acadoVariables.y[55];
acadoWorkspace.Dy[56] -= acadoVariables.y[56];
acadoWorkspace.Dy[57] -= acadoVariables.y[57];
acadoWorkspace.Dy[58] -= acadoVariables.y[58];
acadoWorkspace.Dy[59] -= acadoVariables.y[59];
acadoWorkspace.Dy[60] -= acadoVariables.y[60];
acadoWorkspace.Dy[61] -= acadoVariables.y[61];
acadoWorkspace.Dy[62] -= acadoVariables.y[62];
acadoWorkspace.Dy[63] -= acadoVariables.y[63];
acadoWorkspace.Dy[64] -= acadoVariables.y[64];
acadoWorkspace.Dy[65] -= acadoVariables.y[65];
acadoWorkspace.Dy[66] -= acadoVariables.y[66];
acadoWorkspace.Dy[67] -= acadoVariables.y[67];
acadoWorkspace.Dy[68] -= acadoVariables.y[68];
acadoWorkspace.Dy[69] -= acadoVariables.y[69];
acadoWorkspace.Dy[70] -= acadoVariables.y[70];
acadoWorkspace.Dy[71] -= acadoVariables.y[71];
acadoWorkspace.Dy[72] -= acadoVariables.y[72];
acadoWorkspace.Dy[73] -= acadoVariables.y[73];
acadoWorkspace.Dy[74] -= acadoVariables.y[74];
acadoWorkspace.Dy[75] -= acadoVariables.y[75];
acadoWorkspace.Dy[76] -= acadoVariables.y[76];
acadoWorkspace.Dy[77] -= acadoVariables.y[77];
acadoWorkspace.Dy[78] -= acadoVariables.y[78];
acadoWorkspace.Dy[79] -= acadoVariables.y[79];
acadoWorkspace.Dy[80] -= acadoVariables.y[80];
acadoWorkspace.Dy[81] -= acadoVariables.y[81];
acadoWorkspace.Dy[82] -= acadoVariables.y[82];
acadoWorkspace.Dy[83] -= acadoVariables.y[83];
acadoWorkspace.Dy[84] -= acadoVariables.y[84];
acadoWorkspace.Dy[85] -= acadoVariables.y[85];
acadoWorkspace.Dy[86] -= acadoVariables.y[86];
acadoWorkspace.Dy[87] -= acadoVariables.y[87];
acadoWorkspace.Dy[88] -= acadoVariables.y[88];
acadoWorkspace.Dy[89] -= acadoVariables.y[89];
acadoWorkspace.Dy[90] -= acadoVariables.y[90];
acadoWorkspace.Dy[91] -= acadoVariables.y[91];
acadoWorkspace.Dy[92] -= acadoVariables.y[92];
acadoWorkspace.Dy[93] -= acadoVariables.y[93];
acadoWorkspace.Dy[94] -= acadoVariables.y[94];
acadoWorkspace.Dy[95] -= acadoVariables.y[95];
acadoWorkspace.Dy[96] -= acadoVariables.y[96];
acadoWorkspace.Dy[97] -= acadoVariables.y[97];
acadoWorkspace.Dy[98] -= acadoVariables.y[98];
acadoWorkspace.Dy[99] -= acadoVariables.y[99];
acadoWorkspace.Dy[100] -= acadoVariables.y[100];
acadoWorkspace.Dy[101] -= acadoVariables.y[101];
acadoWorkspace.Dy[102] -= acadoVariables.y[102];
acadoWorkspace.Dy[103] -= acadoVariables.y[103];
acadoWorkspace.Dy[104] -= acadoVariables.y[104];
acadoWorkspace.Dy[105] -= acadoVariables.y[105];
acadoWorkspace.Dy[106] -= acadoVariables.y[106];
acadoWorkspace.Dy[107] -= acadoVariables.y[107];
acadoWorkspace.Dy[108] -= acadoVariables.y[108];
acadoWorkspace.Dy[109] -= acadoVariables.y[109];
acadoWorkspace.Dy[110] -= acadoVariables.y[110];
acadoWorkspace.Dy[111] -= acadoVariables.y[111];
acadoWorkspace.Dy[112] -= acadoVariables.y[112];
acadoWorkspace.Dy[113] -= acadoVariables.y[113];
acadoWorkspace.Dy[114] -= acadoVariables.y[114];
acadoWorkspace.Dy[115] -= acadoVariables.y[115];
acadoWorkspace.Dy[116] -= acadoVariables.y[116];
acadoWorkspace.Dy[117] -= acadoVariables.y[117];
acadoWorkspace.Dy[118] -= acadoVariables.y[118];
acadoWorkspace.Dy[119] -= acadoVariables.y[119];
acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];
acadoWorkspace.DyN[8] -= acadoVariables.yN[8];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 108 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 324 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 27 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 216 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 324 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 648 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 756 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 972 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 81 ]) );

acadoWorkspace.QDy[90] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[91] = + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[92] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[93] = + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[94] = + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[95] = + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[96] = + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[97] = + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[64]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[65]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[66]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[67]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[68]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[69]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[70]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[71]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[98] = + acadoWorkspace.QN2[72]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[73]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[74]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[75]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[76]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[77]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[78]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[79]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[80]*acadoWorkspace.DyN[8];

acadoWorkspace.QDy[9] += acadoWorkspace.Qd[0];
acadoWorkspace.QDy[10] += acadoWorkspace.Qd[1];
acadoWorkspace.QDy[11] += acadoWorkspace.Qd[2];
acadoWorkspace.QDy[12] += acadoWorkspace.Qd[3];
acadoWorkspace.QDy[13] += acadoWorkspace.Qd[4];
acadoWorkspace.QDy[14] += acadoWorkspace.Qd[5];
acadoWorkspace.QDy[15] += acadoWorkspace.Qd[6];
acadoWorkspace.QDy[16] += acadoWorkspace.Qd[7];
acadoWorkspace.QDy[17] += acadoWorkspace.Qd[8];
acadoWorkspace.QDy[18] += acadoWorkspace.Qd[9];
acadoWorkspace.QDy[19] += acadoWorkspace.Qd[10];
acadoWorkspace.QDy[20] += acadoWorkspace.Qd[11];
acadoWorkspace.QDy[21] += acadoWorkspace.Qd[12];
acadoWorkspace.QDy[22] += acadoWorkspace.Qd[13];
acadoWorkspace.QDy[23] += acadoWorkspace.Qd[14];
acadoWorkspace.QDy[24] += acadoWorkspace.Qd[15];
acadoWorkspace.QDy[25] += acadoWorkspace.Qd[16];
acadoWorkspace.QDy[26] += acadoWorkspace.Qd[17];
acadoWorkspace.QDy[27] += acadoWorkspace.Qd[18];
acadoWorkspace.QDy[28] += acadoWorkspace.Qd[19];
acadoWorkspace.QDy[29] += acadoWorkspace.Qd[20];
acadoWorkspace.QDy[30] += acadoWorkspace.Qd[21];
acadoWorkspace.QDy[31] += acadoWorkspace.Qd[22];
acadoWorkspace.QDy[32] += acadoWorkspace.Qd[23];
acadoWorkspace.QDy[33] += acadoWorkspace.Qd[24];
acadoWorkspace.QDy[34] += acadoWorkspace.Qd[25];
acadoWorkspace.QDy[35] += acadoWorkspace.Qd[26];
acadoWorkspace.QDy[36] += acadoWorkspace.Qd[27];
acadoWorkspace.QDy[37] += acadoWorkspace.Qd[28];
acadoWorkspace.QDy[38] += acadoWorkspace.Qd[29];
acadoWorkspace.QDy[39] += acadoWorkspace.Qd[30];
acadoWorkspace.QDy[40] += acadoWorkspace.Qd[31];
acadoWorkspace.QDy[41] += acadoWorkspace.Qd[32];
acadoWorkspace.QDy[42] += acadoWorkspace.Qd[33];
acadoWorkspace.QDy[43] += acadoWorkspace.Qd[34];
acadoWorkspace.QDy[44] += acadoWorkspace.Qd[35];
acadoWorkspace.QDy[45] += acadoWorkspace.Qd[36];
acadoWorkspace.QDy[46] += acadoWorkspace.Qd[37];
acadoWorkspace.QDy[47] += acadoWorkspace.Qd[38];
acadoWorkspace.QDy[48] += acadoWorkspace.Qd[39];
acadoWorkspace.QDy[49] += acadoWorkspace.Qd[40];
acadoWorkspace.QDy[50] += acadoWorkspace.Qd[41];
acadoWorkspace.QDy[51] += acadoWorkspace.Qd[42];
acadoWorkspace.QDy[52] += acadoWorkspace.Qd[43];
acadoWorkspace.QDy[53] += acadoWorkspace.Qd[44];
acadoWorkspace.QDy[54] += acadoWorkspace.Qd[45];
acadoWorkspace.QDy[55] += acadoWorkspace.Qd[46];
acadoWorkspace.QDy[56] += acadoWorkspace.Qd[47];
acadoWorkspace.QDy[57] += acadoWorkspace.Qd[48];
acadoWorkspace.QDy[58] += acadoWorkspace.Qd[49];
acadoWorkspace.QDy[59] += acadoWorkspace.Qd[50];
acadoWorkspace.QDy[60] += acadoWorkspace.Qd[51];
acadoWorkspace.QDy[61] += acadoWorkspace.Qd[52];
acadoWorkspace.QDy[62] += acadoWorkspace.Qd[53];
acadoWorkspace.QDy[63] += acadoWorkspace.Qd[54];
acadoWorkspace.QDy[64] += acadoWorkspace.Qd[55];
acadoWorkspace.QDy[65] += acadoWorkspace.Qd[56];
acadoWorkspace.QDy[66] += acadoWorkspace.Qd[57];
acadoWorkspace.QDy[67] += acadoWorkspace.Qd[58];
acadoWorkspace.QDy[68] += acadoWorkspace.Qd[59];
acadoWorkspace.QDy[69] += acadoWorkspace.Qd[60];
acadoWorkspace.QDy[70] += acadoWorkspace.Qd[61];
acadoWorkspace.QDy[71] += acadoWorkspace.Qd[62];
acadoWorkspace.QDy[72] += acadoWorkspace.Qd[63];
acadoWorkspace.QDy[73] += acadoWorkspace.Qd[64];
acadoWorkspace.QDy[74] += acadoWorkspace.Qd[65];
acadoWorkspace.QDy[75] += acadoWorkspace.Qd[66];
acadoWorkspace.QDy[76] += acadoWorkspace.Qd[67];
acadoWorkspace.QDy[77] += acadoWorkspace.Qd[68];
acadoWorkspace.QDy[78] += acadoWorkspace.Qd[69];
acadoWorkspace.QDy[79] += acadoWorkspace.Qd[70];
acadoWorkspace.QDy[80] += acadoWorkspace.Qd[71];
acadoWorkspace.QDy[81] += acadoWorkspace.Qd[72];
acadoWorkspace.QDy[82] += acadoWorkspace.Qd[73];
acadoWorkspace.QDy[83] += acadoWorkspace.Qd[74];
acadoWorkspace.QDy[84] += acadoWorkspace.Qd[75];
acadoWorkspace.QDy[85] += acadoWorkspace.Qd[76];
acadoWorkspace.QDy[86] += acadoWorkspace.Qd[77];
acadoWorkspace.QDy[87] += acadoWorkspace.Qd[78];
acadoWorkspace.QDy[88] += acadoWorkspace.Qd[79];
acadoWorkspace.QDy[89] += acadoWorkspace.Qd[80];
acadoWorkspace.QDy[90] += acadoWorkspace.Qd[81];
acadoWorkspace.QDy[91] += acadoWorkspace.Qd[82];
acadoWorkspace.QDy[92] += acadoWorkspace.Qd[83];
acadoWorkspace.QDy[93] += acadoWorkspace.Qd[84];
acadoWorkspace.QDy[94] += acadoWorkspace.Qd[85];
acadoWorkspace.QDy[95] += acadoWorkspace.Qd[86];
acadoWorkspace.QDy[96] += acadoWorkspace.Qd[87];
acadoWorkspace.QDy[97] += acadoWorkspace.Qd[88];
acadoWorkspace.QDy[98] += acadoWorkspace.Qd[89];

acado_multEQDy( acadoWorkspace.E, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 27 ]), &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 81 ]), &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 162 ]), &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 270 ]), &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 405 ]), &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 567 ]), &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 972 ]), &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 1215 ]), &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.g );
acado_multEQDy( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.QDy[ 18 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.QDy[ 27 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.QDy[ 36 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.QDy[ 45 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 675 ]), &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.QDy[ 54 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 702 ]), &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 891 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1107 ]), &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 729 ]), &(acadoWorkspace.QDy[ 63 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 918 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1134 ]), &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1377 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 945 ]), &(acadoWorkspace.QDy[ 72 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1161 ]), &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.QDy[ 81 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1431 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multEQDy( &(acadoWorkspace.E[ 1458 ]), &(acadoWorkspace.QDy[ 90 ]), &(acadoWorkspace.g[ 27 ]) );

acadoWorkspace.g[0] += + acadoWorkspace.H10[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[8]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[1] += + acadoWorkspace.H10[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[11]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[12]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[13]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[14]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[15]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[16]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[17]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[2] += + acadoWorkspace.H10[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[23]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[24]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[25]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[26]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[3] += + acadoWorkspace.H10[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[29]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[30]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[31]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[32]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[33]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[34]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[35]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[4] += + acadoWorkspace.H10[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[41]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[42]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[43]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[44]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[5] += + acadoWorkspace.H10[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[49]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[50]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[51]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[52]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[53]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[6] += + acadoWorkspace.H10[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[59]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[60]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[61]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[62]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[7] += + acadoWorkspace.H10[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[65]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[66]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[67]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[68]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[69]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[70]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[71]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[8] += + acadoWorkspace.H10[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[79]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[80]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[9] += + acadoWorkspace.H10[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[83]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[84]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[85]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[86]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[87]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[88]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[89]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[10] += + acadoWorkspace.H10[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[95]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[96]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[97]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[98]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[11] += + acadoWorkspace.H10[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[101]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[102]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[103]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[104]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[105]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[106]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[107]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[12] += + acadoWorkspace.H10[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[113]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[114]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[115]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[116]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[13] += + acadoWorkspace.H10[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[119]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[120]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[121]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[122]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[123]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[124]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[125]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[14] += + acadoWorkspace.H10[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[131]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[132]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[133]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[134]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[15] += + acadoWorkspace.H10[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[139]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[140]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[141]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[142]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[143]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[16] += + acadoWorkspace.H10[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[150]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[151]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[152]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[17] += + acadoWorkspace.H10[153]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[154]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[155]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[156]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[157]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[158]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[159]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[160]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[161]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[18] += + acadoWorkspace.H10[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[165]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[166]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[167]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[168]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[169]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[170]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[19] += + acadoWorkspace.H10[171]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[172]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[173]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[174]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[175]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[176]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[177]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[178]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[179]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[20] += + acadoWorkspace.H10[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[188]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[21] += + acadoWorkspace.H10[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[191]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[192]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[193]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[194]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[195]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[196]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[197]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[22] += + acadoWorkspace.H10[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[201]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[202]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[203]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[204]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[205]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[206]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[23] += + acadoWorkspace.H10[207]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[208]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[209]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[210]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[211]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[212]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[213]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[214]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[215]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[24] += + acadoWorkspace.H10[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[222]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[223]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[224]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[25] += + acadoWorkspace.H10[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[229]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[230]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[231]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[232]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[233]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[26] += + acadoWorkspace.H10[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[236]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[237]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[238]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[239]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[240]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[241]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[242]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[27] += + acadoWorkspace.H10[243]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[244]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[245]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[246]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[247]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[248]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[249]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[250]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[251]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[28] += + acadoWorkspace.H10[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[258]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[259]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[260]*acadoWorkspace.Dx0[8];
acadoWorkspace.g[29] += + acadoWorkspace.H10[261]*acadoWorkspace.Dx0[0] + acadoWorkspace.H10[262]*acadoWorkspace.Dx0[1] + acadoWorkspace.H10[263]*acadoWorkspace.Dx0[2] + acadoWorkspace.H10[264]*acadoWorkspace.Dx0[3] + acadoWorkspace.H10[265]*acadoWorkspace.Dx0[4] + acadoWorkspace.H10[266]*acadoWorkspace.Dx0[5] + acadoWorkspace.H10[267]*acadoWorkspace.Dx0[6] + acadoWorkspace.H10[268]*acadoWorkspace.Dx0[7] + acadoWorkspace.H10[269]*acadoWorkspace.Dx0[8];

}

void acado_expand( void )
{
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];

acadoVariables.x[0] += acadoWorkspace.Dx0[0];
acadoVariables.x[1] += acadoWorkspace.Dx0[1];
acadoVariables.x[2] += acadoWorkspace.Dx0[2];
acadoVariables.x[3] += acadoWorkspace.Dx0[3];
acadoVariables.x[4] += acadoWorkspace.Dx0[4];
acadoVariables.x[5] += acadoWorkspace.Dx0[5];
acadoVariables.x[6] += acadoWorkspace.Dx0[6];
acadoVariables.x[7] += acadoWorkspace.Dx0[7];
acadoVariables.x[8] += acadoWorkspace.Dx0[8];

acadoVariables.x[9] += + acadoWorkspace.evGx[0]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[1]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[2]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[3]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[4]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[5]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[6]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[7]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[8]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[0];
acadoVariables.x[10] += + acadoWorkspace.evGx[9]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[10]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[11]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[12]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[13]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[14]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[15]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[16]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[17]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[1];
acadoVariables.x[11] += + acadoWorkspace.evGx[18]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[19]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[20]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[21]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[22]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[23]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[24]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[25]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[26]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[2];
acadoVariables.x[12] += + acadoWorkspace.evGx[27]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[28]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[29]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[30]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[31]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[32]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[33]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[34]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[35]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[3];
acadoVariables.x[13] += + acadoWorkspace.evGx[36]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[37]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[38]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[39]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[40]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[41]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[42]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[43]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[44]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[4];
acadoVariables.x[14] += + acadoWorkspace.evGx[45]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[46]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[47]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[48]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[49]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[50]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[51]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[52]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[53]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[5];
acadoVariables.x[15] += + acadoWorkspace.evGx[54]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[55]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[56]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[57]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[58]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[59]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[60]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[61]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[62]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[6];
acadoVariables.x[16] += + acadoWorkspace.evGx[63]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[64]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[65]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[66]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[67]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[68]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[69]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[70]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[71]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[7];
acadoVariables.x[17] += + acadoWorkspace.evGx[72]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[73]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[74]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[75]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[76]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[77]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[78]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[79]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[80]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[8];
acadoVariables.x[18] += + acadoWorkspace.evGx[81]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[82]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[83]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[84]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[85]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[86]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[87]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[88]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[89]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[9];
acadoVariables.x[19] += + acadoWorkspace.evGx[90]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[91]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[92]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[93]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[94]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[95]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[96]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[97]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[98]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[10];
acadoVariables.x[20] += + acadoWorkspace.evGx[99]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[100]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[101]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[102]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[103]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[104]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[105]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[106]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[107]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[11];
acadoVariables.x[21] += + acadoWorkspace.evGx[108]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[109]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[110]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[111]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[112]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[113]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[114]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[115]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[116]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[12];
acadoVariables.x[22] += + acadoWorkspace.evGx[117]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[118]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[119]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[120]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[121]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[122]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[123]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[124]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[125]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[13];
acadoVariables.x[23] += + acadoWorkspace.evGx[126]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[127]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[128]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[129]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[130]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[131]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[132]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[133]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[134]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[14];
acadoVariables.x[24] += + acadoWorkspace.evGx[135]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[136]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[137]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[138]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[139]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[140]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[141]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[142]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[143]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[15];
acadoVariables.x[25] += + acadoWorkspace.evGx[144]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[145]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[146]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[147]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[148]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[149]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[150]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[151]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[152]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[16];
acadoVariables.x[26] += + acadoWorkspace.evGx[153]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[154]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[155]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[156]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[157]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[158]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[159]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[160]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[161]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[17];
acadoVariables.x[27] += + acadoWorkspace.evGx[162]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[163]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[164]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[165]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[166]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[167]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[168]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[169]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[170]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[18];
acadoVariables.x[28] += + acadoWorkspace.evGx[171]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[172]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[173]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[174]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[175]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[176]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[177]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[178]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[179]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[19];
acadoVariables.x[29] += + acadoWorkspace.evGx[180]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[181]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[182]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[183]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[184]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[185]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[186]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[187]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[188]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[20];
acadoVariables.x[30] += + acadoWorkspace.evGx[189]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[190]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[191]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[192]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[193]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[194]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[195]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[196]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[197]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[21];
acadoVariables.x[31] += + acadoWorkspace.evGx[198]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[199]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[200]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[201]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[202]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[203]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[204]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[205]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[206]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[22];
acadoVariables.x[32] += + acadoWorkspace.evGx[207]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[208]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[209]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[210]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[211]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[212]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[213]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[214]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[215]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[23];
acadoVariables.x[33] += + acadoWorkspace.evGx[216]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[217]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[218]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[219]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[220]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[221]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[222]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[223]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[224]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[24];
acadoVariables.x[34] += + acadoWorkspace.evGx[225]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[226]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[227]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[228]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[229]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[230]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[231]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[232]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[233]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[25];
acadoVariables.x[35] += + acadoWorkspace.evGx[234]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[235]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[236]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[237]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[238]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[239]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[240]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[241]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[242]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[26];
acadoVariables.x[36] += + acadoWorkspace.evGx[243]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[244]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[245]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[246]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[247]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[248]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[249]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[250]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[251]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[27];
acadoVariables.x[37] += + acadoWorkspace.evGx[252]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[253]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[254]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[255]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[256]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[257]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[258]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[259]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[260]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[28];
acadoVariables.x[38] += + acadoWorkspace.evGx[261]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[262]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[263]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[264]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[265]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[266]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[267]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[268]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[269]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[29];
acadoVariables.x[39] += + acadoWorkspace.evGx[270]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[271]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[272]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[273]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[274]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[275]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[276]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[277]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[278]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[30];
acadoVariables.x[40] += + acadoWorkspace.evGx[279]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[280]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[281]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[282]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[283]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[284]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[285]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[286]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[287]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[31];
acadoVariables.x[41] += + acadoWorkspace.evGx[288]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[289]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[290]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[291]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[292]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[293]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[294]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[295]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[296]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[32];
acadoVariables.x[42] += + acadoWorkspace.evGx[297]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[298]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[299]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[300]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[301]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[302]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[303]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[304]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[305]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[33];
acadoVariables.x[43] += + acadoWorkspace.evGx[306]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[307]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[308]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[309]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[310]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[311]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[312]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[313]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[314]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[34];
acadoVariables.x[44] += + acadoWorkspace.evGx[315]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[316]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[317]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[318]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[319]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[320]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[321]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[322]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[323]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[35];
acadoVariables.x[45] += + acadoWorkspace.evGx[324]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[325]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[326]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[327]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[328]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[329]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[330]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[331]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[332]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[36];
acadoVariables.x[46] += + acadoWorkspace.evGx[333]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[334]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[335]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[336]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[337]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[338]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[339]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[340]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[341]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[37];
acadoVariables.x[47] += + acadoWorkspace.evGx[342]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[343]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[344]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[345]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[346]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[347]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[348]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[349]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[350]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[38];
acadoVariables.x[48] += + acadoWorkspace.evGx[351]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[352]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[353]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[354]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[355]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[356]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[357]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[358]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[359]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[39];
acadoVariables.x[49] += + acadoWorkspace.evGx[360]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[361]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[362]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[363]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[364]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[365]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[366]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[367]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[368]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[40];
acadoVariables.x[50] += + acadoWorkspace.evGx[369]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[370]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[371]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[372]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[373]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[374]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[375]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[376]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[377]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[41];
acadoVariables.x[51] += + acadoWorkspace.evGx[378]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[379]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[380]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[381]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[382]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[383]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[384]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[385]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[386]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[42];
acadoVariables.x[52] += + acadoWorkspace.evGx[387]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[388]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[389]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[390]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[391]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[392]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[393]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[394]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[395]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[43];
acadoVariables.x[53] += + acadoWorkspace.evGx[396]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[397]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[398]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[399]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[400]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[401]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[402]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[403]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[404]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[44];
acadoVariables.x[54] += + acadoWorkspace.evGx[405]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[406]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[407]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[408]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[409]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[410]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[411]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[412]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[413]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[45];
acadoVariables.x[55] += + acadoWorkspace.evGx[414]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[415]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[416]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[417]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[418]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[419]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[420]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[421]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[422]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[46];
acadoVariables.x[56] += + acadoWorkspace.evGx[423]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[424]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[425]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[426]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[427]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[428]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[429]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[430]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[431]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[47];
acadoVariables.x[57] += + acadoWorkspace.evGx[432]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[433]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[434]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[435]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[436]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[437]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[438]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[439]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[440]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[48];
acadoVariables.x[58] += + acadoWorkspace.evGx[441]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[442]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[443]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[444]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[445]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[446]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[447]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[448]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[449]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[49];
acadoVariables.x[59] += + acadoWorkspace.evGx[450]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[451]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[452]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[453]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[454]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[455]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[456]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[457]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[458]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[50];
acadoVariables.x[60] += + acadoWorkspace.evGx[459]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[460]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[461]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[462]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[463]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[464]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[465]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[466]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[467]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[51];
acadoVariables.x[61] += + acadoWorkspace.evGx[468]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[469]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[470]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[471]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[472]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[473]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[474]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[475]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[476]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[52];
acadoVariables.x[62] += + acadoWorkspace.evGx[477]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[478]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[479]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[480]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[481]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[482]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[483]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[484]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[485]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[53];
acadoVariables.x[63] += + acadoWorkspace.evGx[486]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[487]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[488]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[489]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[490]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[491]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[492]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[493]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[494]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[54];
acadoVariables.x[64] += + acadoWorkspace.evGx[495]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[496]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[497]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[498]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[499]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[500]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[501]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[502]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[503]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[55];
acadoVariables.x[65] += + acadoWorkspace.evGx[504]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[505]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[506]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[507]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[508]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[509]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[510]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[511]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[512]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[56];
acadoVariables.x[66] += + acadoWorkspace.evGx[513]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[514]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[515]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[516]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[517]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[518]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[519]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[520]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[521]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[57];
acadoVariables.x[67] += + acadoWorkspace.evGx[522]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[523]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[524]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[525]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[526]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[527]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[528]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[529]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[530]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[58];
acadoVariables.x[68] += + acadoWorkspace.evGx[531]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[532]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[533]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[534]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[535]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[536]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[537]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[538]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[539]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[59];
acadoVariables.x[69] += + acadoWorkspace.evGx[540]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[541]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[542]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[543]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[544]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[545]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[546]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[547]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[548]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[60];
acadoVariables.x[70] += + acadoWorkspace.evGx[549]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[550]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[551]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[552]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[553]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[554]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[555]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[556]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[557]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[61];
acadoVariables.x[71] += + acadoWorkspace.evGx[558]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[559]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[560]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[561]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[562]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[563]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[564]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[565]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[566]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[62];
acadoVariables.x[72] += + acadoWorkspace.evGx[567]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[568]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[569]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[570]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[571]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[572]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[573]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[574]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[575]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[63];
acadoVariables.x[73] += + acadoWorkspace.evGx[576]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[577]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[578]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[579]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[580]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[581]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[582]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[583]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[584]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[64];
acadoVariables.x[74] += + acadoWorkspace.evGx[585]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[586]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[587]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[588]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[589]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[590]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[591]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[592]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[593]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[65];
acadoVariables.x[75] += + acadoWorkspace.evGx[594]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[595]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[596]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[597]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[598]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[599]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[600]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[601]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[602]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[66];
acadoVariables.x[76] += + acadoWorkspace.evGx[603]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[604]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[605]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[606]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[607]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[608]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[609]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[610]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[611]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[67];
acadoVariables.x[77] += + acadoWorkspace.evGx[612]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[613]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[614]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[615]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[616]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[617]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[618]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[619]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[620]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[68];
acadoVariables.x[78] += + acadoWorkspace.evGx[621]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[622]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[623]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[624]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[625]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[626]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[627]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[628]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[629]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[69];
acadoVariables.x[79] += + acadoWorkspace.evGx[630]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[631]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[632]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[633]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[634]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[635]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[636]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[637]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[638]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[70];
acadoVariables.x[80] += + acadoWorkspace.evGx[639]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[640]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[641]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[642]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[643]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[644]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[645]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[646]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[647]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[71];
acadoVariables.x[81] += + acadoWorkspace.evGx[648]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[649]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[650]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[651]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[652]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[653]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[654]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[655]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[656]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[72];
acadoVariables.x[82] += + acadoWorkspace.evGx[657]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[658]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[659]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[660]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[661]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[662]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[663]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[664]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[665]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[73];
acadoVariables.x[83] += + acadoWorkspace.evGx[666]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[667]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[668]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[669]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[670]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[671]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[672]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[673]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[674]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[74];
acadoVariables.x[84] += + acadoWorkspace.evGx[675]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[676]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[677]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[678]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[679]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[680]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[681]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[682]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[683]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[75];
acadoVariables.x[85] += + acadoWorkspace.evGx[684]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[685]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[686]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[687]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[688]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[689]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[690]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[691]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[692]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[76];
acadoVariables.x[86] += + acadoWorkspace.evGx[693]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[694]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[695]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[696]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[697]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[698]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[699]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[700]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[701]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[77];
acadoVariables.x[87] += + acadoWorkspace.evGx[702]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[703]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[704]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[705]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[706]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[707]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[708]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[709]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[710]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[78];
acadoVariables.x[88] += + acadoWorkspace.evGx[711]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[712]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[713]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[714]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[715]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[716]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[717]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[718]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[719]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[79];
acadoVariables.x[89] += + acadoWorkspace.evGx[720]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[721]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[722]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[723]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[724]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[725]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[726]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[727]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[728]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[80];
acadoVariables.x[90] += + acadoWorkspace.evGx[729]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[730]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[731]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[732]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[733]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[734]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[735]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[736]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[737]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[81];
acadoVariables.x[91] += + acadoWorkspace.evGx[738]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[739]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[740]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[741]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[742]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[743]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[744]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[745]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[746]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[82];
acadoVariables.x[92] += + acadoWorkspace.evGx[747]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[748]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[749]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[750]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[751]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[752]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[753]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[754]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[755]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[83];
acadoVariables.x[93] += + acadoWorkspace.evGx[756]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[757]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[758]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[759]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[760]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[761]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[762]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[763]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[764]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[84];
acadoVariables.x[94] += + acadoWorkspace.evGx[765]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[766]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[767]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[768]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[769]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[770]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[771]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[772]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[773]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[85];
acadoVariables.x[95] += + acadoWorkspace.evGx[774]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[775]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[776]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[777]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[778]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[779]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[780]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[781]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[782]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[86];
acadoVariables.x[96] += + acadoWorkspace.evGx[783]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[784]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[785]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[786]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[787]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[788]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[789]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[790]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[791]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[87];
acadoVariables.x[97] += + acadoWorkspace.evGx[792]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[793]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[794]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[795]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[796]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[797]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[798]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[799]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[800]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[88];
acadoVariables.x[98] += + acadoWorkspace.evGx[801]*acadoWorkspace.Dx0[0] + acadoWorkspace.evGx[802]*acadoWorkspace.Dx0[1] + acadoWorkspace.evGx[803]*acadoWorkspace.Dx0[2] + acadoWorkspace.evGx[804]*acadoWorkspace.Dx0[3] + acadoWorkspace.evGx[805]*acadoWorkspace.Dx0[4] + acadoWorkspace.evGx[806]*acadoWorkspace.Dx0[5] + acadoWorkspace.evGx[807]*acadoWorkspace.Dx0[6] + acadoWorkspace.evGx[808]*acadoWorkspace.Dx0[7] + acadoWorkspace.evGx[809]*acadoWorkspace.Dx0[8] + acadoWorkspace.d[89];

acado_multEDu( acadoWorkspace.E, acadoWorkspace.x, &(acadoVariables.x[ 9 ]) );
acado_multEDu( &(acadoWorkspace.E[ 27 ]), acadoWorkspace.x, &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 54 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 18 ]) );
acado_multEDu( &(acadoWorkspace.E[ 81 ]), acadoWorkspace.x, &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 135 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 27 ]) );
acado_multEDu( &(acadoWorkspace.E[ 162 ]), acadoWorkspace.x, &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 189 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 243 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 36 ]) );
acado_multEDu( &(acadoWorkspace.E[ 270 ]), acadoWorkspace.x, &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 297 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 351 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 378 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 45 ]) );
acado_multEDu( &(acadoWorkspace.E[ 405 ]), acadoWorkspace.x, &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 459 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 486 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 513 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 54 ]) );
acado_multEDu( &(acadoWorkspace.E[ 567 ]), acadoWorkspace.x, &(acadoVariables.x[ 63 ]) );
acado_multEDu( &(acadoWorkspace.E[ 594 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 63 ]) );
acado_multEDu( &(acadoWorkspace.E[ 621 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 63 ]) );
acado_multEDu( &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 63 ]) );
acado_multEDu( &(acadoWorkspace.E[ 675 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 63 ]) );
acado_multEDu( &(acadoWorkspace.E[ 702 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 63 ]) );
acado_multEDu( &(acadoWorkspace.E[ 729 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 63 ]) );
acado_multEDu( &(acadoWorkspace.E[ 756 ]), acadoWorkspace.x, &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 783 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 810 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 837 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 891 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 918 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 945 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 72 ]) );
acado_multEDu( &(acadoWorkspace.E[ 972 ]), acadoWorkspace.x, &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 999 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1026 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1053 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1107 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1134 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1161 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 81 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1215 ]), acadoWorkspace.x, &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1242 ]), &(acadoWorkspace.x[ 3 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1269 ]), &(acadoWorkspace.x[ 6 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.x[ 9 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1323 ]), &(acadoWorkspace.x[ 12 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1350 ]), &(acadoWorkspace.x[ 15 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1377 ]), &(acadoWorkspace.x[ 18 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.x[ 21 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1431 ]), &(acadoWorkspace.x[ 24 ]), &(acadoVariables.x[ 90 ]) );
acado_multEDu( &(acadoWorkspace.E[ 1458 ]), &(acadoWorkspace.x[ 27 ]), &(acadoVariables.x[ 90 ]) );
}

int acado_preparationStep( void )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep( void )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver( void )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation( void )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 9];
acadoWorkspace.state[1] = acadoVariables.x[index * 9 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 9 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 9 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 9 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 9 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 9 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 9 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 9 + 8];
acadoWorkspace.state[117] = acadoVariables.u[index * 3];
acadoWorkspace.state[118] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[119] = acadoVariables.u[index * 3 + 2];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 9 + 9] = acadoWorkspace.state[0];
acadoVariables.x[index * 9 + 10] = acadoWorkspace.state[1];
acadoVariables.x[index * 9 + 11] = acadoWorkspace.state[2];
acadoVariables.x[index * 9 + 12] = acadoWorkspace.state[3];
acadoVariables.x[index * 9 + 13] = acadoWorkspace.state[4];
acadoVariables.x[index * 9 + 14] = acadoWorkspace.state[5];
acadoVariables.x[index * 9 + 15] = acadoWorkspace.state[6];
acadoVariables.x[index * 9 + 16] = acadoWorkspace.state[7];
acadoVariables.x[index * 9 + 17] = acadoWorkspace.state[8];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 9] = acadoVariables.x[index * 9 + 9];
acadoVariables.x[index * 9 + 1] = acadoVariables.x[index * 9 + 10];
acadoVariables.x[index * 9 + 2] = acadoVariables.x[index * 9 + 11];
acadoVariables.x[index * 9 + 3] = acadoVariables.x[index * 9 + 12];
acadoVariables.x[index * 9 + 4] = acadoVariables.x[index * 9 + 13];
acadoVariables.x[index * 9 + 5] = acadoVariables.x[index * 9 + 14];
acadoVariables.x[index * 9 + 6] = acadoVariables.x[index * 9 + 15];
acadoVariables.x[index * 9 + 7] = acadoVariables.x[index * 9 + 16];
acadoVariables.x[index * 9 + 8] = acadoVariables.x[index * 9 + 17];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[90] = xEnd[0];
acadoVariables.x[91] = xEnd[1];
acadoVariables.x[92] = xEnd[2];
acadoVariables.x[93] = xEnd[3];
acadoVariables.x[94] = xEnd[4];
acadoVariables.x[95] = xEnd[5];
acadoVariables.x[96] = xEnd[6];
acadoVariables.x[97] = xEnd[7];
acadoVariables.x[98] = xEnd[8];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[90];
acadoWorkspace.state[1] = acadoVariables.x[91];
acadoWorkspace.state[2] = acadoVariables.x[92];
acadoWorkspace.state[3] = acadoVariables.x[93];
acadoWorkspace.state[4] = acadoVariables.x[94];
acadoWorkspace.state[5] = acadoVariables.x[95];
acadoWorkspace.state[6] = acadoVariables.x[96];
acadoWorkspace.state[7] = acadoVariables.x[97];
acadoWorkspace.state[8] = acadoVariables.x[98];
if (uEnd != 0)
{
acadoWorkspace.state[117] = uEnd[0];
acadoWorkspace.state[118] = uEnd[1];
acadoWorkspace.state[119] = uEnd[2];
}
else
{
acadoWorkspace.state[117] = acadoVariables.u[27];
acadoWorkspace.state[118] = acadoVariables.u[28];
acadoWorkspace.state[119] = acadoVariables.u[29];
}

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[90] = acadoWorkspace.state[0];
acadoVariables.x[91] = acadoWorkspace.state[1];
acadoVariables.x[92] = acadoWorkspace.state[2];
acadoVariables.x[93] = acadoWorkspace.state[3];
acadoVariables.x[94] = acadoWorkspace.state[4];
acadoVariables.x[95] = acadoWorkspace.state[5];
acadoVariables.x[96] = acadoWorkspace.state[6];
acadoVariables.x[97] = acadoWorkspace.state[7];
acadoVariables.x[98] = acadoWorkspace.state[8];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[27] = uEnd[0];
acadoVariables.u[28] = uEnd[1];
acadoVariables.u[29] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29];
kkt = fabs( kkt );
for (index = 0; index < 30; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 12 */
real_t tmpDy[ 12 ];

/** Row vector of size: 9 */
real_t tmpDyN[ 9 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 3 + 2];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 12] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 12];
acadoWorkspace.Dy[lRun1 * 12 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 12 + 1];
acadoWorkspace.Dy[lRun1 * 12 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 12 + 2];
acadoWorkspace.Dy[lRun1 * 12 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 12 + 3];
acadoWorkspace.Dy[lRun1 * 12 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 12 + 4];
acadoWorkspace.Dy[lRun1 * 12 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 12 + 5];
acadoWorkspace.Dy[lRun1 * 12 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 12 + 6];
acadoWorkspace.Dy[lRun1 * 12 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 12 + 7];
acadoWorkspace.Dy[lRun1 * 12 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 12 + 8];
acadoWorkspace.Dy[lRun1 * 12 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 12 + 9];
acadoWorkspace.Dy[lRun1 * 12 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 12 + 10];
acadoWorkspace.Dy[lRun1 * 12 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 12 + 11];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[90];
acadoWorkspace.objValueIn[1] = acadoVariables.x[91];
acadoWorkspace.objValueIn[2] = acadoVariables.x[92];
acadoWorkspace.objValueIn[3] = acadoVariables.x[93];
acadoWorkspace.objValueIn[4] = acadoVariables.x[94];
acadoWorkspace.objValueIn[5] = acadoVariables.x[95];
acadoWorkspace.objValueIn[6] = acadoVariables.x[96];
acadoWorkspace.objValueIn[7] = acadoVariables.x[97];
acadoWorkspace.objValueIn[8] = acadoVariables.x[98];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8] - acadoVariables.yN[8];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 12] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 24] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 36] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 48] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 60] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 72] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 84] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 96] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 108] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 120] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 132];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 1] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 13] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 25] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 37] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 49] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 61] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 73] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 85] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 97] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 109] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 121] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 133];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 2] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 14] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 26] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 38] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 50] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 62] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 74] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 86] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 98] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 110] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 122] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 134];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 3] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 15] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 27] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 39] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 51] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 63] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 75] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 87] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 99] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 111] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 123] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 135];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 4] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 16] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 28] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 40] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 52] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 64] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 76] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 88] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 100] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 112] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 124] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 136];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 5] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 17] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 29] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 41] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 53] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 65] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 77] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 89] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 101] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 113] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 125] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 137];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 6] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 18] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 30] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 42] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 54] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 66] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 78] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 90] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 102] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 114] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 126] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 138];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 7] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 19] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 31] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 43] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 55] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 67] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 79] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 91] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 103] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 115] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 127] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 139];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 8] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 20] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 32] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 44] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 56] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 68] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 80] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 92] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 104] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 116] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 128] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 140];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 9] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 21] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 33] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 45] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 57] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 69] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 81] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 93] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 105] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 117] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 129] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 141];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 10] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 22] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 34] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 46] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 58] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 70] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 82] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 94] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 106] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 118] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 130] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 142];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 11] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 23] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 35] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 47] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 59] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 71] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 83] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 95] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 107] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 119] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 131] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 143];
objVal += + acadoWorkspace.Dy[lRun1 * 12]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 12 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 12 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 12 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 12 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 12 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 12 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 12 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 12 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 12 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 12 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 12 + 11]*tmpDy[11];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[10];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[20];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[30];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[40];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[50];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[60];
tmpDyN[7] = + acadoWorkspace.DyN[7]*acadoVariables.WN[70];
tmpDyN[8] = + acadoWorkspace.DyN[8]*acadoVariables.WN[80];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7] + acadoWorkspace.DyN[8]*tmpDyN[8];

objVal *= 0.5;
return objVal;
}


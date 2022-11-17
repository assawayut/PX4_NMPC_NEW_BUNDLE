/* Produced by CVXGEN, 2022-11-04 02:13:58 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.x_ss_0[0] = 0.20319161029830202;
  params.x_ss_0[1] = 0.8325912904724193;
  params.x_ss_0[2] = -0.8363810443482227;
  params.x_ss_0[3] = 0.04331042079065206;
  params.x_ss_0[4] = 1.5717878173906188;
  params.x_ss_0[5] = 1.5851723557337523;
  params.x_ss_0[6] = -1.497658758144655;
  params.x_ss_0[7] = -1.171028487447253;
  params.x_ss_0[8] = -1.7941311867966805;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.4408098436506365;
  params.Q[9] = 0;
  params.Q[18] = 0;
  params.Q[27] = 0;
  params.Q[36] = 0;
  params.Q[45] = 0;
  params.Q[54] = 0;
  params.Q[63] = 0;
  params.Q[72] = 0;
  params.Q[1] = 0;
  params.Q[10] = 1.0298762108785668;
  params.Q[19] = 0;
  params.Q[28] = 0;
  params.Q[37] = 0;
  params.Q[46] = 0;
  params.Q[55] = 0;
  params.Q[64] = 0;
  params.Q[73] = 0;
  params.Q[2] = 0;
  params.Q[11] = 0;
  params.Q[20] = 1.456833224394711;
  params.Q[29] = 0;
  params.Q[38] = 0;
  params.Q[47] = 0;
  params.Q[56] = 0;
  params.Q[65] = 0;
  params.Q[74] = 0;
  params.Q[3] = 0;
  params.Q[12] = 0;
  params.Q[21] = 0;
  params.Q[30] = 1.6491440476147607;
  params.Q[39] = 0;
  params.Q[48] = 0;
  params.Q[57] = 0;
  params.Q[66] = 0;
  params.Q[75] = 0;
  params.Q[4] = 0;
  params.Q[13] = 0;
  params.Q[22] = 0;
  params.Q[31] = 0;
  params.Q[40] = 1.2784872826479754;
  params.Q[49] = 0;
  params.Q[58] = 0;
  params.Q[67] = 0;
  params.Q[76] = 0;
  params.Q[5] = 0;
  params.Q[14] = 0;
  params.Q[23] = 0;
  params.Q[32] = 0;
  params.Q[41] = 0;
  params.Q[50] = 1.6762549019801312;
  params.Q[59] = 0;
  params.Q[68] = 0;
  params.Q[77] = 0;
  params.Q[6] = 0;
  params.Q[15] = 0;
  params.Q[24] = 0;
  params.Q[33] = 0;
  params.Q[42] = 0;
  params.Q[51] = 0;
  params.Q[60] = 1.5908628174163508;
  params.Q[69] = 0;
  params.Q[78] = 0;
  params.Q[7] = 0;
  params.Q[16] = 0;
  params.Q[25] = 0;
  params.Q[34] = 0;
  params.Q[43] = 0;
  params.Q[52] = 0;
  params.Q[61] = 0;
  params.Q[70] = 1.0239818823771654;
  params.Q[79] = 0;
  params.Q[8] = 0;
  params.Q[17] = 0;
  params.Q[26] = 0;
  params.Q[35] = 0;
  params.Q[44] = 0;
  params.Q[53] = 0;
  params.Q[62] = 0;
  params.Q[71] = 0;
  params.Q[80] = 1.5588540879908819;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.R[0] = 1.2592524469074653;
  params.R[3] = 0;
  params.R[6] = 0;
  params.R[1] = 0;
  params.R[4] = 1.4151011970100695;
  params.R[7] = 0;
  params.R[2] = 0;
  params.R[5] = 0;
  params.R[8] = 1.2835250817713186;
  params.x_ss_1[0] = 0.7725516732519853;
  params.x_ss_1[1] = -0.23818512931704205;
  params.x_ss_1[2] = -1.372529046100147;
  params.x_ss_1[3] = 0.17859607212737894;
  params.x_ss_1[4] = 1.1212590580454682;
  params.x_ss_1[5] = -0.774545870495281;
  params.x_ss_1[6] = -1.1121684642712744;
  params.x_ss_1[7] = -0.44811496977740495;
  params.x_ss_1[8] = 1.7455345994417217;
  params.x_ss_2[0] = 1.9039816898917352;
  params.x_ss_2[1] = 0.6895347036512547;
  params.x_ss_2[2] = 1.6113364341535923;
  params.x_ss_2[3] = 1.383003485172717;
  params.x_ss_2[4] = -0.48802383468444344;
  params.x_ss_2[5] = -1.631131964513103;
  params.x_ss_2[6] = 0.6136436100941447;
  params.x_ss_2[7] = 0.2313630495538037;
  params.x_ss_2[8] = -0.5537409477496875;
  params.x_ss_3[0] = -1.0997819806406723;
  params.x_ss_3[1] = -0.3739203344950055;
  params.x_ss_3[2] = -0.12423900520332376;
  params.x_ss_3[3] = -0.923057686995755;
  params.x_ss_3[4] = -0.8328289030982696;
  params.x_ss_3[5] = -0.16925440270808823;
  params.x_ss_3[6] = 1.442135651787706;
  params.x_ss_3[7] = 0.34501161787128565;
  params.x_ss_3[8] = -0.8660485502711608;
  params.x_ss_4[0] = -0.8880899735055947;
  params.x_ss_4[1] = -0.1815116979122129;
  params.x_ss_4[2] = -1.17835862158005;
  params.x_ss_4[3] = -1.1944851558277074;
  params.x_ss_4[4] = 0.05614023926976763;
  params.x_ss_4[5] = -1.6510825248767813;
  params.x_ss_4[6] = -0.06565787059365391;
  params.x_ss_4[7] = -0.5512951504486665;
  params.x_ss_4[8] = 0.8307464872626844;
  params.x_ss_5[0] = 0.9869848924080182;
  params.x_ss_5[1] = 0.7643716874230573;
  params.x_ss_5[2] = 0.7567216550196565;
  params.x_ss_5[3] = -0.5055995034042868;
  params.x_ss_5[4] = 0.6725392189410702;
  params.x_ss_5[5] = -0.6406053441727284;
  params.x_ss_5[6] = 0.29117547947550015;
  params.x_ss_5[7] = -0.6967713677405021;
  params.x_ss_5[8] = -0.21941980294587182;
  params.x_ss_6[0] = -1.753884276680243;
  params.x_ss_6[1] = -1.0292983112626475;
  params.x_ss_6[2] = 1.8864104246942706;
  params.x_ss_6[3] = -1.077663182579704;
  params.x_ss_6[4] = 0.7659100437893209;
  params.x_ss_6[5] = 0.6019074328549583;
  params.x_ss_6[6] = 0.8957565577499285;
  params.x_ss_6[7] = -0.09964555746227477;
  params.x_ss_6[8] = 0.38665509840745127;
  params.x_ss_7[0] = -1.7321223042686946;
  params.x_ss_7[1] = -1.7097514487110663;
  params.x_ss_7[2] = -1.2040958948116867;
  params.x_ss_7[3] = -1.3925560119658358;
  params.x_ss_7[4] = -1.5995826216742213;
  params.x_ss_7[5] = -1.4828245415645833;
  params.x_ss_7[6] = 0.21311092723061398;
  params.x_ss_7[7] = -1.248740700304487;
  params.x_ss_7[8] = 1.808404972124833;
  params.x_ss_8[0] = 0.7264471152297065;
  params.x_ss_8[1] = 0.16407869343908477;
  params.x_ss_8[2] = 0.8287224032315907;
  params.x_ss_8[3] = -0.9444533161899464;
  params.x_ss_8[4] = 1.7069027370149112;
  params.x_ss_8[5] = 1.3567722311998827;
  params.x_ss_8[6] = 0.9052779937121489;
  params.x_ss_8[7] = -0.07904017565835986;
  params.x_ss_8[8] = 1.3684127435065871;
  params.x_ss_9[0] = 0.979009293697437;
  params.x_ss_9[1] = 0.6413036255984501;
  params.x_ss_9[2] = 1.6559010680237511;
  params.x_ss_9[3] = 0.5346622551502991;
  params.x_ss_9[4] = -0.5362376605895625;
  params.x_ss_9[5] = 0.2113782926017822;
  params.x_ss_9[6] = -1.2144776931994525;
  params.x_ss_9[7] = -1.2317108144255875;
  params.x_ss_9[8] = 0.9026784957312834;
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.P[0] = 1.7849367034311312;
  params.P[9] = 0;
  params.P[18] = 0;
  params.P[27] = 0;
  params.P[36] = 0;
  params.P[45] = 0;
  params.P[54] = 0;
  params.P[63] = 0;
  params.P[72] = 0;
  params.P[1] = 0;
  params.P[10] = 1.9720983636837657;
  params.P[19] = 0;
  params.P[28] = 0;
  params.P[37] = 0;
  params.P[46] = 0;
  params.P[55] = 0;
  params.P[64] = 0;
  params.P[73] = 0;
  params.P[2] = 0;
  params.P[11] = 0;
  params.P[20] = 1.8509714170415017;
  params.P[29] = 0;
  params.P[38] = 0;
  params.P[47] = 0;
  params.P[56] = 0;
  params.P[65] = 0;
  params.P[74] = 0;
  params.P[3] = 0;
  params.P[12] = 0;
  params.P[21] = 0;
  params.P[30] = 1.5435943265958227;
  params.P[39] = 0;
  params.P[48] = 0;
  params.P[57] = 0;
  params.P[66] = 0;
  params.P[75] = 0;
  params.P[4] = 0;
  params.P[13] = 0;
  params.P[22] = 0;
  params.P[31] = 0;
  params.P[40] = 1.089790869523065;
  params.P[49] = 0;
  params.P[58] = 0;
  params.P[67] = 0;
  params.P[76] = 0;
  params.P[5] = 0;
  params.P[14] = 0;
  params.P[23] = 0;
  params.P[32] = 0;
  params.P[41] = 0;
  params.P[50] = 1.4888732446161128;
  params.P[59] = 0;
  params.P[68] = 0;
  params.P[77] = 0;
  params.P[6] = 0;
  params.P[15] = 0;
  params.P[24] = 0;
  params.P[33] = 0;
  params.P[42] = 0;
  params.P[51] = 0;
  params.P[60] = 1.9279363475621256;
  params.P[69] = 0;
  params.P[78] = 0;
  params.P[7] = 0;
  params.P[16] = 0;
  params.P[25] = 0;
  params.P[34] = 0;
  params.P[43] = 0;
  params.P[52] = 0;
  params.P[61] = 0;
  params.P[70] = 1.7876181995034763;
  params.P[79] = 0;
  params.P[8] = 0;
  params.P[17] = 0;
  params.P[26] = 0;
  params.P[35] = 0;
  params.P[44] = 0;
  params.P[53] = 0;
  params.P[62] = 0;
  params.P[71] = 0;
  params.P[80] = 1.485094226054088;
  params.A[0] = -0.1788825540764547;
  params.A[1] = -1.1280569263625857;
  params.A[2] = -1.2911464767927057;
  params.A[3] = -1.7055053231225696;
  params.A[4] = 1.56957275034837;
  params.A[5] = 0.5607064675962357;
  params.A[6] = -1.4266707301147146;
  params.A[7] = -0.3434923211351708;
  params.A[8] = -1.8035643024085055;
  params.A[9] = -1.1625066019105454;
  params.A[10] = 0.9228324965161532;
  params.A[11] = 0.6044910817663975;
  params.A[12] = -0.0840868104920891;
  params.A[13] = -0.900877978017443;
  params.A[14] = 0.608892500264739;
  params.A[15] = 1.8257980452695217;
  params.A[16] = -0.25791777529922877;
  params.A[17] = -1.7194699796493191;
  params.A[18] = -1.7690740487081298;
  params.A[19] = -1.6685159248097703;
  params.A[20] = 1.8388287490128845;
  params.A[21] = 0.16304334474597537;
  params.A[22] = 1.3498497306788897;
  params.A[23] = -1.3198658230514613;
  params.A[24] = -0.9586197090843394;
  params.A[25] = 0.7679100474913709;
  params.A[26] = 1.5822813125679343;
  params.A[27] = -0.6372460621593619;
  params.A[28] = -1.741307208038867;
  params.A[29] = 1.456478677642575;
  params.A[30] = -0.8365102166820959;
  params.A[31] = 0.9643296255982503;
  params.A[32] = -1.367865381194024;
  params.A[33] = 0.7798537405635035;
  params.A[34] = 1.3656784761245926;
  params.A[35] = 0.9086083149868371;
  params.A[36] = -0.5635699005460344;
  params.A[37] = 0.9067590059607915;
  params.A[38] = -1.4421315032701587;
  params.A[39] = -0.7447235390671119;
  params.A[40] = -0.32166897326822186;
  params.A[41] = 1.5088481557772684;
  params.A[42] = -1.385039165715428;
  params.A[43] = 1.5204991609972622;
  params.A[44] = 1.1958572768832156;
  params.A[45] = 1.8864971883119228;
  params.A[46] = -0.5291880667861584;
  params.A[47] = -1.1802409243688836;
  params.A[48] = -1.037718718661604;
  params.A[49] = 1.3114512056856835;
  params.A[50] = 1.8609125943756615;
  params.A[51] = 0.7952399935216938;
  params.A[52] = -0.07001183290468038;
  params.A[53] = -0.8518009412754686;
  params.A[54] = 1.3347515373726386;
  params.A[55] = 1.4887180335977037;
  params.A[56] = -1.6314736327976336;
  params.A[57] = -1.1362021159208933;
  params.A[58] = 1.327044361831466;
  params.A[59] = 1.3932155883179842;
  params.A[60] = -0.7413880049440107;
  params.A[61] = -0.8828216126125747;
  params.A[62] = -0.27673991192616;
  params.A[63] = 0.15778600105866714;
  params.A[64] = -1.6177327399735457;
  params.A[65] = 1.3476485548544606;
  params.A[66] = 0.13893948140528378;
  params.A[67] = 1.0998712601636944;
  params.A[68] = -1.0766549376946926;
  params.A[69] = 1.8611734044254629;
  params.A[70] = 1.0041092292735172;
  params.A[71] = -0.6276245424321543;
  params.A[72] = 1.794110587839819;
  params.A[73] = 0.8020471158650913;
  params.A[74] = 1.362244341944948;
  params.A[75] = -1.8180107765765245;
  params.A[76] = -1.7774338357932473;
  params.A[77] = 0.9709490941985153;
  params.A[78] = -0.7812542682064318;
  params.A[79] = 0.0671374633729811;
  params.A[80] = -1.374950305314906;
  params.B[0] = 1.9118096386279388;
  params.B[1] = 0.011004190697677885;
  params.B[2] = 1.3160043138989015;
  params.B[3] = -1.7038488148800144;
  params.B[4] = -0.08433819112864738;
  params.B[5] = -1.7508820783768964;
  params.B[6] = 1.536965724350949;
  params.B[7] = -0.21675928514816478;
  params.B[8] = -1.725800326952653;
  params.B[9] = -1.6940148707361717;
  params.B[10] = 0.15517063201268;
  params.B[11] = -1.697734381979077;
  params.B[12] = -1.264910727950229;
  params.B[13] = -0.2545716633339441;
  params.B[14] = -0.008868675926170244;
  params.B[15] = 0.3332476609670296;
  params.B[16] = 0.48205072561962936;
  params.B[17] = -0.5087540014293261;
  params.B[18] = 0.4749463319223195;
  params.B[19] = -1.371021366459455;
  params.B[20] = -0.8979660982652256;
  params.B[21] = 1.194873082385242;
  params.B[22] = -1.3876427970939353;
  params.B[23] = -1.106708108457053;
  params.B[24] = -1.0280872812241797;
  params.B[25] = -0.08197078070773234;
  params.B[26] = -1.9970179118324083;
}
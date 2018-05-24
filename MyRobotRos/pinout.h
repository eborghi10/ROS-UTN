#pragma once

const uint8_t EN_left = 11;
const uint8_t EN_right = 12;
const uint8_t D0_left = A0;
const uint8_t D1_left = A1;
const uint8_t D0_right = A3;
const uint8_t D1_right = A4;

const uint8_t encoder_left_pin = 6;
const uint8_t encoder_right_pin = 7;

ros::NodeHandle  nh;

double rate(100.0); // Hz

const double wheel_radius = 3.1 / 1E2;    // 3.1 cm
const double wheel_distance = 14.6 / 1E2; // 14.6 cm

const int NUM_POINTS_LEFT = 48;
const int NUM_POINTS_RIGHT = 79;

const double real_left[NUM_POINTS_LEFT] = {0.101638637483, 0.224755629897, 0.366282612085, 0.512028634548, 0.674650430679, 0.857983529568, 1.04016602039, 1.21774590015, 1.3895727396, 1.53646934032, 1.66994202137, 1.78653872013, 1.88242435455, 1.97332382202, 2.05578541756, 2.1378633976, 2.22147536278, 2.31275844574, 2.41478061676, 2.52485728264, 2.65832972527, 2.81941747665, 2.98395705223, 3.16268754005, 3.35100674629, 3.5174639225, 3.66052508354, 3.78555965424, 3.90254020691, 4.00916481018, 4.11540603638, 4.21320915222, 4.30410861969, 4.39999389648, 4.49587965012, 4.59828519821, 4.70759487152, 4.82457494736, 4.95114374161, 5.10417747498, 5.27331924438, 5.43900918961, 5.60393285751, 5.76080131531, 5.89849281311, 6.02237701416, 6.14050769806, 6.24828338623};
const double ideal_left[NUM_POINTS_LEFT] = {0.00000, 0.13368, 0.26737, 0.40105, 0.53474, 0.66842, 0.80211, 0.93579, 1.06948, 1.20316, 1.33685, 1.47053, 1.60422, 1.73790, 1.87159, 2.00527, 2.13896, 2.27264, 2.40633, 2.54001, 2.67370, 2.80738, 2.94107, 3.07475, 3.20844, 3.34212, 3.47580, 3.60949, 3.74317, 3.87686, 4.01054, 4.14423, 4.27791, 4.41160, 4.54528, 4.67897, 4.81265, 4.94634, 5.08002, 5.21371, 5.34739, 5.48108, 5.61476, 5.74845, 5.88213, 6.01582, 6.14950, 6.28319};

const double real_right[NUM_POINTS_RIGHT] = {0.0958855077624, 0.227056875825, 0.349023252726, 0.464085847139, 0.571861147881, 0.67656815052, 0.781658649445, 0.87140750885, 0.958855092525, 1.02827620506, 1.08734154701, 1.14218819141, 1.19204866886, 1.23922431469, 1.28755056858, 1.33472633362, 1.3895727396, 1.44441926479, 1.49504685402, 1.54414021969, 1.58824753761, 1.6365737915, 1.68605077267, 1.73782896996, 1.79574382305, 1.85749399662, 1.93113410473, 2.01129436493, 2.08992052078, 2.16432762146, 2.23183107376, 2.30393695831, 2.38678193092, 2.46502447128, 2.55515694618, 2.66139817238, 2.77492666245, 2.89689278603, 3.01272273064, 3.13545608521, 3.24860095978, 3.36366343498, 3.47949337959, 3.59532308578, 3.71268677711, 3.82813286781, 3.9190325737, 4.00302839279, 4.08817481995, 4.16488265991, 4.2423582077, 4.31408071518, 4.38196754456, 4.44985437393, 4.50853633881, 4.55954742432, 4.61170959473, 4.66425466537, 4.72063493729, 4.76972866058, 4.81920576096, 4.87060022354, 4.91777610779, 4.97377300262, 5.02555131912, 5.08039808273, 5.1386961937, 5.19277524948, 5.25069046021, 5.31435823441, 5.37956047058, 5.47007608414, 5.56826305389, 5.669901371, 5.77499198914, 5.87202835083, 5.9759683609, 6.0940990448, 6.22220230103};
const double ideal_right[NUM_POINTS_RIGHT] = {0.00000, 0.08055, 0.16111, 0.24166, 0.32221, 0.40277, 0.48332, 0.56388, 0.64443, 0.72498, 0.80554, 0.88609, 0.96664, 1.04720, 1.12775, 1.20830, 1.28886, 1.36941, 1.44997, 1.53052, 1.61107, 1.69163, 1.77218, 1.85273, 1.93329, 2.01384, 2.09440, 2.17495, 2.25550, 2.33606, 2.41661, 2.49716, 2.57772, 2.65827, 2.73882, 2.81938, 2.89993, 2.98049, 3.06104, 3.14159, 3.22215, 3.30270, 3.38325, 3.46381, 3.54436, 3.62491, 3.70547, 3.78602, 3.86658, 3.94713, 4.02768, 4.10824, 4.18879, 4.26934, 4.34990, 4.43045, 4.51100, 4.59156, 4.67211, 4.75267, 4.83322, 4.91377, 4.99433, 5.07488, 5.15543, 5.23599, 5.31654, 5.39710, 5.47765, 5.55820, 5.63876, 5.71931, 5.79986, 5.88042, 5.96097, 6.04152, 6.12208, 6.20263, 6.28319};

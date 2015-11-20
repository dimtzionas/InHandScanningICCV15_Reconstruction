// Author: Dimitrios Tzionas
//
// source code for the work:
//
// Dimitrios Tzionas and Juergen Gall
// 3D Object Reconstruction from Hand-Object Interactions
// International Conference on Computer Vision (ICCV) 2015
// http://files.is.tue.mpg.de/dtzionas/In-Hand-Scanning


#include "skinndetector.h"


// For the data of the model defined below, please see page 21 @ the paper
// "Statistical Color Models with Application to Skin Detection"
// by M.Jones and J.Rehg


// Eigen::Vector3d mean;                                                              // see page 21 @ the paper "Statistical Color Models with Application to Skin Detection" by M.Jones and J.Rehg
// Eigen::Vector3d invCovariance; // inverse of the diagonal Covariance matrix        // see page 21 @ the paper "Statistical Color Models with Application to Skin Detection" by M.Jones and J.Rehg
// double          finalWeight;   // mixture weight / ( (2*pi)^3/2 * sqrt( |cov| ) ); // see page 10 @ the paper "Statistical Color Models with Application to Skin Detection" by M.Jones and J.Rehg


void SkinnDetector::createSkinModel_CUSTOM()
{

}

void SkinnDetector::createSkinModel_JONES_REHG()
{

        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
        modelSkinn_POS.resize( PARAM_NUMBBB_Mixture_Components );
        modelSkinn_NEG.resize( PARAM_NUMBBB_Mixture_Components );
        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////

        modelSkinn_POS[0].mean          << 73.53,       29.94,      17.76;
        modelSkinn_POS[0].invCovariance << 0.00130651,  0.00823452, 0.00886525;
        modelSkinn_POS[0].finalWeight   =  0.00000058;

        modelSkinn_POS[1].mean          << 249.71,      233.94,     217.49;
        modelSkinn_POS[1].invCovariance << 0.02503756,  0.00647501, 0.00252493;
        modelSkinn_POS[1].finalWeight   =  0.00000134;

        modelSkinn_POS[2].mean          << 161.68,      116.25,     96.95;
        modelSkinn_POS[2].invCovariance << 0.00343607,  0.01653439, 0.00614062;
        modelSkinn_POS[2].finalWeight   =  0.00000245;

        modelSkinn_POS[3].mean          << 186.07,      136.62,     114.40;
        modelSkinn_POS[3].invCovariance << 0.00363702,  0.01547988, 0.00504363;
        modelSkinn_POS[3].finalWeight   =  0.00000256;

        modelSkinn_POS[4].mean          << 189.26,      98.37,      51.18;
        modelSkinn_POS[4].invCovariance << 0.00157933,  0.00449640, 0.00398899;
        modelSkinn_POS[4].finalWeight   =  0.00000059;

        modelSkinn_POS[5].mean          << 247.00,      152.20,     90.84;
        modelSkinn_POS[5].invCovariance << 0.01533037,  0.00144607, 0.00163956;
        modelSkinn_POS[5].finalWeight   =  0.00000038;

        modelSkinn_POS[6].mean          << 150.10,      72.66,      37.76;
        modelSkinn_POS[6].invCovariance << 0.00244720,  0.00498082, 0.00388244;
        modelSkinn_POS[6].finalWeight   =  0.00000063;

        modelSkinn_POS[7].mean          << 206.85,      171.09,     156.34;
        modelSkinn_POS[7].invCovariance << 0.00188651,  0.00644828, 0.00174584;
        modelSkinn_POS[7].finalWeight   =  0.00000043;

        modelSkinn_POS[8].mean          << 212.78,      152.82,     120.04;
        modelSkinn_POS[8].invCovariance << 0.00622781,  0.01183152, 0.00410004;
        modelSkinn_POS[8].finalWeight   =  0.00000334;

        modelSkinn_POS[9].mean          << 234.87,      175.43,     138.94;
        modelSkinn_POS[9].invCovariance << 0.00610501,  0.00822571, 0.00358141;
        modelSkinn_POS[9].finalWeight   =  0.00000205;

        modelSkinn_POS[10].mean          << 151.19,     97.74,      74.59;
        modelSkinn_POS[10].invCovariance << 0.00235073, 0.01359434, 0.00571070;
        modelSkinn_POS[10].finalWeight   =  0.00000298;

        modelSkinn_POS[11].mean          << 120.52,     77.55,      59.82;
        modelSkinn_POS[11].invCovariance << 0.00302618, 0.01421666, 0.00658675;
        modelSkinn_POS[11].finalWeight   =  0.00000228;

        modelSkinn_POS[12].mean          << 192.20,     119.62,     82.32;
        modelSkinn_POS[12].invCovariance << 0.00654622, 0.01085305, 0.00385877;
        modelSkinn_POS[12].finalWeight   =  0.00000251;

        modelSkinn_POS[13].mean          << 214.29,     136.08,     87.24;
        modelSkinn_POS[13].invCovariance << 0.00488043, 0.00713419, 0.00370110;
        modelSkinn_POS[13].finalWeight   =  0.00000114;

        modelSkinn_POS[14].mean          << 99.57,      54.33,      38.06;
        modelSkinn_POS[14].invCovariance << 0.00223150, 0.01108893, 0.00660982;
        modelSkinn_POS[14].finalWeight   =  0.00000171;

        modelSkinn_POS[15].mean          << 238.88,     203.08,     176.91;
        modelSkinn_POS[15].invCovariance << 0.00560601, 0.00639918, 0.00246920;
        modelSkinn_POS[15].finalWeight   =  0.00000142;

        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////////////////////////////////////////////

        modelSkinn_NEG[0].mean          << 254.37,     254.41,      253.82;
        modelSkinn_NEG[0].invCovariance << 0.36101083, 0.35587189,  0.18315018;
        modelSkinn_NEG[0].finalWeight   =  0.00062041;

        modelSkinn_NEG[1].mean          << 9.39,       8.09,        8.52;
        modelSkinn_NEG[1].invCovariance << 0.02134927, 0.02977077,  0.03078818;
        modelSkinn_NEG[1].finalWeight   =  0.00001449;

        modelSkinn_NEG[2].mean          << 96.57,      96.95,       91.53;
        modelSkinn_NEG[2].invCovariance << 0.00356265, 0.00637796,  0.00229053;
        modelSkinn_NEG[2].finalWeight   =  0.00000125;

        modelSkinn_NEG[3].mean          << 160.44,     162.49,      159.06;
        modelSkinn_NEG[3].invCovariance << 0.00280915, 0.00862887,  0.00169136;
        modelSkinn_NEG[3].finalWeight   =  0.00000082;

        modelSkinn_NEG[4].mean          << 74.98,      63.23,       46.33;
        modelSkinn_NEG[4].invCovariance << 0.00241057, 0.00406587,  0.00276801;
        modelSkinn_NEG[4].finalWeight   =  0.00000078;

        modelSkinn_NEG[5].mean          << 121.83,     60.88,       18.31;
        modelSkinn_NEG[5].invCovariance << 0.00039964, 0.00072279,  0.00421621;
        modelSkinn_NEG[5].finalWeight   =  0.00000008;

        modelSkinn_NEG[6].mean          << 202.18,     154.88,      91.04;
        modelSkinn_NEG[6].invCovariance << 0.00104447, 0.00056595,  0.00063190;
        modelSkinn_NEG[6].finalWeight   =  0.00000004;

        modelSkinn_NEG[7].mean          << 193.06,     201.93,      206.55;
        modelSkinn_NEG[7].invCovariance << 0.00177658, 0.00525679,  0.00223574;
        modelSkinn_NEG[7].finalWeight   =  0.00000060;

        modelSkinn_NEG[8].mean          << 51.88,      57.14,       61.55;
        modelSkinn_NEG[8].invCovariance << 0.00290605, 0.00521458,  0.00230734;
        modelSkinn_NEG[8].finalWeight   =  0.00000078;

        modelSkinn_NEG[9].mean          << 30.88,       26.84,      25.32;
        modelSkinn_NEG[9].invCovariance << 0.00450308,  0.00842815, 0.00548216;
        modelSkinn_NEG[9].finalWeight   =  0.00000344;

        modelSkinn_NEG[10].mean          << 44.97,      85.96,      131.95;
        modelSkinn_NEG[10].invCovariance << 0.00153534, 0.00118974, 0.00103770;
        modelSkinn_NEG[10].finalWeight   =  0.00000010;

        modelSkinn_NEG[11].mean          << 236.02,     236.27,     230.70;
        modelSkinn_NEG[11].invCovariance << 0.00444385, 0.00852588, 0.00301250;
        modelSkinn_NEG[11].finalWeight   =  0.00000182;

        modelSkinn_NEG[12].mean          << 207.86,     191.20,     164.12;
        modelSkinn_NEG[12].invCovariance << 0.00202413, 0.00420716, 0.00187434;
        modelSkinn_NEG[12].finalWeight   =  0.00000030;

        modelSkinn_NEG[13].mean          << 99.83,      148.11,     188.17;
        modelSkinn_NEG[13].invCovariance << 0.00104616, 0.00152683, 0.00109087;
        modelSkinn_NEG[13].finalWeight   =  0.00000010;

        modelSkinn_NEG[14].mean          << 135.06,     131.92,     123.10;
        modelSkinn_NEG[14].invCovariance << 0.00285429, 0.00767460, 0.00257447;
        modelSkinn_NEG[14].finalWeight   =  0.00000142;

        modelSkinn_NEG[15].mean          << 135.96,     103.89,     66.88;
        modelSkinn_NEG[15].invCovariance << 0.00124002, 0.00155715, 0.00285421;
        modelSkinn_NEG[15].finalWeight   =  0.00000022;

}

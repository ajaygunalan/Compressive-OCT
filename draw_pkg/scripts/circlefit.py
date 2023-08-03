import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.ndimage import gaussian_filter1d
# -----------------------------------------------------------------------------
# Data

# coordinates = []
# xp = []
# yp = []
# for x in range(11):
#   for y in range(11):
#     coordinates.append((x, y))

# for each in coordinates:
#     xp.append(each[0])
#     yp.append(each[1])
# print(xp, yp)


xp=np.array([ -1.19824526e-01,  -1.19795807e-01,  -1.22298912e-01,
        -1.24784611e-01,  -1.27233423e-01,  -1.27048456e-01,
        -1.29424259e-01,  -1.31781573e-01,  -1.34102825e-01,
        -1.36386619e-01,  -1.41324999e-01,  -1.43569618e-01,
        -1.48471481e-01,  -1.53300646e-01,  -1.55387133e-01,
        -1.57436481e-01,  -1.53938796e-01,  -1.58562951e-01,
        -1.53139517e-01,  -1.50456275e-01,  -1.49637920e-01,
        -1.48774455e-01,  -1.47843528e-01,  -1.44278335e-01,
        -1.43299274e-01,  -1.39716798e-01,  -1.36111285e-01,
        -1.32534352e-01,  -1.28982866e-01,  -1.25433151e-01,
        -1.21912263e-01,  -1.16106245e-01,  -1.12701128e-01,
        -1.09303316e-01,  -1.05947571e-01,  -1.00467194e-01,
        -9.72083398e-02,  -9.39822094e-02,  -9.08033710e-02,
        -8.96420533e-02,  -8.65053261e-02,  -8.34162875e-02,
        -8.03788778e-02,  -7.73929193e-02,  -7.62032638e-02,
        -7.32655732e-02,  -7.03760465e-02,  -6.91826390e-02,
        -6.63378816e-02,  -6.35537275e-02,  -6.08302060e-02,
        -5.96426925e-02,  -5.69864087e-02,  -5.43931715e-02,
        -5.18641746e-02,  -4.93958173e-02,  -4.82415854e-02,
        -4.58486281e-02,  -4.35196817e-02,  -4.01162919e-02,
        -3.79466513e-02,  -3.48161871e-02,  -3.18596693e-02,
        -2.90650417e-02,  -2.64251761e-02,  -2.31429101e-02,
        -1.94312163e-02,  -1.73997964e-02,  -1.55068323e-02,
        -1.43163160e-02,  -1.31800087e-02,  -1.20987991e-02,
        -1.10708190e-02,  -1.05380016e-02,  -9.58116017e-03,
        -9.06399242e-03,  -8.54450012e-03,  -7.67847396e-03,
        -7.17608354e-03,  -6.67181154e-03,  -5.89474349e-03,
        -5.40878144e-03,  -4.92121197e-03,  -4.43202070e-03,
        -3.94148294e-03,  -3.44986011e-03,  -2.82410814e-03,
        -2.35269319e-03,  -1.88058008e-03,  -1.47393691e-03,
        -9.78376399e-04,  -4.82633521e-04,   1.33099164e-05,
         5.09212801e-04,   1.05098855e-03,   1.56929991e-03,
         2.08706303e-03,   2.72055571e-03,   3.26012954e-03,
         3.79870854e-03,   4.33573131e-03,   4.87172652e-03,
         5.40640816e-03,   5.93914581e-03,   6.47004490e-03,
         6.99921852e-03,   7.52610639e-03,   7.70592714e-03,
         8.20559501e-03,   8.70268809e-03,   9.19766855e-03,
         9.68963219e-03,   1.01781695e-02,   1.01960805e-02,
         1.06577199e-02,   1.11156340e-02,   1.15703286e-02,
         1.20215921e-02,   1.24693015e-02,   1.29129042e-02,
         1.33526781e-02,   1.37884367e-02,   1.42204360e-02,
         1.46473802e-02,   1.50699789e-02,   1.54884533e-02,
         1.59020551e-02,   1.63103362e-02,   8.12110387e-02,
         7.80794051e-02,   1.67140103e-02,   8.31537241e-02,
         7.99472912e-02,   7.99472912e-02,   7.67983984e-02,
         1.71128723e-02,   8.50656342e-02,   8.17851028e-02,
         7.85638577e-02,   7.53861405e-02,   1.75061328e-02,
         8.19411806e-02,   7.38391281e-02,   1.78939640e-02,
         8.70866930e-02,   8.36940292e-02,   8.03586974e-02,
         7.70534244e-02,   7.70534244e-02,   7.38013540e-02,
         7.38013540e-02,   7.06147796e-02,   1.82766038e-02,
         8.54279559e-02,   8.20231372e-02,   7.53294330e-02,
         7.20765174e-02,   1.86539411e-02,   8.36524496e-02,
         7.85095832e-02,   7.51592888e-02,   7.18792721e-02,
         1.90250409e-02,   7.82997201e-02,   7.49183992e-02,
         7.49183992e-02,   7.16144248e-02,   7.16144248e-02,
         6.83771846e-02,   1.93904576e-02,   7.46192919e-02,
         7.12865685e-02,   7.12865685e-02,   6.80175748e-02,
         1.97501330e-02,   7.42568965e-02,   7.08996495e-02,
         7.08996495e-02,   6.75887344e-02,   2.01042729e-02,
         7.38173451e-02,   6.70923613e-02,   2.13903228e-02,
         7.50479910e-02,   6.82108239e-02,   5.69753762e-02,
         5.24303656e-02,   5.24303656e-02,   4.52683211e-02,
         4.52683211e-02,   4.25493203e-02,   2.17470907e-02,
         7.45062992e-02,   6.76173090e-02,   6.76173090e-02,
         6.42925100e-02,   6.42925100e-02,   5.94649095e-02,
         5.94649095e-02,   3.92303424e-02,   2.20977481e-02,
         7.21341379e-02,   3.72338037e-02,   2.24415025e-02,
         7.14448972e-02,   3.40025442e-02,   2.27777176e-02,
         7.07064856e-02,   3.57533680e-02,   2.41421550e-02,
         6.81719132e-02,   3.62534788e-02,   2.44798556e-02,
         6.56110398e-02,   3.80586628e-02,   3.29287629e-02,
         2.93070471e-02,   2.48093588e-02,   6.13326924e-02,
         3.85518913e-02,   3.46206958e-02,   2.85091877e-02,
         2.51312268e-02,   5.38330011e-02,   3.76841669e-02,
         3.50540735e-02,   2.77018960e-02,   2.65615352e-02,
         5.28838088e-02,   3.81396763e-02,   3.54777506e-02,
         2.80364970e-02,   2.68822682e-02,   5.03377702e-02,
         3.85814254e-02,   3.58887890e-02,   4.93316503e-02,
         4.04098395e-02,   3.62892096e-02,   4.67615526e-02,
         4.22828625e-02,   3.80435955e-02,   3.84376145e-02,
         4.02332775e-02,   4.06156847e-02,   4.24553741e-02,
         4.43352031e-02,   4.47040511e-02,   4.66233682e-02,
         4.69790035e-02,   4.89341212e-02,   5.09256192e-02,
         5.12584867e-02,   5.32790231e-02,   5.35890744e-02,
         5.38831411e-02,   5.41625645e-02,   5.44267004e-02,
         5.46700348e-02,   5.48984863e-02,   5.51117932e-02,
         5.53082440e-02,   5.54849716e-02,   5.56464539e-02,
         5.57928396e-02,   5.59201893e-02,   5.60294455e-02,
         5.61233441e-02,   5.62020138e-02,   5.62604489e-02,
         5.63017253e-02,   5.63275468e-02,   5.63341408e-02,
         5.63226424e-02,   5.62957310e-02,   5.62533699e-02,
         5.61937444e-02,   5.61140110e-02,   5.60191106e-02,
         5.59087917e-02,   5.57801898e-02,   5.56328560e-02,
         5.54704141e-02,   5.70775198e-02,   5.68728844e-02,
         5.66515897e-02,   5.64149230e-02,   5.61622287e-02,
         5.76630266e-02,   5.73643873e-02,   5.70502787e-02,
         5.67190716e-02,   5.63668473e-02,   5.59997391e-02,
         5.73489998e-02,   5.69355151e-02,   5.65029189e-02,
         5.77751241e-02,   5.72977910e-02,   5.67990710e-02,
         5.79863269e-02,   5.74393835e-02,   5.68773454e-02,
         5.62926261e-02,   5.56922722e-02,   5.50771272e-02,
         5.44454686e-02,   5.37935810e-02,   5.31273003e-02,
         5.24468411e-02,   5.17483760e-02,   5.10330229e-02,
         5.03036776e-02,   4.95607328e-02,   4.87997085e-02,
         4.80238054e-02,   4.72347342e-02,   4.64331616e-02,
         4.56132865e-02,   4.47805574e-02,   4.39358955e-02,
         4.30782240e-02,   4.22044750e-02,   4.01052073e-02,
         3.92354976e-02,   3.83523540e-02,   3.74567873e-02,
         3.65508593e-02,   3.45751478e-02,   3.36740998e-02,
         3.27625023e-02,   3.18417381e-02,   3.09129121e-02,
         2.90665673e-02,   2.81454989e-02,   2.72171846e-02,
         2.62807950e-02,   2.53342284e-02,   2.43816409e-02,
         2.34221736e-02,   2.24541496e-02,   2.08179757e-02,
         1.98678098e-02,   1.89113740e-02,   1.79488243e-02,
         1.69806146e-02,   1.65158032e-02,   1.55075714e-02,
         1.44932106e-02,   1.34746855e-02,   1.24525920e-02,
         1.14268067e-02,   1.03968750e-02,   9.36414487e-03,
         8.58823755e-03,   7.51804527e-03,   6.44485601e-03,
         5.37002690e-03,   4.29398700e-03,   3.31511044e-03,
         2.20302298e-03,   1.09069996e-03,  -2.27320426e-05,
        -1.16892664e-03,  -2.31490869e-03,  -3.46060569e-03,
        -4.74178052e-03,  -5.91852523e-03,  -7.09360822e-03,
        -8.26683115e-03,  -9.43736653e-03,  -1.06042682e-02,
        -1.17686419e-02,  -1.33107457e-02,  -1.45010352e-02,
        -1.56869180e-02,  -1.68693838e-02,  -1.80464175e-02,
        -1.97732638e-02,  -2.09722818e-02,  -2.21650612e-02,
        -2.40185758e-02,  -2.52303300e-02,  -2.71803154e-02,
        -2.84115598e-02,  -3.04489552e-02,  -3.16936647e-02,
        -3.29299358e-02,  -3.50861051e-02,  -3.63332401e-02,
        -3.85745058e-02,  -3.98348648e-02,  -4.21660006e-02,
        -4.34302610e-02,  -4.46836493e-02,  -4.59254575e-02,
        -4.71530952e-02,  -4.96209305e-02,  -4.95594200e-02,
        -5.07435074e-02,  -5.19101301e-02,  -5.16977894e-02,
        -5.14280802e-02,  -5.11057669e-02,  -5.07251169e-02,
        -5.16985297e-02,  -5.12126585e-02,  -5.06852098e-02,
        -5.15589749e-02,  -5.09397027e-02,  -5.17615499e-02,
        -5.10672514e-02,  -5.18313966e-02,  -5.25816754e-02,
        -5.33179227e-02,  -5.40360028e-02,  -5.47358953e-02,
        -5.54213064e-02,  -5.77400978e-02,  -5.84092053e-02,
        -5.90603644e-02,  -6.14284845e-02,  -6.38379284e-02,
        -6.62872262e-02,  -6.69166162e-02,  -6.93865431e-02,
        -7.18947674e-02,  -7.44284962e-02,  -7.69969804e-02,
        -7.96063191e-02,  -8.01834105e-02,  -8.28053535e-02,
        -8.54623715e-02,  -8.59961071e-02,  -8.86660185e-02,
        -8.91520913e-02,  -9.18335218e-02,  -9.45402708e-02,
        -9.49610563e-02,  -9.76401856e-02,  -1.00332460e-01,
        -1.03032191e-01,  -1.03358935e-01,  -1.06040606e-01,
        -1.06322470e-01,  -1.08984284e-01,  -1.09195131e-01,
        -1.11833426e-01,  -1.11994247e-01,  -1.14596404e-01,
        -1.17192554e-01,  -1.17248317e-01])

yp = np.array([ -3.90948536e-05,  -2.12984775e-03,  -4.31095583e-03,
        -6.58019633e-03,  -8.93758156e-03,  -1.11568100e-02,
        -1.36444162e-02,  -1.62222092e-02,  -1.88895170e-02,
        -2.16446498e-02,  -2.49629308e-02,  -2.79508857e-02,
        -3.16029501e-02,  -3.54376380e-02,  -3.87881494e-02,
        -4.22310942e-02,  -4.41873802e-02,  -4.85246067e-02,
        -4.68663315e-02,  -4.60459599e-02,  -4.86676408e-02,
        -5.12750434e-02,  -5.38586293e-02,  -5.54310799e-02,
        -5.79452426e-02,  -5.93547929e-02,  -6.06497762e-02,
        -6.18505946e-02,  -6.29584706e-02,  -6.39609234e-02,
        -6.48713094e-02,  -6.44090476e-02,  -6.51181556e-02,
        -6.57260659e-02,  -6.62541381e-02,  -6.52943568e-02,
        -6.56184758e-02,  -6.58578685e-02,  -6.60229010e-02,
        -6.76012689e-02,  -6.76366183e-02,  -6.76004442e-02,
        -6.74972483e-02,  -6.73282385e-02,  -6.86657097e-02,
        -6.83738036e-02,  -6.80140059e-02,  -6.92366190e-02,
        -6.87491258e-02,  -6.82071471e-02,  -6.76134579e-02,
        -6.86669494e-02,  -6.79695621e-02,  -6.72259327e-02,
        -6.64391135e-02,  -6.56069234e-02,  -6.64563885e-02,
        -6.55361171e-02,  -6.45783892e-02,  -6.18312378e-02,
        -6.07850085e-02,  -5.80009440e-02,  -5.52383021e-02,
        -5.24888121e-02,  -4.97523554e-02,  -4.54714570e-02,
        -3.98863362e-02,  -3.73592876e-02,  -3.48720213e-02,
        -3.37707235e-02,  -3.26655171e-02,  -3.15625118e-02,
        -3.04616664e-02,  -3.06508019e-02,  -2.95344258e-02,
        -2.96968330e-02,  -2.98505905e-02,  -2.87101259e-02,
        -2.88391064e-02,  -2.89597166e-02,  -2.77967360e-02,
        -2.78958771e-02,  -2.79854740e-02,  -2.80670276e-02,
        -2.81405467e-02,  -2.82051366e-02,  -2.69913041e-02,
        -2.70365186e-02,  -2.70739448e-02,  -2.83768113e-02,
        -2.83979671e-02,  -2.84108899e-02,  -2.84155794e-02,
        -2.84104617e-02,  -2.96993141e-02,  -2.96767995e-02,
        -2.96453017e-02,  -3.09305120e-02,  -3.08782748e-02,
        -3.08172540e-02,  -3.07460634e-02,  -3.06652277e-02,
        -3.05756546e-02,  -3.04773301e-02,  -3.03684498e-02,
        -3.02505329e-02,  -3.01240628e-02,  -2.87032761e-02,
        -2.85638294e-02,  -2.84161924e-02,  -2.82602014e-02,
        -2.80957411e-02,  -2.79220043e-02,  -2.65224371e-02,
        -2.63408455e-02,  -2.61506690e-02,  -2.59523304e-02,
        -2.57465736e-02,  -2.55333569e-02,  -2.53114227e-02,
        -2.50819674e-02,  -2.48453976e-02,  -2.46014650e-02,
        -2.43490672e-02,  -2.40896946e-02,  -2.38232320e-02,
        -2.35495727e-02,  -2.32681400e-02,  -1.11708561e-01,
        -1.07398522e-01,  -2.29799277e-02,  -1.10281290e-01,
        -1.06025945e-01,  -1.06025945e-01,  -1.01847844e-01,
        -2.26850806e-02,  -1.08812919e-01,  -1.04614895e-01,
        -1.00492396e-01,  -9.64256156e-02,  -2.23830803e-02,
        -1.01124594e-01,  -9.11212826e-02,  -2.20738630e-02,
        -1.03723227e-01,  -9.96804013e-02,  -9.57062055e-02,
        -9.17682599e-02,  -9.17682599e-02,  -8.78935733e-02,
        -8.78935733e-02,  -8.40962884e-02,  -2.17583603e-02,
        -9.82127298e-02,  -9.42965108e-02,  -8.65980524e-02,
        -8.28570139e-02,  -2.14365508e-02,  -9.28460674e-02,
        -8.71354106e-02,  -8.34157663e-02,  -7.97743543e-02,
        -2.11075333e-02,  -8.39100274e-02,  -8.02849723e-02,
        -8.02849723e-02,  -7.67428202e-02,  -7.67428202e-02,
        -7.32724167e-02,  -2.07721464e-02,  -7.72159766e-02,
        -7.37663681e-02,  -7.37663681e-02,  -7.03828404e-02,
        -2.04308432e-02,  -7.42042591e-02,  -7.08482147e-02,
        -7.08482147e-02,  -6.75385453e-02,  -2.00834820e-02,
        -7.12338454e-02,  -6.47417418e-02,  -2.06352744e-02,
        -6.99333169e-02,  -6.35600774e-02,  -5.30876202e-02,
        -4.88515872e-02,  -4.88515872e-02,  -4.21763073e-02,
        -4.21763073e-02,  -3.96425097e-02,  -2.02588101e-02,
        -6.70368116e-02,  -6.08364913e-02,  -6.08364913e-02,
        -5.78440553e-02,  -5.78440553e-02,  -5.34994049e-02,
        -5.34994049e-02,  -3.52908904e-02,  -1.98763502e-02,
        -6.26583213e-02,  -3.23368135e-02,  -1.94880238e-02,
        -5.99037138e-02,  -2.85040222e-02,  -1.90931928e-02,
        -5.72132575e-02,  -2.89247783e-02,  -1.95297821e-02,
        -5.32198482e-02,  -2.82971986e-02,  -1.91058177e-02,
        -4.94013681e-02,  -2.86515116e-02,  -2.47888430e-02,
        -2.20618305e-02,  -1.86758942e-02,  -4.45232330e-02,
        -2.79827472e-02,  -2.51286391e-02,  -2.06919011e-02,
        -1.82397645e-02,  -3.76607947e-02,  -2.63609122e-02,
        -2.45208701e-02,  -1.93767971e-02,  -1.85788804e-02,
        -3.56379420e-02,  -2.56998805e-02,  -2.39058698e-02,
        -1.88908564e-02,  -1.81130913e-02,  -3.26595065e-02,
        -2.50304222e-02,  -2.32829732e-02,  -3.07966353e-02,
        -2.52257065e-02,  -2.26527986e-02,  -2.80693713e-02,
        -2.53799880e-02,  -2.28350066e-02,  -2.21686432e-02,
        -2.22782703e-02,  -2.15723084e-02,  -2.16081542e-02,
        -2.15998200e-02,  -2.08220272e-02,  -2.07341864e-02,
        -1.99180705e-02,  -1.97463091e-02,  -1.95241512e-02,
        -1.86330762e-02,  -1.83210810e-02,  -1.73881714e-02,
        -1.64501676e-02,  -1.55073488e-02,  -1.45603397e-02,
        -1.36076891e-02,  -1.26514336e-02,  -1.16918550e-02,
        -1.07281971e-02,  -9.76103257e-03,  -8.79150351e-03,
        -7.81935696e-03,  -6.84417527e-03,  -5.86703766e-03,
        -4.88857954e-03,  -3.90851347e-03,  -2.92690669e-03,
        -1.94445885e-03,  -9.62077293e-04,   2.10973681e-05,
         1.00443470e-03,   1.98670872e-03,   2.96920518e-03,
         3.95065293e-03,   4.93054490e-03,   5.90896238e-03,
         6.88594418e-03,   7.86095305e-03,   8.83291761e-03,
         9.80227952e-03,   1.11168744e-02,   1.21109612e-02,
         1.31014370e-02,   1.40884671e-02,   1.50714343e-02,
         1.65579859e-02,   1.75619959e-02,   1.85609524e-02,
         1.95539892e-02,   2.05406220e-02,   2.15208623e-02,
         2.31958067e-02,   2.41936890e-02,   2.51825785e-02,
         2.69676402e-02,   2.79735240e-02,   2.89676199e-02,
         3.08600313e-02,   3.18685118e-02,   3.28673845e-02,
         3.38531747e-02,   3.48305552e-02,   3.57981735e-02,
         3.67545357e-02,   3.76978426e-02,   3.86308181e-02,
         3.95533112e-02,   4.04626970e-02,   4.13583593e-02,
         4.22429533e-02,   4.31163338e-02,   4.39732984e-02,
         4.48174616e-02,   4.56497573e-02,   4.64690781e-02,
         4.72699006e-02,   4.80584575e-02,   4.88339015e-02,
         4.95941309e-02,   5.03364921e-02,   4.95646923e-02,
         5.02584615e-02,   5.09357803e-02,   5.15956682e-02,
         5.22416815e-02,   5.13017754e-02,   5.18954788e-02,
         5.24741267e-02,   5.30389590e-02,   5.35886852e-02,
         5.24828002e-02,   5.29815950e-02,   5.34658214e-02,
         5.39333680e-02,   5.43819816e-02,   5.48154596e-02,
         5.52339801e-02,   5.56342267e-02,   5.42908141e-02,
         5.46458325e-02,   5.49864909e-02,   5.53070690e-02,
         5.56106186e-02,   5.76746395e-02,   5.79561804e-02,
         5.82151857e-02,   5.84584712e-02,   5.86858866e-02,
         5.88950787e-02,   5.90831689e-02,   5.92552324e-02,
         6.12619766e-02,   6.14026109e-02,   6.15224608e-02,
         6.16256880e-02,   6.17123394e-02,   6.36720486e-02,
         6.37186812e-02,   6.37481408e-02,   6.56861133e-02,
         6.56722393e-02,   6.56407664e-02,   6.55917721e-02,
         6.74743714e-02,   6.73786194e-02,   6.72646677e-02,
         6.71325153e-02,   6.69773741e-02,   6.68003322e-02,
         6.66053297e-02,   6.83473413e-02,   6.81027202e-02,
         6.78376164e-02,   6.75542903e-02,   6.72524243e-02,
         6.88634515e-02,   6.85066915e-02,   6.81318613e-02,
         6.96716731e-02,   6.92387957e-02,   7.07292209e-02,
         7.02467261e-02,   7.16584645e-02,   7.11134550e-02,
         7.05499862e-02,   7.18681370e-02,   7.12419450e-02,
         7.24822144e-02,   7.17989089e-02,   7.29694293e-02,
         7.22180480e-02,   7.14476517e-02,   7.06588875e-02,
         6.98486903e-02,   7.08078412e-02,   6.81567317e-02,
         6.72843393e-02,   6.63881936e-02,   6.37899997e-02,
         6.12404950e-02,   5.87433383e-02,   5.62902969e-02,
         5.53950962e-02,   5.29895052e-02,   5.06437557e-02,
         4.97490264e-02,   4.74631181e-02,   4.65678359e-02,
         4.43551128e-02,   4.34554011e-02,   4.25440351e-02,
         4.16213883e-02,   4.06842153e-02,   3.97338457e-02,
         3.87727819e-02,   3.89122376e-02,   3.78978623e-02,
         3.68719281e-02,   3.68766567e-02,   3.68230044e-02,
         3.67095055e-02,   3.55465346e-02,   3.53200609e-02,
         3.50311849e-02,   3.46717730e-02,   3.42461153e-02,
         3.37555022e-02,   3.23610029e-02,   3.17505933e-02,
         3.10701527e-02,   2.95754797e-02,   2.87735213e-02,
         2.72210019e-02,   2.62970023e-02,   2.52956273e-02,
         2.36404433e-02,   2.25053642e-02,   2.12889860e-02,
         1.99902757e-02,   1.81872330e-02,   1.67574555e-02,
         1.49054892e-02,   1.33429656e-02,   1.14391250e-02,
         9.74643800e-03,   7.79267351e-03,   5.96714375e-03,
         4.05355227e-03,   2.00672241e-03])
    # -----------------------------------------------------------------------------
# Use scipy to interpolate.
coord = np.c_[xp, yp]
xp = []
yp = []
for each in coord:
    xp.append(each[0])
    yp.append(each[1])
xp = np.array(xp)
yp = np.array(yp)
okay = np.where(np.abs(np.diff(xp)) + np.abs(np.diff(yp)) > 0)
print(xp.shape)
xp = np.r_[xp, xp[-1], xp[0]]
yp = np.r_[yp, yp[-1], yp[0]]
jump = np.sqrt(np.diff(xp)**2 + np.diff(yp)**2) 
smooth_jump = gaussian_filter1d(jump, 5, mode='wrap')  # window of size 5 is arbitrary
limit = 2*np.median(smooth_jump)    # factor 2 is arbitrary
xn, yn = xp[:-1], yp[:-1]
xn = xn[(jump > 0) & (smooth_jump < limit)]
yn = yn[(jump > 0) & (smooth_jump < limit)]
tck, u = interpolate.splprep([xn, yn], s=0, k=3, per=True)
xi, yi = interpolate.splev(np.linspace(0, 1, 1000), tck)

# -----------------------------------------------------------------------------
# Plot result
fig = plt.figure()
ax = plt.subplot(111)
ax.plot(xp, yp, 'o', markersize=2)
ax.plot(xi, yi, 'r', alpha=0.5)

plt.show()
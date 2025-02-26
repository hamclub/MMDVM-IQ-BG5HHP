/*
 *   Copyright (C) 2024 by Jonathan Naylor G4KLX
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include "arm_math.h"

#define FAST_MATH_TABLE_SIZE  512
#define FAST_MATH_Q31_SHIFT   (32 - 10)
#define FAST_MATH_Q15_SHIFT   (16 - 10)

static const q15_t sinTable_q15[FAST_MATH_TABLE_SIZE + 1] = {
    0, 402, 804, 1206, 1608, 2009, 2411, 2811, 3212, 3612, 4011, 4410, 4808,
    5205, 5602, 5998, 6393, 6787, 7180, 7571, 7962, 8351, 8740, 9127, 9512,
    9896, 10279, 10660, 11039, 11417, 11793, 12167, 12540, 12910, 13279,
    13646, 14010, 14373, 14733, 15091, 15447, 15800, 16151, 16500, 16846,
    17190, 17531, 17869, 18205, 18538, 18868, 19195, 19520, 19841, 20160,
    20475, 20788, 21097, 21403, 21706, 22006, 22302, 22595, 22884, 23170,
    23453, 23732, 24008, 24279, 24548, 24812, 25073, 25330, 25583, 25833,
    26078, 26320, 26557, 26791, 27020, 27246, 27467, 27684, 27897, 28106,
    28311, 28511, 28707, 28899, 29086, 29269, 29448, 29622, 29792, 29957,
    30118, 30274, 30425, 30572, 30715, 30853, 30986, 31114, 31238, 31357,
    31471, 31581, 31686, 31786, 31881, 31972, 32058, 32138, 32214, 32286,
    32352, 32413, 32470, 32522, 32568, 32610, 32647, 32679, 32706, 32729,
    32746, 32758, 32766, 32767, 32766, 32758, 32746, 32729, 32706, 32679,
    32647, 32610, 32568, 32522, 32470, 32413, 32352, 32286, 32214, 32138,
    32058, 31972, 31881, 31786, 31686, 31581, 31471, 31357, 31238, 31114,
    30986, 30853, 30715, 30572, 30425, 30274, 30118, 29957, 29792, 29622,
    29448, 29269, 29086, 28899, 28707, 28511, 28311, 28106, 27897, 27684,
    27467, 27246, 27020, 26791, 26557, 26320, 26078, 25833, 25583, 25330,
    25073, 24812, 24548, 24279, 24008, 23732, 23453, 23170, 22884, 22595,
    22302, 22006, 21706, 21403, 21097, 20788, 20475, 20160, 19841, 19520,
    19195, 18868, 18538, 18205, 17869, 17531, 17190, 16846, 16500, 16151,
    15800, 15447, 15091, 14733, 14373, 14010, 13646, 13279, 12910, 12540,
    12167, 11793, 11417, 11039, 10660, 10279, 9896, 9512, 9127, 8740, 8351,
    7962, 7571, 7180, 6787, 6393, 5998, 5602, 5205, 4808, 4410, 4011, 3612,
    3212, 2811, 2411, 2009, 1608, 1206, 804, 402, 0, -402, -804, -1206,
    -1608, -2009, -2411, -2811, -3212, -3612, -4011, -4410, -4808, -5205,
    -5602, -5998, -6393, -6787, -7180, -7571, -7962, -8351, -8740, -9127,
    -9512, -9896, -10279, -10660, -11039, -11417, -11793, -12167, -12540,
    -12910, -13279, -13646, -14010, -14373, -14733, -15091, -15447, -15800,
    -16151, -16500, -16846, -17190, -17531, -17869, -18205, -18538, -18868,
    -19195, -19520, -19841, -20160, -20475, -20788, -21097, -21403, -21706,
    -22006, -22302, -22595, -22884, -23170, -23453, -23732, -24008, -24279,
    -24548, -24812, -25073, -25330, -25583, -25833, -26078, -26320, -26557,
    -26791, -27020, -27246, -27467, -27684, -27897, -28106, -28311, -28511,
    -28707, -28899, -29086, -29269, -29448, -29622, -29792, -29957, -30118,
    -30274, -30425, -30572, -30715, -30853, -30986, -31114, -31238, -31357,
    -31471, -31581, -31686, -31786, -31881, -31972, -32058, -32138, -32214,
    -32286, -32352, -32413, -32470, -32522, -32568, -32610, -32647, -32679,
    -32706, -32729, -32746, -32758, -32766, -32768, -32766, -32758, -32746,
    -32729, -32706, -32679, -32647, -32610, -32568, -32522, -32470, -32413,
    -32352, -32286, -32214, -32138, -32058, -31972, -31881, -31786, -31686,
    -31581, -31471, -31357, -31238, -31114, -30986, -30853, -30715, -30572,
    -30425, -30274, -30118, -29957, -29792, -29622, -29448, -29269, -29086,
    -28899, -28707, -28511, -28311, -28106, -27897, -27684, -27467, -27246,
    -27020, -26791, -26557, -26320, -26078, -25833, -25583, -25330, -25073,
    -24812, -24548, -24279, -24008, -23732, -23453, -23170, -22884, -22595,
    -22302, -22006, -21706, -21403, -21097, -20788, -20475, -20160, -19841,
    -19520, -19195, -18868, -18538, -18205, -17869, -17531, -17190, -16846,
    -16500, -16151, -15800, -15447, -15091, -14733, -14373, -14010, -13646,
    -13279, -12910, -12540, -12167, -11793, -11417, -11039, -10660, -10279,
    -9896, -9512, -9127, -8740, -8351, -7962, -7571, -7180, -6787, -6393,
    -5998, -5602, -5205, -4808, -4410, -4011, -3612, -3212, -2811, -2411,
    -2009, -1608, -1206, -804, -402, 0
};

static const q31_t sinTable_q31[FAST_MATH_TABLE_SIZE + 1] = {
	0L, 26352928L, 52701887L, 79042909L, 105372028L, 131685278L, 157978697L,
	184248325L, 210490206L, 236700388L, 262874923L, 289009871L, 315101295L,
	341145265L, 367137861L, 393075166L, 418953276L, 444768294L, 470516330L,
	496193509L, 521795963L, 547319836L, 572761285L, 598116479L, 623381598L,
	648552838L, 673626408L, 698598533L, 723465451L, 748223418L, 772868706L,
	797397602L, 821806413L, 846091463L, 870249095L, 894275671L, 918167572L,
	941921200L, 965532978L, 988999351L, 1012316784L, 1035481766L, 1058490808L,
	1081340445L, 1104027237L, 1126547765L, 1148898640L, 1171076495L, 1193077991L,
	1214899813L, 1236538675L, 1257991320L, 1279254516L, 1300325060L, 1321199781L,
	1341875533L, 1362349204L, 1382617710L, 1402678000L, 1422527051L, 1442161874L,
	1461579514L, 1480777044L, 1499751576L, 1518500250L, 1537020244L, 1555308768L,
	1573363068L, 1591180426L, 1608758157L, 1626093616L, 1643184191L, 1660027308L,
	1676620432L, 1692961062L, 1709046739L, 1724875040L, 1740443581L, 1755750017L,
	1770792044L, 1785567396L, 1800073849L, 1814309216L, 1828271356L, 1841958164L,
	1855367581L, 1868497586L, 1881346202L, 1893911494L, 1906191570L, 1918184581L,
	1929888720L, 1941302225L, 1952423377L, 1963250501L, 1973781967L, 1984016189L,
	1993951625L, 2003586779L, 2012920201L, 2021950484L, 2030676269L, 2039096241L,
	2047209133L, 2055013723L, 2062508835L, 2069693342L, 2076566160L, 2083126254L,
	2089372638L, 2095304370L, 2100920556L, 2106220352L, 2111202959L, 2115867626L,
	2120213651L, 2124240380L, 2127947206L, 2131333572L, 2134398966L, 2137142927L,
	2139565043L, 2141664948L, 2143442326L, 2144896910L, 2146028480L, 2146836866L,
	2147321946L, 2147483647L, 2147321946L, 2146836866L, 2146028480L, 2144896910L,
	2143442326L, 2141664948L, 2139565043L, 2137142927L, 2134398966L, 2131333572L,
	2127947206L, 2124240380L, 2120213651L, 2115867626L, 2111202959L, 2106220352L,
	2100920556L, 2095304370L, 2089372638L, 2083126254L, 2076566160L, 2069693342L,
	2062508835L, 2055013723L, 2047209133L, 2039096241L, 2030676269L, 2021950484L,
	2012920201L, 2003586779L, 1993951625L, 1984016189L, 1973781967L, 1963250501L,
	1952423377L, 1941302225L, 1929888720L, 1918184581L, 1906191570L, 1893911494L,
	1881346202L, 1868497586L, 1855367581L, 1841958164L, 1828271356L, 1814309216L,
	1800073849L, 1785567396L, 1770792044L, 1755750017L, 1740443581L, 1724875040L,
	1709046739L, 1692961062L, 1676620432L, 1660027308L, 1643184191L, 1626093616L,
	1608758157L, 1591180426L, 1573363068L, 1555308768L, 1537020244L, 1518500250L,
	1499751576L, 1480777044L, 1461579514L, 1442161874L, 1422527051L, 1402678000L,
	1382617710L, 1362349204L, 1341875533L, 1321199781L, 1300325060L, 1279254516L,
	1257991320L, 1236538675L, 1214899813L, 1193077991L, 1171076495L, 1148898640L,
	1126547765L, 1104027237L, 1081340445L, 1058490808L, 1035481766L, 1012316784L,
	988999351L, 965532978L, 941921200L, 918167572L, 894275671L, 870249095L,
	846091463L, 821806413L, 797397602L, 772868706L, 748223418L, 723465451L,
	698598533L, 673626408L, 648552838L, 623381598L, 598116479L, 572761285L,
	547319836L, 521795963L, 496193509L, 470516330L, 444768294L, 418953276L,
	393075166L, 367137861L, 341145265L, 315101295L, 289009871L, 262874923L,
	236700388L, 210490206L, 184248325L, 157978697L, 131685278L, 105372028L,
	79042909L, 52701887L, 26352928L, 0L, -26352928L, -52701887L, -79042909L,
	-105372028L, -131685278L, -157978697L, -184248325L, -210490206L, -236700388L,
	-262874923L, -289009871L, -315101295L, -341145265L, -367137861L, -393075166L,
	-418953276L, -444768294L, -470516330L, -496193509L, -521795963L, -547319836L,
	-572761285L, -598116479L, -623381598L, -648552838L, -673626408L, -698598533L,
	-723465451L, -748223418L, -772868706L, -797397602L, -821806413L, -846091463L,
	-870249095L, -894275671L, -918167572L, -941921200L, -965532978L, -988999351L,
	-1012316784L, -1035481766L, -1058490808L, -1081340445L, -1104027237L,
	-1126547765L, -1148898640L, -1171076495L, -1193077991L, -1214899813L,
	-1236538675L, -1257991320L, -1279254516L, -1300325060L, -1321199781L,
	-1341875533L, -1362349204L, -1382617710L, -1402678000L, -1422527051L,
	-1442161874L, -1461579514L, -1480777044L, -1499751576L, -1518500250L,
	-1537020244L, -1555308768L, -1573363068L, -1591180426L, -1608758157L,
	-1626093616L, -1643184191L, -1660027308L, -1676620432L, -1692961062L,
	-1709046739L, -1724875040L, -1740443581L, -1755750017L, -1770792044L,
	-1785567396L, -1800073849L, -1814309216L, -1828271356L, -1841958164L,
	-1855367581L, -1868497586L, -1881346202L, -1893911494L, -1906191570L,
	-1918184581L, -1929888720L, -1941302225L, -1952423377L, -1963250501L,
	-1973781967L, -1984016189L, -1993951625L, -2003586779L, -2012920201L,
	-2021950484L, -2030676269L, -2039096241L, -2047209133L, -2055013723L,
	-2062508835L, -2069693342L, -2076566160L, -2083126254L, -2089372638L,
	-2095304370L, -2100920556L, -2106220352L, -2111202959L, -2115867626L,
	-2120213651L, -2124240380L, -2127947206L, -2131333572L, -2134398966L,
	-2137142927L, -2139565043L, -2141664948L, -2143442326L, -2144896910L,
	-2146028480L, -2146836866L, -2147321946L, (q31_t)0x80000000, -2147321946L,
	-2146836866L, -2146028480L, -2144896910L, -2143442326L, -2141664948L,
	-2139565043L, -2137142927L, -2134398966L, -2131333572L, -2127947206L,
	-2124240380L, -2120213651L, -2115867626L, -2111202959L, -2106220352L,
	-2100920556L, -2095304370L, -2089372638L, -2083126254L, -2076566160L,
	-2069693342L, -2062508835L, -2055013723L, -2047209133L, -2039096241L,
	-2030676269L, -2021950484L, -2012920201L, -2003586779L, -1993951625L,
	-1984016189L, -1973781967L, -1963250501L, -1952423377L, -1941302225L,
	-1929888720L, -1918184581L, -1906191570L, -1893911494L, -1881346202L,
	-1868497586L, -1855367581L, -1841958164L, -1828271356L, -1814309216L,
	-1800073849L, -1785567396L, -1770792044L, -1755750017L, -1740443581L,
	-1724875040L, -1709046739L, -1692961062L, -1676620432L, -1660027308L,
	-1643184191L, -1626093616L, -1608758157L, -1591180426L, -1573363068L,
	-1555308768L, -1537020244L, -1518500250L, -1499751576L, -1480777044L,
	-1461579514L, -1442161874L, -1422527051L, -1402678000L, -1382617710L,
	-1362349204L, -1341875533L, -1321199781L, -1300325060L, -1279254516L,
	-1257991320L, -1236538675L, -1214899813L, -1193077991L, -1171076495L,
	-1148898640L, -1126547765L, -1104027237L, -1081340445L, -1058490808L,
	-1035481766L, -1012316784L, -988999351L, -965532978L, -941921200L,
	-918167572L, -894275671L, -870249095L, -846091463L, -821806413L, -797397602L,
	-772868706L, -748223418L, -723465451L, -698598533L, -673626408L, -648552838L,
	-623381598L, -598116479L, -572761285L, -547319836L, -521795963L, -496193509L,
	-470516330L, -444768294L, -418953276L, -393075166L, -367137861L, -341145265L,
	-315101295L, -289009871L, -262874923L, -236700388L, -210490206L, -184248325L,
	-157978697L, -131685278L, -105372028L, -79042909L, -52701887L, -26352928L, 0
};

/* Q2.29 */
#define ATANHALF_Q29 0xed63383
#define PIHALF_Q29 0x3243f6a9
#define PIQ29 0x6487ed51

#define ATAN2_NB_COEFS_Q31 13

#define Q28QUARTER 0x20000000 

static const q31_t atan2_coefs_q31[ATAN2_NB_COEFS_Q31] = { 0x00000000
,0x7ffffffe
,0x000001b6
,0xd555158e
,0x00036463
,0x1985f617
,0x001992ae
,0xeed53a7f
,0xf8f15245
,0x2215a3a4
,0xe0fab004
,0x0cdd4825
,0xfddbc054
};

static const q31_t sqrt_initial_lut_q31[32] = { 536870912, 506166750, 480191942, 457845052, 438353264, 421156193, \
405836263, 392075079, 379625062, 368290407, 357913941, 348367849, \
339546978, 331363921, 323745341, 316629190, 309962566, 303700050, \
297802400, 292235509, 286969573, 281978417, 277238947, 272730696, \
268435456, 264336964, 260420644, 256673389, 253083375, 249639903, \
246333269, 243154642};

void arm_q15_to_q31(const q15_t* pSrc, q31_t* pDst, uint32_t blockSize)
{
    uint32_t blkCnt;                               /* Loop counter */
    const q15_t* pIn = pSrc;                             /* Source pointer */

    /* Initialize blkCnt with number of samples */
    blkCnt = blockSize;

    while (blkCnt > 0U)
    {
        /* C = (q31_t) A << 16 */

        /* Convert from q15 to q31 and store result in destination buffer */
        *pDst++ = (q31_t)*pIn++ << 16;

        /* Decrement loop counter */
        blkCnt--;
    }
}

q15_t arm_sin_q15(q15_t x)
{
    q15_t sinVal;                                  /* Temporary variables for input, output */
    int32_t index;                                 /* Index variables */
    q15_t a, b;                                    /* Four nearest output values */
    q15_t fract;                                   /* Temporary values for fractional values */

    /* Calculate the nearest index */
    index = (uint32_t)x >> FAST_MATH_Q15_SHIFT;

    /* Calculation of fractional value */
    fract = (x - (index << FAST_MATH_Q15_SHIFT)) << 9;

    /* Read two nearest values of input value from the sin table */
    a = sinTable_q15[index];
    b = sinTable_q15[index + 1];

    /* Linear interpolation process */
    sinVal = (q31_t)(0x8000 - fract) * a >> 16;
    sinVal = (q15_t)((((q31_t)sinVal << 16) + ((q31_t)fract * b)) >> 16);

    return sinVal << 1;
}

q15_t arm_cos_q15(q15_t x)
{
    q15_t cosVal;                                  /* Temporary variables for input, output */
    int32_t index;                                 /* Index variables */
    q15_t a, b;                                    /* Four nearest output values */
    q15_t fract;                                   /* Temporary values for fractional values */

    /* add 0.25 (pi/2) to read sine table */
    x = (uint16_t)x + 0x2000;
    if (x < 0)
    {   /* convert negative numbers to corresponding positive ones */
        x = (uint16_t)x + 0x8000;
    }

    /* Calculate the nearest index */
    index = (uint32_t)x >> FAST_MATH_Q15_SHIFT;

    /* Calculation of fractional value */
    fract = (x - (index << FAST_MATH_Q15_SHIFT)) << 9;

    /* Read two nearest values of input value from the sin table */
    a = sinTable_q15[index];
    b = sinTable_q15[index + 1];

    /* Linear interpolation process */
    cosVal = (q31_t)(0x8000 - fract) * a >> 16;
    cosVal = (q15_t)((((q31_t)cosVal << 16) + ((q31_t)fract * b)) >> 16);

    return cosVal << 1;
}

q31_t arm_sin_q31(q31_t x)
{
	q31_t sinVal;                                  /* Temporary variables for input, output */
	int32_t index;                                 /* Index variable */
	q31_t a, b;                                    /* Two nearest output values */
	q31_t fract;                                   /* Temporary values for fractional values */

	if (x < 0) {
		/* convert negative numbers to corresponding positive ones */
		x = (uint32_t)x + 0x80000000;
	}

	/* Calculate the nearest index */
	index = (uint32_t)x >> FAST_MATH_Q31_SHIFT;

	/* Calculation of fractional value */
	fract = (x - (index << FAST_MATH_Q31_SHIFT)) << 9;

	/* Read two nearest values of input value from the sin table */
	a = sinTable_q31[index];
	b = sinTable_q31[index + 1];

	/* Linear interpolation process */
	sinVal = (q63_t)(0x80000000 - fract) * a >> 32;
	sinVal = (q31_t)((((q63_t)sinVal << 32) + ((q63_t)fract * b)) >> 32);

	/* Return output value */
	return (sinVal << 1);
}

q31_t arm_cos_q31(q31_t x)
{
    q31_t cosVal;                                  /* Temporary variables for input, output */
    int32_t index;                                 /* Index variables */
    q31_t a, b;                                    /* Four nearest output values */
    q31_t fract;                                   /* Temporary values for fractional values */

    /* add 0.25 (pi/2) to read sine table */
    x = (uint32_t)x + 0x20000000;
    if (x < 0)
    {   /* convert negative numbers to corresponding positive ones */
        x = (uint32_t)x + 0x80000000;
    }

    /* Calculate the nearest index */
    index = (uint32_t)x >> FAST_MATH_Q31_SHIFT;

    /* Calculation of fractional value */
    fract = (x - (index << FAST_MATH_Q31_SHIFT)) << 9;

    /* Read two nearest values of input value from the sin table */
    a = sinTable_q31[index];
    b = sinTable_q31[index + 1];

    /* Linear interpolation process */
    cosVal = (q63_t)(0x80000000 - fract) * a >> 32;
    cosVal = (q31_t)((((q63_t)cosVal << 32) + ((q63_t)fract * b)) >> 32);

    return cosVal << 1;
}

static uint8_t __CLZ(uint32_t data)
{
    if (data == 0U)
        return 32U;

    uint32_t count = 0U;
    uint32_t mask = 0x80000000U;

    while ((data & mask) == 0U) {
        count += 1U;
        mask = mask >> 1U;
    }

    return count;
}

int32_t __SSAT(int32_t val, uint32_t sat)
{
    if ((sat >= 1U) && (sat <= 32U)) {
        const int32_t max = (int32_t)((1U << (sat - 1U)) - 1U);
        const int32_t min = -1 - max;

        if (val > max)
            return max;
        else if (val < min)
            return min;
    }

    return val;
}

static q31_t clip_q63_to_q31(q63_t x)
{
    return ((q31_t)(x >> 32) != ((q31_t)x >> 31)) ?
        ((0x7FFFFFFF ^ ((q31_t)(x >> 63)))) : (q31_t)x;
}

void arm_abs_q31(const q31_t* pSrc, q31_t* pDst, uint32_t blockSize)
{
    uint32_t blkCnt;                               /* Loop counter */
    q31_t in;                                      /* Temporary variable */

  /* Initialize blkCnt with number of samples */
    blkCnt = blockSize;

    while (blkCnt > 0U) {
        /* C = |A| */

        /* Calculate absolute of input (if -1 then saturated to 0x7fffffff) and store result in destination buffer. */
        in = *pSrc++;
        *pDst++ = (in > 0) ? in : ((in == INT32_MIN) ? INT32_MAX : -in);

        /* Decrement loop counter */
        blkCnt--;
    }
}

static void arm_divide_q31(q31_t numerator, q31_t denominator, q31_t* quotient, int16_t* shift)
{
    int16_t sign = 0;
    q63_t temp;
    int16_t shiftForNormalizing;

    *shift = 0;

    sign = (numerator < 0) ^ (denominator < 0);

    if (denominator == 0) {
        if (sign)
            *quotient = 0x80000000;
        else
            *quotient = 0x7FFFFFFF;

        return;
    }

    arm_abs_q31(&numerator, &numerator, 1);
    arm_abs_q31(&denominator, &denominator, 1);

    temp = ((q63_t)numerator << 31) / ((q63_t)denominator);

    shiftForNormalizing = 32 - __CLZ(temp >> 31);
    if (shiftForNormalizing > 0) {
        *shift = shiftForNormalizing;
        temp = temp >> shiftForNormalizing;
    }

    if (sign)
        temp = -temp;

    *quotient = (q31_t)temp;
}

static q31_t arm_atan_limited_q31(q31_t x)
{
    q63_t res = (q63_t)atan2_coefs_q31[ATAN2_NB_COEFS_Q31 - 1];
    int i = 1;
    for (i = 1; i < ATAN2_NB_COEFS_Q31; i++) {
        res = ((q63_t)x * res) >> 31U;
        res = res + ((q63_t)atan2_coefs_q31[ATAN2_NB_COEFS_Q31 - 1 - i]);
    }

    return clip_q63_to_q31(res >> 2);
}

q31_t arm_atan_q31(q31_t y, q31_t x)
{
    int sign = 0;
    q31_t res = 0;

    if (y < 0) {
        /* Negate y */
        y = (y == INT32_MIN) ? INT32_MAX : -y;
        sign = 1 - sign;
    }

    if (x < 0) {
        sign = 1 - sign;

        /* Negate x */
        x = (x == INT32_MIN) ? INT32_MAX : -x;
    }

    if (y > x) {
        q31_t ratio;
        int16_t shift;

        arm_divide_q31(x, y, &ratio, &shift);

        /* Shift ratio by shift */
        if (shift >= 0)
            ratio = clip_q63_to_q31((q63_t)ratio << shift);
        else
            ratio = (ratio >> -shift);

        res = PIHALF_Q29 - arm_atan_limited_q31(ratio);
    } else {
        q31_t ratio;
        int16_t shift;

        arm_divide_q31(y, x, &ratio, &shift);

        /* Shift ratio by shift */
        if (shift >= 0)
            ratio = clip_q63_to_q31((q63_t)ratio << shift);
        else
            ratio = (ratio >> -shift);

        res = arm_atan_limited_q31(ratio);
    }

    if (sign) {
        /* Negate res */
        res = (res == INT32_MIN) ? INT32_MAX : -res;
    }

    return res;
}

void arm_sqrt_q31(q31_t in, q31_t* pOut)
{
    q31_t number, var1, signBits1, temp;

    number = in;

    /* If the input is a positive number then compute the signBits. */
    if (number > 0) {
        signBits1 = __CLZ(number) - 1;

        /* Shift by the number of signBits1 */
        if ((signBits1 % 2) == 0)
            number = number << signBits1;
        else
            number = number << (signBits1 - 1);

        /* Start value for 1/sqrt(x) for the Newton iteration */
        var1 = sqrt_initial_lut_q31[(number >> 26) - (Q28QUARTER >> 26)];

        /* 0.5 var1 * (3 - number * var1 * var1) */

        /* 1st iteration */

        temp = ((q63_t)var1 * var1) >> 28;
        temp = ((q63_t)number * temp) >> 31;
        temp = 0x30000000 - temp;
        var1 = ((q63_t)var1 * temp) >> 29;


        /* 2nd iteration */
        temp = ((q63_t)var1 * var1) >> 28;
        temp = ((q63_t)number * temp) >> 31;
        temp = 0x30000000 - temp;
        var1 = ((q63_t)var1 * temp) >> 29;

        /* 3nd iteration */
        temp = ((q63_t)var1 * var1) >> 28;
        temp = ((q63_t)number * temp) >> 31;
        temp = 0x30000000 - temp;
        var1 = ((q63_t)var1 * temp) >> 29;

        /* Multiply the inverse square root with the original value */
        var1 = ((q31_t)(((q63_t)number * var1) >> 28));

        /* Shift the output down accordingly */
        if ((signBits1 % 2) == 0)
            var1 = var1 >> (signBits1 / 2);
        else
            var1 = var1 >> ((signBits1 - 1) / 2);
        *pOut = var1;

        return;
    }
    /* If the number is a negative number then store zero as its square root value */
    else {
        *pOut = 0;

        return;
    }
}

void arm_fir_fast_q15(const arm_fir_instance_q15* S, const q15_t* pSrc, q15_t* pDst, uint32_t blockSize)
{
    q15_t* pState = S->pState;                     /* State pointer */
    const q15_t* pCoeffs = S->pCoeffs;                   /* Coefficient pointer */
    q15_t* pStateCurnt;                            /* Points to the current sample of the state */
    q15_t* px;                                     /* Temporary pointer for state buffer */
    const q15_t* pb;                                     /* Temporary pointer for coefficient buffer */
    q31_t acc0;                                    /* Accumulators */
    uint32_t numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
    uint32_t tapCnt, blkCnt;                       /* Loop counters */

  /* S->pState points to state array which contains previous frame (numTaps - 1) samples */
  /* pStateCurnt points to the location where the new input data should be written */
    pStateCurnt = &(S->pState[(numTaps - 1U)]);

    /* Initialize blkCnt with number of taps */
    blkCnt = blockSize;

    while (blkCnt > 0U)
    {
        /* Copy two samples into state buffer */
        *pStateCurnt++ = *pSrc++;

        /* Set the accumulator to zero */
        acc0 = 0;

        /* Use SIMD to hold states and coefficients */
        px = pState;
        pb = pCoeffs;

        tapCnt = numTaps >> 1U;

        do
        {
            acc0 += (q31_t)*px++ * *pb++;
            acc0 += (q31_t)*px++ * *pb++;

            tapCnt--;
        } while (tapCnt > 0U);

        /* The result is in 2.30 format. Convert to 1.15 with saturation.
           Then store the output in the destination buffer. */
        *pDst++ = (q15_t)(__SSAT((acc0 >> 15), 16));

        /* Advance state pointer by 1 for the next sample */
        pState = pState + 1U;

        /* Decrement loop counter */
        blkCnt--;
    }

    /* Processing is complete.
       Now copy the last numTaps - 1 samples to the start of the state buffer.
       This prepares the state buffer for the next function call. */

    /* Points to the start of the state buffer */
    pStateCurnt = S->pState;

    /* Initialize tapCnt with number of taps */
    tapCnt = (numTaps - 1U);

    /* Copy remaining data */
    while (tapCnt > 0U)
    {
        *pStateCurnt++ = *pState++;

        /* Decrement loop counter */
        tapCnt--;
    }
}

void arm_fir_interpolate_q15(const arm_fir_interpolate_instance_q15* S, const q15_t* pSrc, q15_t* pDst, uint32_t blockSize)
{
    q15_t* pState = S->pState;                     /* State pointer */
    const q15_t* pCoeffs = S->pCoeffs;                   /* Coefficient pointer */
    q15_t* pStateCur;                              /* Points to the current sample of the state */
    q15_t* ptr1;                                   /* Temporary pointer for state buffer */
    const q15_t* ptr2;                                   /* Temporary pointer for coefficient buffer */
    q63_t sum0;                                    /* Accumulators */
    uint32_t i, blkCnt, tapCnt;                    /* Loop counters */
    uint32_t phaseLen = S->phaseLength;            /* Length of each polyphase filter component */

    /* S->pState buffer contains previous frame (phaseLen - 1) samples */
    /* pStateCur points to the location where the new input data should be written */
    pStateCur = S->pState + (phaseLen - 1U);

    /* Total number of intput samples */
    blkCnt = blockSize;

    /* Loop over the blockSize. */
    while (blkCnt > 0U)
    {
        /* Copy new input sample into the state buffer */
        *pStateCur++ = *pSrc++;

        /* Loop over the Interpolation factor. */
        i = S->L;

        while (i > 0U)
        {
            /* Set accumulator to zero */
            sum0 = 0;

            /* Initialize state pointer */
            ptr1 = pState;

            /* Initialize coefficient pointer */
            ptr2 = pCoeffs + (i - 1U);

            /* Loop over the polyPhase length */
            tapCnt = phaseLen;

            while (tapCnt > 0U)
            {
                /* Perform the multiply-accumulate */
                sum0 += ((q63_t)*ptr1++ * *ptr2);

                /* Increment the coefficient pointer by interpolation factor times. */
                ptr2 += S->L;

                /* Decrement the loop counter */
                tapCnt--;
            }

            /* Store the result after converting to 1.15 format in the destination buffer. */
            *pDst++ = (q15_t)(__SSAT((sum0 >> 15), 16));

            /* Decrement loop counter */
            i--;
        }

        /* Advance the state pointer by 1
         * to process the next group of interpolation factor number samples */
        pState = pState + 1;

        /* Decrement loop counter */
        blkCnt--;
    }

    /* Processing is complete.
     ** Now copy the last phaseLen - 1 samples to the start of the state buffer.
     ** This prepares the state buffer for the next function call. */

    /* Points to the start of the state buffer */
    pStateCur = S->pState;

    tapCnt = phaseLen - 1U;

    /* Copy data */
    while (tapCnt > 0U)
    {
        *pStateCur++ = *pState++;

        /* Decrement loop counter */
        tapCnt--;
    }
}

void arm_fir_f32(const arm_fir_instance_f32* S, const float32_t* pSrc, float32_t* pDst, uint32_t blockSize)
{
    float32_t* pState = S->pState;                 /* State pointer */
    const float32_t* pCoeffs = S->pCoeffs;               /* Coefficient pointer */
    float32_t* pStateCurnt;                        /* Points to the current sample of the state */
    float32_t* px;                                 /* Temporary pointer for state buffer */
    const float32_t* pb;                                 /* Temporary pointer for coefficient buffer */
    float32_t acc0;                                /* Accumulator */
    uint32_t numTaps = S->numTaps;                 /* Number of filter coefficients in the filter */
    uint32_t i, tapCnt, blkCnt;                    /* Loop counters */

  /* S->pState points to state array which contains previous frame (numTaps - 1) samples */
  /* pStateCurnt points to the location where the new input data should be written */
    pStateCurnt = &(S->pState[(numTaps - 1U)]);

    /* Initialize blkCnt with number of taps */
    blkCnt = blockSize;

    while (blkCnt > 0U)
    {
        /* Copy one sample at a time into state buffer */
        *pStateCurnt++ = *pSrc++;

        /* Set the accumulator to zero */
        acc0 = 0.0f;

        /* Initialize state pointer */
        px = pState;

        /* Initialize Coefficient pointer */
        pb = pCoeffs;

        i = numTaps;

        /* Perform the multiply-accumulates */
        while (i > 0U)
        {
            /* acc =  b[numTaps-1] * x[n-numTaps-1] + b[numTaps-2] * x[n-numTaps-2] + b[numTaps-3] * x[n-numTaps-3] +...+ b[0] * x[0] */
            acc0 += *px++ * *pb++;

            i--;
        }

        /* Store result in destination buffer. */
        *pDst++ = acc0;

        /* Advance state pointer by 1 for the next sample */
        pState = pState + 1U;

        /* Decrement loop counter */
        blkCnt--;
    }

    /* Processing is complete.
       Now copy the last numTaps - 1 samples to the start of the state buffer.
       This prepares the state buffer for the next function call. */

       /* Points to the start of the state buffer */
    pStateCurnt = S->pState;

    /* Initialize tapCnt with number of taps */
    tapCnt = (numTaps - 1U);

    /* Copy remaining data */
    while (tapCnt > 0U)
    {
        *pStateCurnt++ = *pState++;

        /* Decrement loop counter */
        tapCnt--;
    }
}

void arm_biquad_cascade_df1_q31(const arm_biquad_casd_df1_inst_q31* S, const q31_t* pSrc, q31_t* pDst, uint32_t blockSize)
{
    const q31_t* pIn = pSrc;                             /* Source pointer */
    q31_t* pOut = pDst;                            /* Destination pointer */
    q31_t* pState = S->pState;                     /* pState pointer */
    const q31_t* pCoeffs = S->pCoeffs;                   /* Coefficient pointer */
    q63_t acc;                                     /* Accumulator */
    q31_t b0, b1, b2, a1, a2;                      /* Filter coefficients */
    q31_t Xn1, Xn2, Yn1, Yn2;                      /* Filter pState variables */
    q31_t Xn;                                      /* Temporary input */
    uint32_t uShift = ((uint32_t)S->postShift + 1U);
    uint32_t lShift = 32U - uShift;                /* Shift to be applied to the output */
    uint32_t sample, stage = S->numStages;         /* Loop counters */

    do
    {
        /* Reading the coefficients */
        b0 = *pCoeffs++;
        b1 = *pCoeffs++;
        b2 = *pCoeffs++;
        a1 = *pCoeffs++;
        a2 = *pCoeffs++;
        
        /* Reading the pState values */
        Xn1 = pState[0];
        Xn2 = pState[1];
        Yn1 = pState[2];
        Yn2 = pState[3];

        /* Initialize blkCnt with number of samples */
        sample = blockSize;

        while (sample > 0U)
        {
            /* Read the input */
            Xn = *pIn++;

            /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
            acc = ((q63_t)b0 * Xn) + ((q63_t)b1 * Xn1) + ((q63_t)b2 * Xn2) + ((q63_t)a1 * Yn1) + ((q63_t)a2 * Yn2);
           
            /* The result is converted to 1.31  */
            acc = acc >> lShift;

            /* Store output in destination buffer. */
            *pOut++ = (q31_t)acc;

            /* Every time after the output is computed state should be updated. */
            /* The states should be updated as: */
            /* Xn2 = Xn1 */
            /* Xn1 = Xn  */
            /* Yn2 = Yn1 */
            /* Yn1 = acc */
            Xn2 = Xn1;
            Xn1 = Xn;
            Yn2 = Yn1;
            Yn1 = (q31_t)acc;

            /* decrement loop counter */
            sample--;
        }

        /* Store the updated state variables back into the pState array */
        *pState++ = Xn1;
        *pState++ = Xn2;
        *pState++ = Yn1;
        *pState++ = Yn2;

        /* The first stage goes from the input buffer to the output buffer. */
        /* Subsequent numStages occur in-place in the output buffer */
        pIn = pDst;

        /* Reset output pointer */
        pOut = pDst;

        /* decrement loop counter */
        stage--;

    } while (stage > 0U);
}

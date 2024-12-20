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

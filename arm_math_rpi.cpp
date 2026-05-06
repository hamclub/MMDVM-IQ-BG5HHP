/**
 * @brief ARM DSP Compatible layer for RPI build
 */

/* ----------------------------------------------------------------------    
* Copyright (C) 2010-2014 ARM Limited. All rights reserved.    
*    
* $Date:        19. March 2015
* $Revision: 	V.1.4.5
*    
* Project: 	    CMSIS DSP Library    
* Title:
*    
* Description:
*    
* Target Processor: Cortex-M4/Cortex-M3/Cortex-M0
*  
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the 
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.   
* ---------------------------------------------------------------------------- */

#if defined(ARM_MATH_RPI)

#include <stdint.h>
#include "arm_math_rpi.h"

#define __SIMD32_TYPE int32_t
#define __SIMD32(addr)        (*(__SIMD32_TYPE **) & (addr))
#define _SIMD32_OFFSET(addr)  (*(__SIMD32_TYPE *)  (addr))

#define __PKHBT(ARG1, ARG2, ARG3)      ( (((int32_t)(ARG1) <<  0) & (int32_t)0x0000FFFF) | \
                                         (((int32_t)(ARG2) << ARG3) & (int32_t)0xFFFF0000)  )


static uint32_t __SMLAD(
  uint32_t x,
  uint32_t y,
  uint32_t sum)
  {
    return ((uint32_t)(((((q31_t)x << 16) >> 16) * (((q31_t)y << 16) >> 16)) +
                       ((((q31_t)x      ) >> 16) * (((q31_t)y      ) >> 16)) +
                       ( ((q31_t)sum    )                                  )   ));
  }


  /*
   * @brief C custom defined SMLADX for M3 and M0 processors
   */

static uint32_t __SMLADX(
  uint32_t x,
  uint32_t y,
  uint32_t sum)
{
    return ((uint32_t)(((((q31_t)x << 16) >> 16) * (((q31_t)y      ) >> 16)) +
                       ((((q31_t)x      ) >> 16) * (((q31_t)y << 16) >> 16)) +
                       ( ((q31_t)sum    )                                  )   ));
}

void arm_fir_interpolate_q15(
  const arm_fir_interpolate_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize)
{
  q15_t *pState = S->pState;                     /* State pointer                                            */
  q15_t *pCoeffs = S->pCoeffs;                   /* Coefficient pointer                                      */
  q15_t *pStateCurnt;                            /* Points to the current sample of the state                */
  q15_t *ptr1, *ptr2;                            /* Temporary pointers for state and coefficient buffers     */
  q63_t sum;                                     /* Accumulator */
  q15_t x0, c0;                                  /* Temporary variables to hold state and coefficient values */
  uint32_t i, blkCnt, tapCnt;                    /* Loop counters                                            */
  uint16_t phaseLen = S->phaseLength;            /* Length of each polyphase filter component */


  /* S->pState buffer contains previous frame (phaseLen - 1) samples */
  /* pStateCurnt points to the location where the new input data should be written */
  pStateCurnt = S->pState + (phaseLen - 1u);

  /* Total number of intput samples */
  blkCnt = blockSize;

  /* Loop over the blockSize. */
  while(blkCnt > 0u)
  {
    /* Copy new input sample into the state buffer */
    *pStateCurnt++ = *pSrc++;

    /* Loop over the Interpolation factor. */
    i = S->L;

    while(i > 0u)
    {
      /* Set accumulator to zero */
      sum = 0;

      /* Initialize state pointer */
      ptr1 = pState;

      /* Initialize coefficient pointer */
      ptr2 = pCoeffs + (i - 1u);

      /* Loop over the polyPhase length */
      tapCnt = (uint32_t) phaseLen;

      while(tapCnt > 0u)
      {
        /* Read the coefficient */
        c0 = *ptr2;

        /* Increment the coefficient pointer by interpolation factor times. */
        ptr2 += S->L;

        /* Read the input sample */
        x0 = *ptr1++;

        /* Perform the multiply-accumulate */
        sum += ((q31_t) x0 * c0);

        /* Decrement the loop counter */
        tapCnt--;
      }

      /* Store the result after converting to 1.15 format in the destination buffer */
      *pDst++ = (q15_t) (__SSAT((sum >> 15), 16));

      /* Decrement the loop counter */
      i--;
    }

    /* Advance the state pointer by 1           
     * to process the next group of interpolation factor number samples */
    pState = pState + 1;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Processing is complete.         
   ** Now copy the last phaseLen - 1 samples to the start of the state buffer.       
   ** This prepares the state buffer for the next function call. */

  /* Points to the start of the state buffer */
  pStateCurnt = S->pState;

  i = (uint32_t) phaseLen - 1u;

  while(i > 0u)
  {
    *pStateCurnt++ = *pState++;

    /* Decrement the loop counter */
    i--;
  }

}

void arm_fir_fast_q15(
  const arm_fir_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize)
{
  q15_t *pState = S->pState;                     /* State pointer */
  q15_t *pCoeffs = S->pCoeffs;                   /* Coefficient pointer */
  q15_t *pStateCurnt;                            /* Points to the current sample of the state */
  q31_t acc0, acc1, acc2, acc3;                  /* Accumulators */
  q15_t *pb;                                     /* Temporary pointer for coefficient buffer */
  q15_t *px;                                     /* Temporary q31 pointer for SIMD state buffer accesses */
  q31_t x0, x1, x2, c0;                          /* Temporary variables to hold SIMD state and coefficient values */
  uint32_t numTaps = S->numTaps;                 /* Number of taps in the filter */
  uint32_t tapCnt, blkCnt;                       /* Loop counters */


  /* S->pState points to state array which contains previous frame (numTaps - 1) samples */
  /* pStateCurnt points to the location where the new input data should be written */
  pStateCurnt = &(S->pState[(numTaps - 1u)]);

  /* Apply loop unrolling and compute 4 output values simultaneously.      
   * The variables acc0 ... acc3 hold output values that are being computed:      
   *      
   *    acc0 =  b[numTaps-1] * x[n-numTaps-1] + b[numTaps-2] * x[n-numTaps-2] + b[numTaps-3] * x[n-numTaps-3] +...+ b[0] * x[0]      
   *    acc1 =  b[numTaps-1] * x[n-numTaps] +   b[numTaps-2] * x[n-numTaps-1] + b[numTaps-3] * x[n-numTaps-2] +...+ b[0] * x[1]      
   *    acc2 =  b[numTaps-1] * x[n-numTaps+1] + b[numTaps-2] * x[n-numTaps] +   b[numTaps-3] * x[n-numTaps-1] +...+ b[0] * x[2]      
   *    acc3 =  b[numTaps-1] * x[n-numTaps+2] + b[numTaps-2] * x[n-numTaps+1] + b[numTaps-3] * x[n-numTaps]   +...+ b[0] * x[3]      
   */

  blkCnt = blockSize >> 2;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.      
   ** a second loop below computes the remaining 1 to 3 samples. */
  while(blkCnt > 0u)
  {
    /* Copy four new input samples into the state buffer.      
     ** Use 32-bit SIMD to move the 16-bit data.  Only requires two copies. */
    *pStateCurnt++ = *pSrc++;
    *pStateCurnt++ = *pSrc++;
    *pStateCurnt++ = *pSrc++;
    *pStateCurnt++ = *pSrc++;


    /* Set all accumulators to zero */
    acc0 = 0;
    acc1 = 0;
    acc2 = 0;
    acc3 = 0;

    /* Typecast q15_t pointer to q31_t pointer for state reading in q31_t */
    px = pState;

    /* Typecast q15_t pointer to q31_t pointer for coefficient reading in q31_t */
    pb = pCoeffs;

    /* Read the first two samples from the state buffer:  x[n-N], x[n-N-1] */
    x0 = *__SIMD32(px)++;

    /* Read the third and forth samples from the state buffer: x[n-N-2], x[n-N-3] */
    x2 = *__SIMD32(px)++;

    /* Loop over the number of taps.  Unroll by a factor of 4.      
     ** Repeat until we've computed numTaps-(numTaps%4) coefficients. */
    tapCnt = numTaps >> 2;

    while(tapCnt > 0)
    {
      /* Read the first two coefficients using SIMD:  b[N] and b[N-1] coefficients */
      c0 = *__SIMD32(pb)++;

      /* acc0 +=  b[N] * x[n-N] + b[N-1] * x[n-N-1] */
      acc0 = __SMLAD(x0, c0, acc0);

      /* acc2 +=  b[N] * x[n-N-2] + b[N-1] * x[n-N-3] */
      acc2 = __SMLAD(x2, c0, acc2);

      /* pack  x[n-N-1] and x[n-N-2] */
#ifndef ARM_MATH_BIG_ENDIAN
      x1 = __PKHBT(x2, x0, 0);
#else
      x1 = __PKHBT(x0, x2, 0);
#endif

      /* Read state x[n-N-4], x[n-N-5] */
      x0 = _SIMD32_OFFSET(px);

      /* acc1 +=  b[N] * x[n-N-1] + b[N-1] * x[n-N-2] */
      acc1 = __SMLADX(x1, c0, acc1);

      /* pack  x[n-N-3] and x[n-N-4] */
#ifndef ARM_MATH_BIG_ENDIAN
      x1 = __PKHBT(x0, x2, 0);
#else
      x1 = __PKHBT(x2, x0, 0);
#endif

      /* acc3 +=  b[N] * x[n-N-3] + b[N-1] * x[n-N-4] */
      acc3 = __SMLADX(x1, c0, acc3);

      /* Read coefficients b[N-2], b[N-3] */
      c0 = *__SIMD32(pb)++;

      /* acc0 +=  b[N-2] * x[n-N-2] + b[N-3] * x[n-N-3] */
      acc0 = __SMLAD(x2, c0, acc0);

      /* Read state x[n-N-6], x[n-N-7] with offset */
      x2 = _SIMD32_OFFSET(px + 2u);

      /* acc2 +=  b[N-2] * x[n-N-4] + b[N-3] * x[n-N-5] */
      acc2 = __SMLAD(x0, c0, acc2);

      /* acc1 +=  b[N-2] * x[n-N-3] + b[N-3] * x[n-N-4] */
      acc1 = __SMLADX(x1, c0, acc1);

      /* pack  x[n-N-5] and x[n-N-6] */
#ifndef ARM_MATH_BIG_ENDIAN
      x1 = __PKHBT(x2, x0, 0);
#else
      x1 = __PKHBT(x0, x2, 0);
#endif

      /* acc3 +=  b[N-2] * x[n-N-5] + b[N-3] * x[n-N-6] */
      acc3 = __SMLADX(x1, c0, acc3);

      /* Update state pointer for next state reading */
      px += 4u;

      /* Decrement tap count */
      tapCnt--;

    }

    /* If the filter length is not a multiple of 4, compute the remaining filter taps.       
     ** This is always be 2 taps since the filter length is even. */
    if((numTaps & 0x3u) != 0u)
    {

      /* Read last two coefficients */
      c0 = *__SIMD32(pb)++;

      /* Perform the multiply-accumulates */
      acc0 = __SMLAD(x0, c0, acc0);
      acc2 = __SMLAD(x2, c0, acc2);

      /* pack state variables */
#ifndef ARM_MATH_BIG_ENDIAN
      x1 = __PKHBT(x2, x0, 0);
#else
      x1 = __PKHBT(x0, x2, 0);
#endif

      /* Read last state variables */
      x0 = *__SIMD32(px);

      /* Perform the multiply-accumulates */
      acc1 = __SMLADX(x1, c0, acc1);

      /* pack state variables */
#ifndef ARM_MATH_BIG_ENDIAN
      x1 = __PKHBT(x0, x2, 0);
#else
      x1 = __PKHBT(x2, x0, 0);
#endif

      /* Perform the multiply-accumulates */
      acc3 = __SMLADX(x1, c0, acc3);
    }

    /* The results in the 4 accumulators are in 2.30 format.  Convert to 1.15 with saturation.       
     ** Then store the 4 outputs in the destination buffer. */

#ifndef ARM_MATH_BIG_ENDIAN

    *__SIMD32(pDst)++ =
      __PKHBT(__SSAT((acc0 >> 15), 16), __SSAT((acc1 >> 15), 16), 16);

    *__SIMD32(pDst)++ =
      __PKHBT(__SSAT((acc2 >> 15), 16), __SSAT((acc3 >> 15), 16), 16);

#else

    *__SIMD32(pDst)++ =
      __PKHBT(__SSAT((acc1 >> 15), 16), __SSAT((acc0 >> 15), 16), 16);

    *__SIMD32(pDst)++ =
      __PKHBT(__SSAT((acc3 >> 15), 16), __SSAT((acc2 >> 15), 16), 16);


#endif /*      #ifndef ARM_MATH_BIG_ENDIAN       */

    /* Advance the state pointer by 4 to process the next group of 4 samples */
    pState = pState + 4u;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the blockSize is not a multiple of 4, compute any remaining output samples here.      
   ** No loop unrolling is used. */
  blkCnt = blockSize % 0x4u;
  while(blkCnt > 0u)
  {
    /* Copy two samples into state buffer */
    *pStateCurnt++ = *pSrc++;

    /* Set the accumulator to zero */
    acc0 = 0;

    /* Use SIMD to hold states and coefficients */
    px = pState;
    pb = pCoeffs;

    tapCnt = numTaps >> 1u;

    do
    {

      acc0 += (q31_t) * px++ * *pb++;
	  acc0 += (q31_t) * px++ * *pb++;

      tapCnt--;
    }
    while(tapCnt > 0u);

    /* The result is in 2.30 format.  Convert to 1.15 with saturation.      
     ** Then store the output in the destination buffer. */
    *pDst++ = (q15_t) (__SSAT((acc0 >> 15), 16));

    /* Advance state pointer by 1 for the next sample */
    pState = pState + 1u;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Processing is complete.      
   ** Now copy the last numTaps - 1 samples to the satrt of the state buffer.      
   ** This prepares the state buffer for the next function call. */

  /* Points to the start of the state buffer */
  pStateCurnt = S->pState;

  /* Calculation of count for copying integer writes */
  tapCnt = (numTaps - 1u) >> 2;

  while(tapCnt > 0u)
  {
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;
    *pStateCurnt++ = *pState++;

    tapCnt--;

  }

  /* Calculation of count for remaining q15_t data */
  tapCnt = (numTaps - 1u) % 0x4u;

  /* copy remaining data */
  while(tapCnt > 0u)
  {
    *pStateCurnt++ = *pState++;

    /* Decrement the loop counter */
    tapCnt--;
  }

}

void arm_biquad_cascade_df1_q31(
  const arm_biquad_casd_df1_inst_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize)
{
  q63_t acc;                                     /*  accumulator                   */
  uint32_t uShift = ((uint32_t) S->postShift + 1u);
  uint32_t lShift = 32u - uShift;                /*  Shift to be applied to the output */
  q31_t *pIn = pSrc;                             /*  input pointer initialization  */
  q31_t *pOut = pDst;                            /*  output pointer initialization */
  q31_t *pState = S->pState;                     /*  pState pointer initialization */
  q31_t *pCoeffs = S->pCoeffs;                   /*  coeff pointer initialization  */
  q31_t Xn1, Xn2, Yn1, Yn2;                      /*  Filter state variables        */
  q31_t b0, b1, b2, a1, a2;                      /*  Filter coefficients           */
  q31_t Xn;                                      /*  temporary input               */
  uint32_t sample, stage = S->numStages;         /*  loop counters                     */

 do
  {
    /* Reading the coefficients */
    b0 = *pCoeffs++;
    b1 = *pCoeffs++;
    b2 = *pCoeffs++;
    a1 = *pCoeffs++;
    a2 = *pCoeffs++;

    /* Reading the state values */
    Xn1 = pState[0];
    Xn2 = pState[1];
    Yn1 = pState[2];
    Yn2 = pState[3];

    /*      The variables acc holds the output value that is computed:         
     *    acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2]         
     */

    sample = blockSize;

    while(sample > 0u)
    {
      /* Read the input */
      Xn = *pIn++;

      /* acc =  b0 * x[n] + b1 * x[n-1] + b2 * x[n-2] + a1 * y[n-1] + a2 * y[n-2] */
      /* acc =  b0 * x[n] */
      acc = (q63_t) b0 *Xn;

      /* acc +=  b1 * x[n-1] */
      acc += (q63_t) b1 *Xn1;
      /* acc +=  b[2] * x[n-2] */
      acc += (q63_t) b2 *Xn2;
      /* acc +=  a1 * y[n-1] */
      acc += (q63_t) a1 *Yn1;
      /* acc +=  a2 * y[n-2] */
      acc += (q63_t) a2 *Yn2;

      /* The result is converted to 1.31  */
      acc = acc >> lShift;

      /* Every time after the output is computed state should be updated. */
      /* The states should be updated as:  */
      /* Xn2 = Xn1    */
      /* Xn1 = Xn     */
      /* Yn2 = Yn1    */
      /* Yn1 = acc    */
      Xn2 = Xn1;
      Xn1 = Xn;
      Yn2 = Yn1;
      Yn1 = (q31_t) acc;

      /* Store the output in the destination buffer. */
      *pOut++ = (q31_t) acc;

      /* decrement the loop counter */
      sample--;
    }

    /*  The first stage goes from the input buffer to the output buffer. */
    /*  Subsequent stages occur in-place in the output buffer */
    pIn = pDst;

    /* Reset to destination pointer */
    pOut = pDst;

    /*  Store the updated state variables back into the pState array */
    *pState++ = Xn1;
    *pState++ = Xn2;
    *pState++ = Yn1;
    *pState++ = Yn2;

  } while(--stage);

}

void arm_q15_to_q31(
  q15_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize)
{
  q15_t *pIn = pSrc;                             /* Src pointer */
  uint32_t blkCnt;                               /* loop counter */

/* Loop over blockSize number of values */
  blkCnt = blockSize;

  while(blkCnt > 0u)
  {
    /* C = (q31_t)A << 16 */
    /* convert from q15 to q31 and then store the results in the destination buffer */
    *pDst++ = (q31_t) * pIn++ << 16;

    /* Decrement the loop counter */
    blkCnt--;
  }

}

/**
 * \par
 * Tables generated are in Q31(1.31 Fixed point format)
 * Generation of sin values in floating point:
 * <pre>tableSize = 256;
 * for(n = -1; n < (tableSize + 1); n++)
 * {
 *	sinTable[n+1]= sin(2*pi*n/tableSize);
 * } </pre>
 * where pi value is  3.14159265358979
 * \par
 * Convert Floating point to Q31(Fixed point):
 *	(sinTable[i] * pow(2, 31))
 * \par
 * rounding to nearest integer is done
 * 	sinTable[i] += (sinTable[i] > 0 ? 0.5 :-0.5);
 */

static const q31_t sinTableQ31[259] = {
  0xfcdbd541, 0x0, 0x3242abf, 0x647d97c, 0x96a9049, 0xc8bd35e, 0xfab272b,
  0x12c8106f,
  0x15e21445, 0x18f8b83c, 0x1c0b826a, 0x1f19f97b, 0x2223a4c5, 0x25280c5e,
  0x2826b928, 0x2b1f34eb,
  0x2e110a62, 0x30fbc54d, 0x33def287, 0x36ba2014, 0x398cdd32, 0x3c56ba70,
  0x3f1749b8, 0x41ce1e65,
  0x447acd50, 0x471cece7, 0x49b41533, 0x4c3fdff4, 0x4ebfe8a5, 0x5133cc94,
  0x539b2af0, 0x55f5a4d2,
  0x5842dd54, 0x5a82799a, 0x5cb420e0, 0x5ed77c8a, 0x60ec3830, 0x62f201ac,
  0x64e88926, 0x66cf8120,
  0x68a69e81, 0x6a6d98a4, 0x6c242960, 0x6dca0d14, 0x6f5f02b2, 0x70e2cbc6,
  0x72552c85, 0x73b5ebd1,
  0x7504d345, 0x7641af3d, 0x776c4edb, 0x78848414, 0x798a23b1, 0x7a7d055b,
  0x7b5d039e, 0x7c29fbee,
  0x7ce3ceb2, 0x7d8a5f40, 0x7e1d93ea, 0x7e9d55fc, 0x7f0991c4, 0x7f62368f,
  0x7fa736b4, 0x7fd8878e,
  0x7ff62182, 0x7fffffff, 0x7ff62182, 0x7fd8878e, 0x7fa736b4, 0x7f62368f,
  0x7f0991c4, 0x7e9d55fc,
  0x7e1d93ea, 0x7d8a5f40, 0x7ce3ceb2, 0x7c29fbee, 0x7b5d039e, 0x7a7d055b,
  0x798a23b1, 0x78848414,
  0x776c4edb, 0x7641af3d, 0x7504d345, 0x73b5ebd1, 0x72552c85, 0x70e2cbc6,
  0x6f5f02b2, 0x6dca0d14,
  0x6c242960, 0x6a6d98a4, 0x68a69e81, 0x66cf8120, 0x64e88926, 0x62f201ac,
  0x60ec3830, 0x5ed77c8a,
  0x5cb420e0, 0x5a82799a, 0x5842dd54, 0x55f5a4d2, 0x539b2af0, 0x5133cc94,
  0x4ebfe8a5, 0x4c3fdff4,
  0x49b41533, 0x471cece7, 0x447acd50, 0x41ce1e65, 0x3f1749b8, 0x3c56ba70,
  0x398cdd32, 0x36ba2014,
  0x33def287, 0x30fbc54d, 0x2e110a62, 0x2b1f34eb, 0x2826b928, 0x25280c5e,
  0x2223a4c5, 0x1f19f97b,
  0x1c0b826a, 0x18f8b83c, 0x15e21445, 0x12c8106f, 0xfab272b, 0xc8bd35e,
  0x96a9049, 0x647d97c,
  0x3242abf, 0x0, 0xfcdbd541, 0xf9b82684, 0xf6956fb7, 0xf3742ca2, 0xf054d8d5,
  0xed37ef91,
  0xea1debbb, 0xe70747c4, 0xe3f47d96, 0xe0e60685, 0xdddc5b3b, 0xdad7f3a2,
  0xd7d946d8, 0xd4e0cb15,
  0xd1eef59e, 0xcf043ab3, 0xcc210d79, 0xc945dfec, 0xc67322ce, 0xc3a94590,
  0xc0e8b648, 0xbe31e19b,
  0xbb8532b0, 0xb8e31319, 0xb64beacd, 0xb3c0200c, 0xb140175b, 0xaecc336c,
  0xac64d510, 0xaa0a5b2e,
  0xa7bd22ac, 0xa57d8666, 0xa34bdf20, 0xa1288376, 0x9f13c7d0, 0x9d0dfe54,
  0x9b1776da, 0x99307ee0,
  0x9759617f, 0x9592675c, 0x93dbd6a0, 0x9235f2ec, 0x90a0fd4e, 0x8f1d343a,
  0x8daad37b, 0x8c4a142f,
  0x8afb2cbb, 0x89be50c3, 0x8893b125, 0x877b7bec, 0x8675dc4f, 0x8582faa5,
  0x84a2fc62, 0x83d60412,
  0x831c314e, 0x8275a0c0, 0x81e26c16, 0x8162aa04, 0x80f66e3c, 0x809dc971,
  0x8058c94c, 0x80277872,
  0x8009de7e, 0x80000000, 0x8009de7e, 0x80277872, 0x8058c94c, 0x809dc971,
  0x80f66e3c, 0x8162aa04,
  0x81e26c16, 0x8275a0c0, 0x831c314e, 0x83d60412, 0x84a2fc62, 0x8582faa5,
  0x8675dc4f, 0x877b7bec,
  0x8893b125, 0x89be50c3, 0x8afb2cbb, 0x8c4a142f, 0x8daad37b, 0x8f1d343a,
  0x90a0fd4e, 0x9235f2ec,
  0x93dbd6a0, 0x9592675c, 0x9759617f, 0x99307ee0, 0x9b1776da, 0x9d0dfe54,
  0x9f13c7d0, 0xa1288376,
  0xa34bdf20, 0xa57d8666, 0xa7bd22ac, 0xaa0a5b2e, 0xac64d510, 0xaecc336c,
  0xb140175b, 0xb3c0200c,
  0xb64beacd, 0xb8e31319, 0xbb8532b0, 0xbe31e19b, 0xc0e8b648, 0xc3a94590,
  0xc67322ce, 0xc945dfec,
  0xcc210d79, 0xcf043ab3, 0xd1eef59e, 0xd4e0cb15, 0xd7d946d8, 0xdad7f3a2,
  0xdddc5b3b, 0xe0e60685,
  0xe3f47d96, 0xe70747c4, 0xea1debbb, 0xed37ef91, 0xf054d8d5, 0xf3742ca2,
  0xf6956fb7, 0xf9b82684,
  0xfcdbd541, 0x0, 0x3242abf
};

/**
 * @brief Fast approximation to the trigonometric sine function for Q31 data.
 * @param[in] x Scaled input value in radians.
 * @return  sin(x).
 *
 * The Q31 input value is in the range [0 +0.9999] and is mapped to a radian value in the range [0 2*pi), Here range excludes 2*pi.
 */

q31_t arm_sin_q31(
  q31_t x)
{
  q31_t sinVal, in, in2;                         /* Temporary variables for input, output */
  int32_t index;                                 /* Index variables */
  q31_t wa, wb, wc, wd;                          /* Cubic interpolation coefficients */
  q31_t a, b, c, d;                              /* Four nearest output values */
  q31_t *tablePtr;                               /* Pointer to table */
  q31_t fract, fractCube, fractSquare;           /* Temporary values for fractional values */
  q31_t oneBy6 = 0x15555555;                     /* Fixed point value of 1/6 */
  q31_t tableSpacing = TABLE_SPACING_Q31;        /* Table spacing */
  q31_t temp;                                    /* Temporary variable for intermediate process */

  in = x;

  /* Calculate the nearest index */
  index = (uint32_t) in / (uint32_t) tableSpacing;

  /* Calculate the nearest value of input */
  in2 = (q31_t) index *tableSpacing;

  /* Calculation of fractional value */
  fract = (in - in2) << 8;

  /* fractSquare = fract * fract */
  fractSquare = ((q31_t) (((q63_t) fract * fract) >> 32));
  fractSquare = fractSquare << 1;

  /* fractCube = fract * fract * fract */
  fractCube = ((q31_t) (((q63_t) fractSquare * fract) >> 32));
  fractCube = fractCube << 1;

  /* Checking min and max index of table */
  if(index < 0)
  {
    index = 0;
  }
  else if(index > 256)
  {
    index = 256;
  }

  /* Initialise table pointer */
  tablePtr = (q31_t *) & sinTableQ31[index];

  /* Cubic interpolation process */
  /* Calculation of wa */
  /* wa = -(oneBy6)*fractCube + (fractSquare >> 1u) - (0x2AAAAAAA)*fract; */
  wa = ((q31_t) (((q63_t) oneBy6 * fractCube) >> 32));
  temp = 0x2AAAAAAA;
  wa = (q31_t) ((((q63_t) wa << 32) + ((q63_t) temp * fract)) >> 32);
  wa = -(wa << 1u);
  wa += (fractSquare >> 1u);

  /* Read first nearest value of output from the sin table */
  a = *tablePtr++;

  /* sinVal = a*wa */
  sinVal = ((q31_t) (((q63_t) a * wa) >> 32));

  /* q31(1.31) Fixed point value of 1 */
  temp = 0x7FFFFFFF;

  /* Calculation of wb */
  wb = ((fractCube >> 1u) - (fractSquare + (fract >> 1u))) + temp;

  /* Read second nearest value of output from the sin table */
  b = *tablePtr++;

  /*  sinVal += b*wb */
  sinVal = (q31_t) ((((q63_t) sinVal << 32) + (q63_t) b * (wb)) >> 32);

  /* Calculation of wc */
  wc = -fractCube + fractSquare;
  wc = (wc >> 1u) + fract;

  /* Read third nearest value of output from the sin table */
  c = *tablePtr++;

  /*      sinVal += c*wc */
  sinVal = (q31_t) ((((q63_t) sinVal << 32) + ((q63_t) c * wc)) >> 32);

  /* Calculation of wd */
  /* wd = (oneBy6) * fractCube - (oneBy6) * fract; */
  fractCube = fractCube - fract;
  wd = ((q31_t) (((q63_t) oneBy6 * fractCube) >> 32));
  wd = (wd << 1u);

  /* Read fourth nearest value of output from the sin table */
  d = *tablePtr++;

  /* sinVal += d*wd; */
  sinVal = (q31_t) ((((q63_t) sinVal << 32) + ((q63_t) d * wd)) >> 32);

  /* convert sinVal in 2.30 format to 1.31 format */
  return (__QADD(sinVal, sinVal));

}

#endif

/**
 * @brief ARM DSP Compatible layer for RPI build
 * @todo Use NEON/SIMD optimization
 */

/* ----------------------------------------------------------------------
* Copyright (C) 2010-2015 ARM Limited. All rights reserved.
*
* $Date:        19. March 2015
* $Revision: 	V.1.4.5
*
* Project: 	    CMSIS DSP Library
* Title:	    arm_math.h
*
* Description:	Public header file for CMSIS DSP Library
*
* Target Processor: Cortex-M7/Cortex-M4/Cortex-M3/Cortex-M0
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
 * -------------------------------------------------------------------- */

#ifndef ARM_MATH_RPI_H
#define ARM_MATH_RPI_H

#include <cstddef>
#include <cstdint>

#define __INLINE __inline

typedef int16_t q15_t;
typedef int32_t q31_t;
typedef int64_t q63_t;

typedef float float32_t;
#define TABLE_SPACING_Q31     0x400000

typedef struct
  {
    uint16_t numTaps;         /**< number of filter coefficients in the filter. */
    q15_t *pState;            /**< points to the state variable array. The array is of length numTaps+blockSize-1. */
    q15_t *pCoeffs;           /**< points to the coefficient array. The array is of length numTaps.*/
  } arm_fir_instance_q15;

typedef struct
  {
    uint8_t L;                      /**< upsample factor. */
    uint16_t phaseLength;           /**< length of each polyphase filter component. */
    q15_t *pCoeffs;                 /**< points to the coefficient array. The array is of length L*phaseLength. */
    q15_t *pState;                  /**< points to the state variable array. The array is of length blockSize+phaseLength-1. */
  } arm_fir_interpolate_instance_q15;

typedef struct
  {
    uint32_t numStages;      /**< number of 2nd order stages in the filter.  Overall order is 2*numStages. */
    q31_t *pState;           /**< Points to the array of state coefficients.  The array is of length 4*numStages. */
    q31_t *pCoeffs;          /**< Points to the array of coefficients.  The array is of length 5*numStages. */
    uint8_t postShift;       /**< Additional shift, in bits, applied to each output sample. */
  } arm_biquad_casd_df1_inst_q31;

/**
   * @brief Processing function for the Q15 FIR interpolator.
   * @param[in]  S          points to an instance of the Q15 FIR interpolator structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of input samples to process per call.
   */
  void arm_fir_interpolate_q15(
  const arm_fir_interpolate_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);

/**
   * @brief Processing function for the fast Q15 FIR filter for Cortex-M3 and Cortex-M4.
   * @param[in]  S          points to an instance of the Q15 FIR filter structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
  void arm_fir_fast_q15(
  const arm_fir_instance_q15 * S,
  q15_t * pSrc,
  q15_t * pDst,
  uint32_t blockSize);

/**
   * @brief Processing function for the Q31 Biquad cascade filter
   * @param[in]  S          points to an instance of the Q31 Biquad cascade structure.
   * @param[in]  pSrc       points to the block of input data.
   * @param[out] pDst       points to the block of output data.
   * @param[in]  blockSize  number of samples to process.
   */
  void arm_biquad_cascade_df1_q31(
  const arm_biquad_casd_df1_inst_q31 * S,
  q31_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);

/**
   * @brief  Converts the elements of the Q15 vector to Q31 vector.
   * @param[in]  pSrc       is input pointer
   * @param[out] pDst       is output pointer
   * @param[in]  blockSize  is the number of samples to process
   */
  void arm_q15_to_q31(
  q15_t * pSrc,
  q31_t * pDst,
  uint32_t blockSize);

  /**
   * @brief  Fast approximation to the trigonometric sine function for Q31 data.
   * @param[in] x Scaled input value in radians.
   * @return  sin(x).
   */

  q31_t arm_sin_q31(
  q31_t x);

  /**
   * @brief Clips Q63 to Q31 values.
   */
  static __INLINE q31_t clip_q63_to_q31(
  q63_t x)
  {
    return ((q31_t) (x >> 32) != ((q31_t) x >> 31)) ?
      ((0x7FFFFFFF ^ ((q31_t) (x >> 63)))) : (q31_t) x;
  }

  /*
   * @brief C custom defined QADD for M3 and M0 processors
   */
  static __INLINE int32_t __QADD(
  int32_t x,
  int32_t y)
  {
    return ((int32_t)(clip_q63_to_q31((q63_t)x + (q31_t)y)));
  }

#if defined(__linux__) && defined(__arm__) && !defined(__aarch64__)
    /* arm32 */
    #define __SSAT(x, sat) \
    __extension__ \
    ({ \
        int32_t __res; \
        int32_t __val = (int32_t)(x); \
        __asm__ ("ssat %0, %1, %2" \
                 : "=r" (__res) \
                 : "I" (sat), "r" (__val) \
                 : "cc"); \
        __res; \
    })
#elif defined(__aarch64__)
    /* arm64 */
    #if defined(__GNUC__) || defined(__clang__)
        #define __SSAT(x, sat) ({ \
            int32_t __pos_limit = (1 << ((sat) - 1)) - 1; \
            int32_t __neg_limit = -(1 << ((sat) - 1)); \
            int32_t __v = (int32_t)(x); \
            (__v > __pos_limit) ? __pos_limit : ((__v < __neg_limit) ? __neg_limit : __v); \
        })
    #endif
#else
    /* fallback */
    static inline int32_t __SSAT(int32_t x, uint32_t sat) {
        int32_t pos_limit = (1 << (sat - 1)) - 1;
        int32_t neg_limit = -(1 << (sat - 1));
        if (x > pos_limit) return pos_limit;
        if (x < neg_limit) return neg_limit;
        return x;
    }
#endif

#endif

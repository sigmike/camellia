/***************************************
 *
 *  Camellia Image Processing Library
 *

    The Camellia Image Processing Library is an open source low-level image processing library.
    As it uses the IplImage structure to describe images, it is a good replacement to the IPL (Intel) library
    and a good complement to the OpenCV library. It includes a lot of functions for image processing
    (filtering, morphological mathematics, labeling, warping, loading/saving images, etc.),
    some of them being highly optimized; It is also cross-platform and robust. It is doxygen-documented
    and examples of use are provided.

    This software library is an outcome of the Camellia european project (IST-2001-34410).
    It was developped by the Ecole des Mines de Paris (ENSMP), in coordination with
    the other partners of the project.

  ==========================================================================

    Copyright (c) 2002-2007, Ecole des Mines de Paris - Centre de Robotique
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer
          in the documentation and/or other materials provided with the distribution.
        * Neither the name of the Ecole des Mines de Paris nor the names of
          its contributors may be used to endorse or promote products
          derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
    PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
    EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
    PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
    PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
    LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  ==========================================================================
*/

/* Monadic and Dyadic Arithmetic Operators kernel
 * C code */
#include "camellia.h"
#include "camellia_internals.h"

#define CAM_INVERSE1U
#include "cam_arithmetics_code.c"
#undef CAM_INVERSE1U
#include "cam_arithmetics_code.c"

#define CAM_GRAYSCALE

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation
#undef CAM_PIXEL
#undef CAM_SIGNED_PIXEL
#define CAM_PIXEL unsigned char
#define CAM_SIGNED_PIXEL signed char
#define camThreshold camThreshold8
#define camThresholdInv camThresholdInv8
#define camAbs camAbs8
#define camMonadicArithm camMonadicArithm8
#define camMonadicArithm1U camMonadicArithm1U8
#define camDyadicArithm camDyadicArithm8
#include "cam_arithmetics_code.c"
#undef camThreshold
#undef camThresholdInv
#undef camAbs
#undef camMonadicArithm
#undef camMonadicArithm1U
#undef camDyadicArithm

#undef CAM_PIXEL
#undef CAM_SIGNED_PIXEL
#define CAM_PIXEL unsigned short
#define CAM_SIGNED_PIXEL signed short 
#define camThreshold camThreshold16
#define camThresholdInv camThresholdInv16
#define camAbs camAbs16
#define camMonadicArithm camMonadicArithm16
#define camMonadicArithm1U camMonadicArithm1U16
#define camDyadicArithm camDyadicArithm16
#include "cam_arithmetics_code.c"
#undef camThreshold
#undef camThresholdInv
#undef camAbs
#undef camMonadicArithm
#undef camMonadicArithm1U
#undef camDyadicArithm

int camThreshold(CamImage *source, CamImage *dest,int threshold)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camThreshold16(source,dest,threshold);
    } else {
	return camThreshold8(source,dest,threshold);
    }
}

int camThresholdInv(CamImage *source, CamImage *dest,int thresholdInv)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camThresholdInv16(source,dest,thresholdInv);
    } else {
	return camThresholdInv8(source,dest,thresholdInv);
    }
}

int camAbs(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camAbs16(source,dest);
    } else {
	return camAbs8(source,dest);
    }
}

int camMonadicArithm(CamImage *source, CamImage *dest, CamArithmParams *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camMonadicArithm16(source,dest,params);
    } else {
	return camMonadicArithm8(source,dest,params);
    }
}

int camAdd(CamImage *source1, CamImage *source2, CamImage *dest) 
{
    CamArithmParams params;
    params.operation = CAM_ARITHM_ADD;
    return camDyadicArithm(source1, source2, dest, &params);
}

int camMul(CamImage *source1, CamImage *source2, CamImage *dest)
{
    CamArithmParams params;
    params.operation = CAM_ARITHM_MUL;
    params.c1 = 0;
    return camDyadicArithm(source1, source2, dest, &params);
}

int camSub(CamImage *source1, CamImage *source2, CamImage *dest)
{
    CamArithmParams params;
    params.operation = CAM_ARITHM_SUB;
    return camDyadicArithm(source1, source2, dest, &params);
}

int camDyadicArithm(CamImage *source1, CamImage *source2, CamImage *dest, CamArithmParams *params)
{
    if ((source1->depth&CAM_DEPTH_MASK)>8) {
	return camDyadicArithm16(source1,source2,dest,params);
    } else {
	return camDyadicArithm8(source1,source2,dest,params);
    }
}

#else
#include "cam_arithmetics_code.c"
#endif


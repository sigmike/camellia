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

/* Linear Filtering Kernel
 * C code */
 
#include <stdio.h>
#include <string.h>
#include "camellia.h"
#include "camellia_internals.h"

#ifdef CAM_GENERATE_FULL_CODE

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST unsigned char

// 3x3 Kernel Linear Filtering code generation
#define camLinearFilter camLinearFilter3x38
#define camLinearFilterKernelPtr CamLinearFilterKernel*
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs3x38
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Linear Filter code generation
#define camLinearFilter camLinearFilter5x58
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs5x58
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_SOBEL
#define camLinearFilter camSobel8
#define vertical_edges params
#define camLinearFilterKernelPtr int
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camSobelAbs8
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST unsigned short

// 3x3 Kernel Linear Filtering code generation
#define camLinearFilter camLinearFilter3x316
#define camLinearFilterKernelPtr CamLinearFilterKernel*
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs3x316
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Linear Filter code generation
#define camLinearFilter camLinearFilter5x516
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs5x516
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_SOBEL
#define camLinearFilter camSobel16
#define vertical_edges params
#define camLinearFilterKernelPtr int
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camSobelAbs16
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST unsigned short

// 3x3 Kernel Linear Filtering code generation
#define camLinearFilter camLinearFilter3x38to16
#define camLinearFilterKernelPtr CamLinearFilterKernel*
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs3x38to16
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Linear Filter code generation
#define camLinearFilter camLinearFilter5x58to16
#define camLinearFilterKernelPtr CamLinearFilterKernel*
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs5x58to16
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_SOBEL
#define camLinearFilter camSobel8to16
#define vertical_edges params
#define camLinearFilterKernelPtr int
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camSobelAbs8to16
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST unsigned char

// 3x3 Kernel Linear Filtering code generation
#define camLinearFilter camLinearFilter3x316to8
#define camLinearFilterKernelPtr CamLinearFilterKernel*
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs3x316to8
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Linear Filter code generation
#define camLinearFilter camLinearFilter5x516to8
#define camLinearFilterKernelPtr CamLinearFilterKernel*
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs5x516to8
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_SOBEL
#define camLinearFilter camSobel16to8
#define vertical_edges params
#define camLinearFilterKernelPtr int
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camSobelAbs16to8
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

int camLinearFilter3x3(CamImage* source, CamImage* dest, CamLinearFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camLinearFilter3x316to8(source,dest,params);
        } else return camLinearFilter3x316(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camLinearFilter3x38(source,dest,params);
        } else return camLinearFilter3x38to16(source,dest,params);
    }
}

int camLinearFilterAbs3x3(CamImage* source, CamImage* dest, CamLinearFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camLinearFilterAbs3x316to8(source,dest,params);
        } else return camLinearFilterAbs3x316(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camLinearFilterAbs3x38(source,dest,params);
        } else return camLinearFilterAbs3x38to16(source,dest,params);
    }
}

int camLinearFilter5x5(CamImage* source, CamImage* dest, CamLinearFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camLinearFilter5x516to8(source,dest,params);
        } else return camLinearFilter5x516(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camLinearFilter5x58(source,dest,params);
        } else return camLinearFilter5x58to16(source,dest,params);
    }
}

int camLinearFilterAbs5x5(CamImage* source, CamImage* dest, CamLinearFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camLinearFilterAbs5x516to8(source,dest,params);
        } else return camLinearFilterAbs5x516(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camLinearFilterAbs5x58(source,dest,params);
        } else return camLinearFilterAbs5x58to16(source,dest,params);
    }
}

int camSobel(CamImage* source, CamImage* dest, int vert_edges)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobel16to8(source,dest,vert_edges);
        } else return camSobel16(source,dest,vert_edges);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobel8(source,dest,vert_edges);
        } else return camSobel8to16(source,dest,vert_edges);
    }
}

int camSobelAbs(CamImage* source, CamImage* dest, int vert_edges)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelAbs16to8(source,dest,vert_edges);
        } else return camSobelAbs16(source,dest,vert_edges);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelAbs8(source,dest,vert_edges);
        } else return camSobelAbs8to16(source,dest,vert_edges);
    }
}

// Separable filters

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST char

// 3x3 Kernel Sep Filtering code generation
#define camSepFilter camSepFilter3x38
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs3x38
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Sep Filter code generation
#define camSepFilter camSepFilter5x58
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs5x58
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 7x7 Kernel Sep Filter code generation
#define camSepFilter camSepFilter7x78
#define CAM_LF_NEIGHB_X 7
#define CAM_LF_NEIGHB_Y 7
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs7x78
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define CAM_FIXED_FILTER CAM_SOBEL_H
#define camSepFilter camSobelH8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSobelHAbs8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#define CAM_FIXED_FILTER CAM_SOBEL_V
#define camSepFilter camSobelV8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSobelVAbs8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Scharr Filter
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define CAM_FIXED_FILTER CAM_SCHARR_H
#define camSepFilter camScharrH8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camScharrHAbs8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#define CAM_FIXED_FILTER CAM_SCHARR_V
#define camSepFilter camScharrV8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camScharrVAbs8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST short

// 3x3 Kernel Sep Filtering code generation
#define camSepFilter camSepFilter3x316
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs3x316
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Sep Filter code generation
#define camSepFilter camSepFilter5x516
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs5x516
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 7x7 Kernel Sep Filter code generation
#define camSepFilter camSepFilter7x716
#define CAM_LF_NEIGHB_X 7
#define CAM_LF_NEIGHB_Y 7
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs7x716
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define CAM_FIXED_FILTER CAM_SOBEL_H
#define camSepFilter camSobelH16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSobelHAbs16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#define CAM_FIXED_FILTER CAM_SOBEL_V
#define camSepFilter camSobelV16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSobelVAbs16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Scharr Filter
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define CAM_FIXED_FILTER CAM_SCHARR_H
#define camSepFilter camScharrH16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camScharrHAbs16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#define CAM_FIXED_FILTER CAM_SCHARR_V
#define camSepFilter camScharrV16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camScharrVAbs16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST short

// 3x3 Kernel Sep Filtering code generation
#define camSepFilter camSepFilter3x38to16
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs3x38to16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Sep Filter code generation
#define camSepFilter camSepFilter5x58to16
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs5x58to16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 7x7 Kernel Sep Filter code generation
#define camSepFilter camSepFilter7x78to16
#define CAM_LF_NEIGHB_X 7
#define CAM_LF_NEIGHB_Y 7
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs7x78to16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define CAM_FIXED_FILTER CAM_SOBEL_H
#define camSepFilter camSobelH8to16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSobelHAbs8to16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#define CAM_FIXED_FILTER CAM_SOBEL_V
#define camSepFilter camSobelV8to16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSobelVAbs8to16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Scharr Filter
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define CAM_FIXED_FILTER CAM_SCHARR_H
#define camSepFilter camScharrH8to16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camScharrHAbs8to16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#define CAM_FIXED_FILTER CAM_SCHARR_V
#define camSepFilter camScharrV8to16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camScharrVAbs8to16
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST char

// 3x3 Kernel Sep Filtering code generation
#define camSepFilter camSepFilter3x316to8
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs3x316to8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Sep Filter code generation
#define camSepFilter camSepFilter5x516to8
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs5x516to8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 7x7 Kernel Sep Filter code generation
#define camSepFilter camSepFilter7x716to8
#define CAM_LF_NEIGHB_X 7
#define CAM_LF_NEIGHB_Y 7
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs7x716to8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define CAM_FIXED_FILTER CAM_SOBEL_H
#define camSepFilter camSobelH16to8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSobelHAbs16to8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#define CAM_FIXED_FILTER CAM_SOBEL_V
#define camSepFilter camSobelV16to8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSobelVAbs16to8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Scharr Filter
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define CAM_FIXED_FILTER CAM_SCHARR_H
#define camSepFilter camScharrH16to8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camScharrHAbs16to8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#define CAM_FIXED_FILTER CAM_SCHARR_V
#define camSepFilter camScharrV16to8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camScharrVAbs16to8
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_FIXED_FILTER
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

int camSepFilter3x3(CamImage* source, CamImage* dest, CamSepFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilter3x316to8(source,dest,params);
        } else return camSepFilter3x316(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilter3x38(source,dest,params);
        } else return camSepFilter3x38to16(source,dest,params);
    }
}

int camSepFilterAbs3x3(CamImage* source, CamImage* dest, CamSepFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilterAbs3x316to8(source,dest,params);
        } else return camSepFilterAbs3x316(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilterAbs3x38(source,dest,params);
        } else return camSepFilterAbs3x38to16(source,dest,params);
    }
}

int camSepFilter5x5(CamImage* source, CamImage* dest, CamSepFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilter5x516to8(source,dest,params);
        } else return camSepFilter5x516(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilter5x58(source,dest,params);
        } else return camSepFilter5x58to16(source,dest,params);
    }
}

int camSepFilterAbs5x5(CamImage* source, CamImage* dest, CamSepFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilterAbs5x516to8(source,dest,params);
        } else return camSepFilterAbs5x516(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilterAbs5x58(source,dest,params);
        } else return camSepFilterAbs5x58to16(source,dest,params);
    }
}

int camSepFilter7x7(CamImage* source, CamImage* dest, CamSepFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilter7x716to8(source,dest,params);
        } else return camSepFilter7x716(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilter7x78(source,dest,params);
        } else return camSepFilter7x78to16(source,dest,params);
    }
}

int camSepFilterAbs7x7(CamImage* source, CamImage* dest, CamSepFilterKernel *params)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilterAbs7x716to8(source,dest,params);
        } else return camSepFilterAbs7x716(source,dest,params);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSepFilterAbs7x78(source,dest,params);
        } else return camSepFilterAbs7x78to16(source,dest,params);
    }
}

int camSobelH(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelH16to8(source,dest);
        } else return camSobelH16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelH8(source,dest);
        } else return camSobelH8to16(source,dest);
    }
}

int camSobelHAbs(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelHAbs16to8(source,dest);
        } else return camSobelHAbs16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelHAbs8(source,dest);
        } else return camSobelHAbs8to16(source,dest);
    }
}

int camSobelV(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelV16to8(source,dest);
        } else return camSobelV16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelV8(source,dest);
        } else return camSobelV8to16(source,dest);
    }
}

int camSobelVAbs(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelVAbs16to8(source,dest);
        } else return camSobelVAbs16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camSobelVAbs8(source,dest);
        } else return camSobelVAbs8to16(source,dest);
    }
}

int camScharrH(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camScharrH16to8(source,dest);
        } else return camScharrH16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camScharrH8(source,dest);
        } else return camScharrH8to16(source,dest);
    }
}

int camScharrHAbs(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camScharrHAbs16to8(source,dest);
        } else return camScharrHAbs16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camScharrHAbs8(source,dest);
        } else return camScharrHAbs8to16(source,dest);
    }
}

int camScharrV(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camScharrV16to8(source,dest);
        } else return camScharrV16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camScharrV8(source,dest);
        } else return camScharrV8to16(source,dest);
    }
}

int camScharrVAbs(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camScharrVAbs16to8(source,dest);
        } else return camScharrVAbs16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camScharrVAbs8(source,dest);
        } else return camScharrVAbs8to16(source,dest);
    }
}

// Fixed filter generation
#define CAM_FIXED_FILTER CAM_GAUSSIAN_3x3
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST char
#define camSepFilter camGaussian3x38
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST short
#define camSepFilter camGaussian3x316
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST short
#define camSepFilter camGaussian3x38to16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST char
#define camSepFilter camGaussian3x316to8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y
#undef CAM_FIXED_FILTER

#define CAM_FIXED_FILTER CAM_GAUSSIAN_5x5
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST char
#define camSepFilter camGaussian5x58
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST short
#define camSepFilter camGaussian5x516
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST short
#define camSepFilter camGaussian5x58to16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST char
#define camSepFilter camGaussian5x516to8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y
#undef CAM_FIXED_FILTER

#define CAM_FIXED_FILTER CAM_GAUSSIAN_7x7
#define CAM_LF_NEIGHB_X 7
#define CAM_LF_NEIGHB_Y 7
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST char
#define camSepFilter camGaussian7x78
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST short
#define camSepFilter camGaussian7x716
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST short
#define camSepFilter camGaussian7x78to16
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL short
#define CAM_PIXEL_DST char
#define camSepFilter camGaussian7x716to8
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y
#undef CAM_FIXED_FILTER

int camFixedFilter(CamImage *source, CamImage *dest, int filter)
{
    switch (filter) {
	case CAM_SOBEL_H :
	    if ((source->depth&CAM_DEPTH_MASK)>8) {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camSobelH16to8(source,dest);
		} else return camSobelH16(source,dest);
	    } else {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camSobelH8(source,dest);
		} else return camSobelH8to16(source,dest);
	    }
	case CAM_SOBEL_V :
	    if ((source->depth&CAM_DEPTH_MASK)>8) {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camSobelV16to8(source,dest);
		} else return camSobelV16(source,dest);
	    } else {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camSobelV8(source,dest);
		} else return camSobelV8to16(source,dest);
	    }
	case CAM_GAUSSIAN_3x3 :
	    if ((source->depth&CAM_DEPTH_MASK)>8) {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camGaussian3x316to8(source,dest);
		} else return camGaussian3x316(source,dest);
	    } else {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camGaussian3x38(source,dest);
		} else return camGaussian3x38to16(source,dest);
	    }
	case CAM_GAUSSIAN_5x5 :
	    if ((source->depth&CAM_DEPTH_MASK)>8) {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camGaussian5x516to8(source,dest);
		} else return camGaussian5x516(source,dest);
	    } else {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camGaussian5x58(source,dest);
		} else return camGaussian5x58to16(source,dest);
	    }
	case CAM_GAUSSIAN_7x7 :
	    if ((source->depth&CAM_DEPTH_MASK)>8) {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camGaussian7x716to8(source,dest);
		} else return camGaussian7x716(source,dest);
	    } else {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camGaussian7x78(source,dest);
		} else return camGaussian7x78to16(source,dest);
	    }
	case CAM_SCHARR_H :
	    if ((source->depth&CAM_DEPTH_MASK)>8) {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camScharrH16to8(source,dest);
		} else return camScharrH16(source,dest);
	    } else {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camScharrH8(source,dest);
		} else return camScharrH8to16(source,dest);
	    }
	case CAM_SCHARR_V :
	    if ((source->depth&CAM_DEPTH_MASK)>8) {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camScharrV16to8(source,dest);
		} else return camScharrV16(source,dest);
	    } else {
		if ((dest->depth&CAM_DEPTH_MASK)==8) {
		    return camScharrV8(source,dest);
		} else return camScharrV8to16(source,dest);
	    }
    }
    return 0;
}

#else

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL char
#define CAM_PIXEL_DST char

// 3x3 Kernel Linear Filtering code generation
#define camLinearFilter camLinearFilter3x3
#define camLinearFilterKernelPtr CamLinearFilterKernel*
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs3x3
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Linear Filter code generation
#define camLinearFilter camLinearFilter5x5
#define camLinearFilterKernelPtr CamLinearFilterKernel*
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camLinearFilterAbs5x5
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Sobel Filter
#define CAM_SOBEL
#define camLinearFilter camSobel
#define vertical_edges params
#define camLinearFilterKernelPtr int
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_linear_filtering_code.c"
#undef camLinearFilter
#define camLinearFilter camSobelAbs
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camLinearFilter
#undef camLinearFilterKernelPtr
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 3x3 Kernel Linear Filtering code generation
#define camSepFilter camSepFilter3x3
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs3x3
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 5x5 Kernel Sep Filter code generation
#define camSepFilter camSepFilter5x5
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs5x5
#define CAM_LF_ABS
#include "cam_separable_filter_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// 7x7 Kernel Sep Filter code generation
#define camSepFilter camSepFilter7x7
#define CAM_LF_NEIGHB_X 7
#define CAM_LF_NEIGHB_Y 7
#include "cam_separable_filter_code.c"
#undef camSepFilter
#define camSepFilter camSepFilterAbs7x7
#define CAM_LF_ABS
#include "cam_linear_filtering_code.c"
#undef CAM_LF_ABS
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y

// Fixed filter generation
#define CAM_FIXED_FILTER CAM_GAUSSIAN_3x3
#define CAM_LF_NEIGHB_X 3
#define CAM_LF_NEIGHB_Y 3
#define camSepFilter camGaussian3x3
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y
#undef CAM_FIXED_FILTER

#define CAM_FIXED_FILTER CAM_GAUSSIAN_5x5
#define CAM_LF_NEIGHB_X 5
#define CAM_LF_NEIGHB_Y 5
#define camSepFilter camGaussian5x5
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y
#undef CAM_FIXED_FILTER

#define CAM_FIXED_FILTER CAM_GAUSSIAN_7x7
#define CAM_LF_NEIGHB_X 7
#define CAM_LF_NEIGHB_Y 7
#define camSepFilter camGaussian7x7
#include "cam_separable_filter_code.c"
#undef camSepFilter
#undef CAM_LF_NEIGHB_X
#undef CAM_LF_NEIGHB_Y
#undef CAM_FIXED_FILTER

int camFixedFilter(CamImage *source, CamImage *dest, int filter)
{
    switch (filter) {
    case CAM_SOBEL_H : return camSobelH(source,dest);
    case CAM_SOBEL_V : return camSobelV(source,dest);
    case CAM_GAUSSIAN_3x3 : return camGaussian3x3(source,dest);
    case CAM_GAUSSIAN_5x5 : return camGaussian5x5(source,dest);
    case CAM_GAUSSIAN_7x7 : return camGaussian7x7(source,dest);
    }
    return 0;
}
#endif

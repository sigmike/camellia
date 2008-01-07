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
/* Fundamental Morphological Mathematics Kernel
 * C code */
#include "camellia.h"
#include "camellia_internals.h"

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation

#undef CAM_PIXEL
#define CAM_PIXEL char

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 5
#define camMorphoMaths camMorphoMaths8
#include "cam_morphomaths_code.c"
#undef camMorphoMaths

#define CAM_MM_ONE_OP

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 3
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode3x38
#define camMorphoMaths1U camErode3x31U
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef camMorphoMaths1U
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate3x38
#define camMorphoMaths1U camDilate3x31U
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths
#undef camMorphoMaths1U

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 5
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode5x58
#define camMorphoMaths1U camErode5x51U
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef camMorphoMaths1U
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate5x58
#define camMorphoMaths1U camDilate5x51U
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths
#undef camMorphoMaths1U

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 7
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode7x78
#define camMorphoMaths1U camErode7x71U
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef camMorphoMaths1U
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate7x78
#define camMorphoMaths1U camDilate7x71U
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths
#undef camMorphoMaths1U

#define CAM_MM_OPT_CIRCLE5
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 5
#define CAM_MM_DO_EROSION
#define camErodeCircle5 camErodeCircle58
#include "cam_morphomaths_code_opt.c"
#undef camErodeCircle5
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camDilateCircle5 camDilateCircle58
#include "cam_morphomaths_code_opt.c"
#undef camDilateCircle5
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#define camMorphoGradientCircle5 camMorphoGradientCircle58
#include "cam_morphomaths_code_opt.c"
#undef camMorphoGradientCircle5
#undef CAM_MM_GRADIENT

#define CAM_MM_FINDLOCALMAXIMA
#define camFindLocalMaximaCircle5 camFindLocalMaximaCircle58
#include "cam_morphomaths_code_opt.c"
#undef camFindLocalMaximaCircle5
#undef CAM_MM_FINDLOCALMAXIMA

#undef CAM_MM_OPT_CIRCLE5

#define CAM_MM_OPT_SQUARE3
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 3
#define CAM_MM_DO_EROSION
#define camErodeSquare3 camErodeSquare38
#include "cam_morphomaths_code_opt.c"
#undef camErodeSquare3
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camDilateSquare3 camDilateSquare38
#include "cam_morphomaths_code_opt.c"
#undef camDilateSquare3
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#define camMorphoGradientSquare3 camMorphoGradientSquare38
#include "cam_morphomaths_code_opt.c"
#undef camMorphoGradientSquare3
#undef CAM_MM_GRADIENT

#define CAM_MM_FINDLOCALMAXIMA
#define camFindLocalMaximaSquare3 camFindLocalMaximaSquare38
#include "cam_morphomaths_code_opt.c"
#undef camFindLocalMaximaSquare3
#undef CAM_MM_FINDLOCALMAXIMA

#undef CAM_MM_OPT_SQUARE3

// For Circle7 binary erosion
static const int camCircle7StructElt[7][7]={
    {0,0,1,1,1,0,0},
    {0,1,1,1,1,1,0},
    {1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1},
    {0,1,1,1,1,1,0},
    {0,0,1,1,1,0,0}
};

#define CAM_MM_OPT_CIRCLE7
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 7
#define CAM_MM_DO_EROSION
#define camErodeCircle7 camErodeCircle78
#include "cam_morphomaths_code_opt.c"
#undef camErodeCircle7
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camDilateCircle7 camDilateCircle78
#include "cam_morphomaths_code_opt.c"
#undef camDilateCircle7
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#define camMorphoGradientCircle7 camMorphoGradientCircle78
#include "cam_morphomaths_code_opt.c"
#undef camMorphoGradientCircle7
#undef CAM_MM_GRADIENT

#define CAM_MM_FINDLOCALMAXIMA
#define camFindLocalMaximaCircle7 camFindLocalMaximaCircle78
#include "cam_morphomaths_code_opt.c"
#undef camFindLocalMaximaCircle7
#undef CAM_MM_FINDLOCALMAXIMA

#undef CAM_MM_OPT_CIRCLE7

#define GRAY_SCALE_ONLY

#undef CAM_PIXEL
#define CAM_PIXEL short 

#undef CAM_MM_ONE_OP

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 5
#define camMorphoMaths camMorphoMaths16
#include "cam_morphomaths_code.c"
#undef camMorphoMaths

#define CAM_MM_ONE_OP

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 3
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode3x316
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate3x316
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 5
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode5x516
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate5x516
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 7
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode7x716
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate7x716
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths

#define CAM_MM_OPT_CIRCLE5
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 5
#define CAM_MM_DO_EROSION
#define camErodeCircle5 camErodeCircle516
#include "cam_morphomaths_code_opt.c"
#undef camErodeCircle5
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camDilateCircle5 camDilateCircle516
#include "cam_morphomaths_code_opt.c"
#undef camDilateCircle5
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#define camMorphoGradientCircle5 camMorphoGradientCircle516
#include "cam_morphomaths_code_opt.c"
#undef camMorphoGradientCircle5
#undef CAM_MM_GRADIENT

#define CAM_MM_FINDLOCALMAXIMA
#define camFindLocalMaximaCircle5 camFindLocalMaximaCircle516
#include "cam_morphomaths_code_opt.c"
#undef camFindLocalMaximaCircle5
#undef CAM_MM_FINDLOCALMAXIMA

#undef CAM_MM_OPT_CIRCLE5

#define CAM_MM_OPT_SQUARE3
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 3
#define CAM_MM_DO_EROSION
#define camErodeSquare3 camErodeSquare316
#include "cam_morphomaths_code_opt.c"
#undef camErodeSquare3
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camDilateSquare3 camDilateSquare316
#include "cam_morphomaths_code_opt.c"
#undef camDilateSquare3
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#define camMorphoGradientSquare3 camMorphoGradientSquare316
#include "cam_morphomaths_code_opt.c"
#undef camMorphoGradientSquare3
#undef CAM_MM_GRADIENT

#define CAM_MM_FINDLOCALMAXIMA
#define camFindLocalMaximaSquare3 camFindLocalMaximaSquare316
#include "cam_morphomaths_code_opt.c"
#undef camFindLocalMaximaSquare3
#undef CAM_MM_FINDLOCALMAXIMA

#undef CAM_MM_OPT_SQUARE3

#define CAM_MM_OPT_CIRCLE7
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 7
#define CAM_MM_DO_EROSION
#define camErodeCircle7 camErodeCircle716
#include "cam_morphomaths_code_opt.c"
#undef camErodeCircle7
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camDilateCircle7 camDilateCircle716
#include "cam_morphomaths_code_opt.c"
#undef camDilateCircle7
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#define camMorphoGradientCircle7 camMorphoGradientCircle716
#include "cam_morphomaths_code_opt.c"
#undef camMorphoGradientCircle7
#undef CAM_MM_GRADIENT

#define CAM_MM_FINDLOCALMAXIMA
#define camFindLocalMaximaCircle7 camFindLocalMaximaCircle716
#include "cam_morphomaths_code_opt.c"
#undef camFindLocalMaximaCircle7
#undef CAM_MM_FINDLOCALMAXIMA

#undef CAM_MM_OPT_CIRCLE7

int camFindLocalMaximaCircle5(CamImage *source, CamKeypoints *points, int threshold)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camFindLocalMaximaCircle516(source, points, threshold);
    } else {
        return camFindLocalMaximaCircle58(source, points, threshold);
    }
}

int camFindLocalMaximaSquare3(CamImage *source, CamKeypoints *points, int threshold)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camFindLocalMaximaSquare316(source, points, threshold);
    } else {
        return camFindLocalMaximaSquare38(source, points, threshold);
    }
}

int camFindLocalMaximaCircle7(CamImage *source, CamKeypoints *points, int threshold)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camFindLocalMaximaCircle716(source, points, threshold);
    } else {
        return camFindLocalMaximaCircle78(source, points, threshold);
    }
}

int camErodeCircle7(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camErodeCircle716(source,dest);
    } else {
        return camErodeCircle78(source,dest);
    }
}

int camErodeCircle5(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camErodeCircle516(source,dest);
    } else {
        return camErodeCircle58(source,dest);
    }
}

int camErodeSquare3(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camErodeSquare316(source,dest);
    } else {
        return camErodeSquare38(source,dest);
    }
}

int camDilateCircle7(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camDilateCircle716(source,dest);
    } else {
        return camDilateCircle78(source,dest);
    }
}

int camDilateCircle5(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camDilateCircle516(source,dest);
    } else {
        return camDilateCircle58(source,dest);
    }
}

int camDilateSquare3(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camDilateSquare316(source,dest);
    } else {
        return camDilateSquare38(source,dest);
    }
}

int camMorphoGradientCircle7(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camMorphoGradientCircle716(source,dest);
    } else {
        return camMorphoGradientCircle78(source,dest);
    }
}

int camMorphoGradientCircle5(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camMorphoGradientCircle516(source,dest);
    } else {
        return camMorphoGradientCircle58(source,dest);
    }
}

int camMorphoGradientSquare3(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camMorphoGradientSquare316(source,dest);
    } else {
        return camMorphoGradientSquare38(source,dest);
    }
}

int camMorphoMaths(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camMorphoMaths16(source,dest,kernel);
    } else {
        return camMorphoMaths8(source,dest,kernel);
    }
}

int camDilate3x3(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camDilate3x316(source,dest,kernel);
    } else {
        return camDilate3x38(source,dest,kernel);
    }
}

int camDilate5x5(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camDilate5x516(source,dest,kernel);
    } else {
        return camDilate5x58(source,dest,kernel);
    }
}

int camDilate7x7(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camDilate7x716(source,dest,kernel);
    } else {
        return camDilate7x78(source,dest,kernel);
    }
}

int camErode3x3(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camErode3x316(source,dest,kernel);
    } else {
        return camErode3x38(source,dest,kernel);
    }
}

int camErode5x5(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camErode5x516(source,dest,kernel);
    } else {
        return camErode5x58(source,dest,kernel);
    }
}

int camErode7x7(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        return camErode7x716(source,dest,kernel);
    } else {
        return camErode7x78(source,dest,kernel);
    }
}

#else

#define CAM_MM_NEIGHB 5
#include "cam_morphomaths_code.c"

#define CAM_MM_ONE_OP

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 3
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode3x3
#define camMorphoMaths1U camErode3x31U
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef camMorphoMaths1U
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate3x3
#define camMorphoMaths1U camDilate3x31U
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths
#undef camMorphoMaths1U

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 5
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode5x5
#define camMorphoMaths1U camErode5x51U
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef camMorphoMaths1U
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate5x5
#define camMorphoMaths1U camDilate5x51U
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths
#undef camMorphoMaths1U

#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 7
#define CAM_MM_DO_EROSION
#define camMorphoMaths camErode7x7
#define camMorphoMaths1U camErode7x71U
#include "cam_morphomaths_code.c"
#undef camMorphoMaths
#undef camMorphoMaths1U
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#define camMorphoMaths camDilate7x7
#define camMorphoMaths1U camDilate7x71U
#include "cam_morphomaths_code.c"
#undef CAM_MM_DO_DILATION
#undef camMorphoMaths
#undef camMorphoMaths1U

#define CAM_MM_OPT_CIRCLE5
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 5
#define CAM_MM_DO_EROSION
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_GRADIENT
#undef CAM_MM_OPT_CIRCLE5

#define CAM_MM_OPT_SQUARE3
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 3
#define CAM_MM_DO_EROSION
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_GRADIENT
#undef CAM_MM_OPT_SQUARE3

// For Circle7 binary erosion
static const int camCircle7StructElt[7][7]={
    {0,0,1,1,1,0,0},
    {0,1,1,1,1,1,0},
    {1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1},
    {0,1,1,1,1,1,0},
    {0,0,1,1,1,0,0}
};

#define CAM_MM_OPT_CIRCLE7
#undef CAM_MM_NEIGHB
#define CAM_MM_NEIGHB 7
#define CAM_MM_DO_EROSION
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_DO_DILATION
#define CAM_MM_GRADIENT
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_GRADIENT
#define CAM_MM_FINDLOCALMAXIMA
#include "cam_morphomaths_code_opt.c"
#undef CAM_MM_FINDLOCALMAXIMA
#undef CAM_MM_OPT_CIRCLE7

#endif

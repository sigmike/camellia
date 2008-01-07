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

#include "camellia.h"
#include "camellia_internals.h"

int camHarris(CamImage *source, CamKeypoints *points, int k)
{
    CamImage X, Y, XY, R, tr;
    int i, width, height;
    CamInternalROIPolicyStruct iROI;
    CamArithmParams params;

    CAM_CHECK(camHarris, camInternalROIPolicy(source, NULL, &iROI, 1));
    CAM_CHECK_ARGS(camHarris, (source->depth & CAM_DEPTH_MASK) >= 8);
    CAM_CHECK_ARGS(camHarris, points->allocated != 0);
    width = iROI.srcroi.width;
    height = iROI.srcroi.height;

    // Bag allocation
    if (points->bag == NULL) {
	points->bag = (CamKeypoint*)malloc(sizeof(CamKeypoint) * points->allocated);
	for (i = 0; i < points->allocated; i++) {
	    points->keypoint[i] = &points->bag[i];
	    points->bag[i].set = points;
	}
    }
    points->nbPoints = 0;

    params.operation = CAM_ARITHM_MUL;

    camAllocateImage(&X, width, height, CAM_DEPTH_16S);
    X.mask = iROI.mask;
    camAllocateImage(&Y, width, height, CAM_DEPTH_16S);
    Y.mask = iROI.mask;
    camAllocateImage(&XY, width, height, CAM_DEPTH_16S);
    XY.mask = iROI.mask;
    camAllocateImage(&R, width, height, CAM_DEPTH_16S);
    R.mask = iROI.mask;
    camAllocateImage(&tr, width, height, CAM_DEPTH_16S);
    tr.mask = iROI.mask;

    camFixedFilter(source, &X, CAM_SCHARR_V); 
    camFixedFilter(source, &Y, CAM_SCHARR_H); 
    params.c1 = 4;
    camDyadicArithm(&X, &Y, &XY, &params);
    camFixedFilter(&XY, &XY, CAM_GAUSSIAN_7x7);
    params.c1 = 8;
    camDyadicArithm(&XY, &XY, &XY, &params);
    params.c1 = 4;
    camDyadicArithm(&X, &X, &X, &params);
    camDyadicArithm(&Y, &Y, &Y, &params);
    camFixedFilter(&X, &X, CAM_GAUSSIAN_7x7);
    camFixedFilter(&Y, &Y, CAM_GAUSSIAN_7x7);
    params.c1 = 8;
    camDyadicArithm(&X, &Y, &R, &params);
    camSub(&R, &XY, &R);
    // Compute the trace
    if (k) {
	camAdd(&X, &Y, &tr);
	camDyadicArithm(&tr, &tr, &tr, &params);
	params.operation = CAM_ARITHM_WEIGHTED_SUM;
	params.c1 = 1024;
	params.c2 = -k;
	params.c3 = 10;
	camDyadicArithm(&R, &tr, &R, &params);
    }
    camFindLocalMaximaCircle7(&R, points, 0);
/*
    for (i=0; i<points->nbPoints; i++) {
	camPlot(&A, points->keypoint[i].x, points->keypoint[i].y, 32767, CAM_CROSS);
    }
    camAbs(&A, &A);
    camSavePGM(&A, "output/test_harris.pgm");
*/

    camDeallocateImage(&X);
    camDeallocateImage(&Y);
    camDeallocateImage(&XY);
    camDeallocateImage(&R);
    camDeallocateImage(&tr);

    camInternalROIPolicyExit(&iROI);
    return 1;
}



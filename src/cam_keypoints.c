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

    Redistribution and use in integral and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of integral code must retain the above copyright
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

/* CamKeypoints implementation
 * C code */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795 
#endif
#include "camellia.h"
#include "camellia_internals.h"
#ifdef __SSE2__
#include <emmintrin.h>
#endif

#define SEPARATED_NORMALIZATION
//#define SURF_DESCRIPTOR
static int camPatchSizeParam = 8*3;
static int camSigmaParam = 5;
static int camThreshGradientParam = 128;

int camKeypointsSetParameters(int patchSize, int sigma, int threshGradient)
{
    camPatchSizeParam = patchSize;
    camSigmaParam = sigma;
    camThreshGradientParam = threshGradient;
    return 1;
}

// Feature points allocation
int camAllocateKeypoints(CamKeypoints *fpoints, int nbPoints)
{
    CAM_CHECK_ARGS(camAllocateKeypoints, fpoints != NULL);
    fpoints->nbPoints = 0;
    fpoints->cx = 0;
    fpoints->cy = 0;
    fpoints->keypoint = (CamKeypoint**)malloc(sizeof(CamKeypoint*) * nbPoints);
    fpoints->bag = NULL;
    if (fpoints->keypoint == NULL) {
	fpoints->allocated = 0;
	camError("camAllocateKeypoints", "Memory allocation error");
	return 0;
    }
    fpoints->allocated = nbPoints;
    return 1;
}

// Feature points reallocation
int camReallocateKeypoints(CamKeypoints *fpoints, int nbPoints)
{
    CAM_CHECK_ARGS(camKeypointsReallocate, fpoints != NULL);
    if (fpoints->keypoint == NULL) {
	return camAllocateKeypoints(fpoints, nbPoints);
    }
    fpoints->keypoint = (CamKeypoint**)realloc(fpoints->keypoint, sizeof(CamKeypoint*) * nbPoints);
    if (fpoints->keypoint == NULL) {
	fpoints->nbPoints = 0;
	fpoints->allocated = 0;
	camError("camKeypointsReallocate", "Memory allocation error");
	return 0;
    }
    fpoints->allocated = nbPoints;
    return 1;
}

/// Feature points deallocation
int camFreeKeypoints(CamKeypoints *fpoints)
{
    CAM_CHECK_ARGS(camKeypointsDeallocate, fpoints != NULL);
    if (fpoints->keypoint) free(fpoints->keypoint);
    if (fpoints->bag) free(fpoints->bag);
    fpoints->keypoint = NULL;
    fpoints->bag = NULL;
    fpoints->nbPoints = 0;
    fpoints->allocated = 0;
    return 1;
}

#define CAM_INTEGRAL(ptr, oLeft, oTop, oRight, oBottom) \
    ( *(ptr + oRight + oBottom) - *(ptr + oLeft + oBottom) - *(ptr + oRight + oTop) + *(ptr + oLeft + oTop) )

static const int CamScale[] = {3, 5, 7, 9, 13, 17, 25, 33, 49, 65};
static const int CamOffset[][2] = {
	{2, 1}, {3, 2}, {4, 2}, {6, 3}, {9, 4}, {11, 6}, {17, 8}, {22, 11}, {33, 16}, {43, 22}		
    };
static const int CamSampling[] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3};
static const int CamSamplingOffset[] = {0, 0, 0, 0, 0, 0, 1, 1, 3, 3};

int camFastHessianDetectorFixedScale(CamImage *integral, CamImage *dest, int scale)
{
    int x, y;
    int width, height;
    unsigned long *srcptr, *tmpsrcptr;
    signed short *dstptr, *tmpdstptr;
    CamInternalROIPolicyStruct iROI;
    int acc=0, det, coeff;
    int Dxx, Dyy, Dxy, tmp1, tmp2;
    int multiplier, shift;
    int offset0, offset1, offset2, offset3, offset4, offset5, offset6, offset7, offset8, offset9;
    int offset10, offset11, offset12, offset13, offset14, offset15, offset16, offset17, offset18, offset19;
    int startx2, endx2;
    int sampling_mask;
    DECLARE_MASK_MANAGEMENT;
#ifdef ELIMINATE_EDGE_RESPONSE
    int r;
#endif

    CAM_CHECK_ARGS2(camFastHessianDetectorFixedScale, integral->imageData != NULL, "source integral image is not allocated");
    CAM_CHECK_ARGS2(camFastHessianDetectorFixedScale, (scale >=0 && scale < sizeof(CamScale) / sizeof(CamScale[0])), "invalid scale parameter"); 
    if (dest->imageData==NULL) {
        // Automatic allocation
	CAM_CHECK(camFastHessianDetectorFixedScale, camInternalROIPolicy(integral, NULL, &iROI, 0));
        camAllocateImage(dest, iROI.srcroi.width >> CamSampling[scale], iROI.srcroi.height >> CamSampling[scale], CAM_DEPTH_16U);
    }
    CAM_CHECK(camFastHessianDetectorFixedScale, camInternalROIPolicy(integral, dest, &iROI, CAM_MASK_SUPPORT | CAM_NO_ROI_INTERSECTION));
    CAM_CHECK_ARGS(camFastHessianDetectorFixedScale, (integral->depth & CAM_DEPTH_MASK) == 32);
    CAM_CHECK_ARGS(camFastHessianDetectorFixedScale, (dest->depth & CAM_DEPTH_MASK) == 16);

    // ROI (Region Of Interest) management
    width = iROI.srcroi.width;
    height = iROI.srcroi.height;
    CAM_CHECK_ARGS2(camFastHessianDetectorFixedScale, (width & 3) == 0 && (height & 3) == 0, "ROI width and height must be multiple of 4");
    srcptr = (unsigned long *)iROI.srcptr;
    dstptr = (unsigned short *)iROI.dstptr;

    INIT_MASK_MANAGEMENT;
    
    // Algorithm initialization
    // Multiplier : for 9, divider should be 9x9 = 81.
    // In order to stay in 8 bits range, one should multiply by 1/81 = 0.0123
    // Which is equivalent to 809/65536, so we keep 809 as the multiplier for scale 9.
    // Formula is 65536/(s*s)
    // multiplier = 65536 / (CamScale[scale] * CamScale[scale]);  
    // It is made more accurate by using specifically the surfaces used for computation
    multiplier = 65536 / ((CamScale[scale]*3 - 2 * CamOffset[scale][0]) * CamScale[scale]);  
    shift = 18 - 2 * scale;
    if (shift >= 0) 
    	coeff = (multiplier * multiplier) >> shift;
    else
	coeff = (multiplier * multiplier) << (-shift); 
    // printf("scale #%d, multiplier = %d, coeff = %d\n", scale, multiplier, coeff);
    // Fill the offset tables
    offset0 = -CamScale[scale]*3 / 2 - 1 + CamOffset[scale][0];
    offset1 = CamScale[scale]*3 / 2 - CamOffset[scale][0];
    offset2 = (-CamScale[scale]*3 / 2 - 1) * iROI.srclinc;
    offset3 = offset2 + iROI.srclinc * CamScale[scale];
    offset4 = offset3 + iROI.srclinc * CamScale[scale];
    offset5 = offset4 + iROI.srclinc * CamScale[scale];
    offset6 = -CamScale[scale]*3 / 2 - 1;
    offset7 = offset6 + CamScale[scale];
    offset8 = offset7 + CamScale[scale];
    offset9 = offset8 + CamScale[scale];
    offset10 = iROI.srclinc * offset0;
    offset11 = iROI.srclinc * offset1;
    offset12 = -CamScale[scale]*3 / 2 - 1 + CamOffset[scale][1];
    offset13 = iROI.srclinc * offset12;
    offset14 = offset12 + CamScale[scale];
    offset15 = iROI.srclinc * offset14;
    offset16 = CamScale[scale]*3 / 2 - CamOffset[scale][1];
    offset17 = iROI.srclinc * offset16;
    offset18 = offset16 - CamScale[scale];
    offset19 = iROI.srclinc * offset18;
    // Other
    sampling_mask = (1 << CamSampling[scale]) - 1;

    // Main loop
    for (y = 0; y < height; y++) {
       	tmpsrcptr = srcptr;
	tmpdstptr = dstptr;
	BEGIN_MASK_MANAGEMENT(
	    srcptr = tmpsrcptr + startx + CamSamplingOffset[scale];
	    dstptr = tmpdstptr + (startx >> CamSampling[scale]);
	)
	    if ((y & sampling_mask) == CamSamplingOffset[scale]) {
		// Check border limits
		startx2 = startx;
		endx2 = endx;
		if ((y <= CamScale[scale]*3 / 2) || (y >= height - CamScale[scale]*3 / 2)) {
		    startx2 = endx;
		    endx2 = endx; // Skip the line
		} else {
		    if (startx <= CamScale[scale]*3 / 2) {
			startx2 = CamScale[scale]*3 / 2 + 1;
		    }
		    if (endx >= width - CamScale[scale]*3 / 2) {
			endx2 = width - CamScale[scale]*3 / 2 - 1;
		    }
		}

		for (x = startx; x < startx2 ; x += (1 << CamSampling[scale]), srcptr += (1 << CamSampling[scale]), dstptr++ ) *dstptr = 0;
		for (; x < endx2; x += (1 << CamSampling[scale]), srcptr += (1 << CamSampling[scale]), dstptr++) {
		    //printf("(%d, %d)\n", x, y);
		    tmp1 = CAM_INTEGRAL(srcptr, offset0, offset3, offset1, offset4);
		    Dxx = CAM_INTEGRAL(srcptr, offset0, offset2, offset1, offset5) -
			(tmp1 + tmp1 + tmp1);
		    Dxx = Dxx >> scale;
		    tmp2 = CAM_INTEGRAL(srcptr, offset7, offset10, offset8, offset11); 
		    Dyy = CAM_INTEGRAL(srcptr, offset6, offset10, offset9, offset11) -
			(tmp2 + tmp2 + tmp2);
		    Dyy = Dyy >> scale;
		    Dxy = CAM_INTEGRAL(srcptr, offset12, offset13, offset14, offset15) -
			CAM_INTEGRAL(srcptr, offset18, offset13, offset16, offset15) -
			CAM_INTEGRAL(srcptr, offset12, offset19, offset14, offset17) +
			CAM_INTEGRAL(srcptr, offset18, offset19, offset16, offset17); 
		    Dxy = Dxy >> scale;
		    // det = Dxx * Dyy - 0.81 * Dxy^2
		    // det should be 26 bits max
		    det = Dxx * Dyy - ((13 * Dxy * Dxy) >> 4);
		    if (det <= 0) det = 0;
		    else {
			det = (det * coeff) >> 6;
#ifdef ELIMINATE_EDGE_RESPONSE
			r = (Dxx + Dyy) * (Dxx + Dyy) / det;
			if (r > 10)
			    det = 0;
			else 
#endif
			    det >>= 10;
			    // det is on 16 bits max
		    }
		    acc += det;
		    *dstptr = (unsigned short)det;
		}
		for (; x < endx ; x += (1 << CamSampling[scale]), srcptr += (1 << CamSampling[scale]), dstptr++ ) *dstptr = 0;
	    }
        END_MASK_MANAGEMENT;
	srcptr = (unsigned long*)(((char*)tmpsrcptr) + integral->widthStep);
	if ((y & sampling_mask) == CamSamplingOffset[scale])
	    dstptr = (unsigned short*)(((char*)tmpdstptr) + dest->widthStep);
    }

    camInternalROIPolicyExit(&iROI);
    return acc;    
}

int camBuildGaussianFilter(CamImage *image, double sigma)
{
    int x,y;
    int width,height;
    signed short *imptr,*tmpptr;
    CamInternalROIPolicyStruct iROI;
    DECLARE_MASK_MANAGEMENT;
    double cx, cy;

    CAM_CHECK(camBuildGaussianFilter,camInternalROIPolicy(image, NULL, &iROI, 1));
    CAM_CHECK_ARGS(camBuildGaussianFilter,(image->depth & CAM_DEPTH_MASK) == 16);

    // ROI (Region Of Interest) management
    width = iROI.srcroi.width;
    height = iROI.srcroi.height;
    imptr = (signed short*)iROI.srcptr;
    cx = iROI.srcroi.xOffset + (iROI.srcroi.width / 2.0) - 0.5;
    cy = iROI.srcroi.yOffset + (iROI.srcroi.height / 2.0) - 0.5;

    INIT_MASK_MANAGEMENT;

    for (y = 0; y < height; y++) {
	tmpptr = imptr; 
        BEGIN_MASK_MANAGEMENT(imptr = tmpptr + startx * iROI.srcinc;)
            for (x = startx; x < endx; x++, imptr += iROI.srcinc) {
                *imptr = (signed short)(32767 * exp(-((x-cx) * (x-cx) + (y-cy) * (y-cy)) / (2 * sigma * sigma)));
            }
        END_MASK_MANAGEMENT;
        imptr = tmpptr + iROI.srclinc;
    }
    
    camInternalROIPolicyExit(&iROI);
    return 1;    
}

#define CAM_NB_SECTORS 36
#define CAM_ORIENTATION_STAMP_SIZE 30

int camKeypointsInternalsPrepareOrientation()
{
    int i, s, x, y;
    double angle, v1[2], v2[2], v3[2];
    double dp1, dp2; // Dot products
    int xok[CAM_ORIENTATION_STAMP_SIZE * CAM_ORIENTATION_STAMP_SIZE], yok[CAM_ORIENTATION_STAMP_SIZE * CAM_ORIENTATION_STAMP_SIZE], nbok;
    FILE *handle;
    CamImage dummy;
#define OFFSET(x,y) (dummy.widthStep * (y) / 2 + (x)) 

    camAllocateImage(&dummy, CAM_ORIENTATION_STAMP_SIZE, CAM_ORIENTATION_STAMP_SIZE, CAM_DEPTH_16S);
    handle = fopen("src/cam_keypoints_sectors_code.c", "wt");
    fprintf(handle, "const int CamKeypointSectors[CAM_NB_SECTORS][MAX_PIX_PER_SECTOR] = {\n");
    for (s = 0; s < CAM_NB_SECTORS; s++) {
	nbok = 0;
	// Try to find out which points lie within the sector
	angle = 2 * M_PI * s / CAM_NB_SECTORS;
	v1[0] = cos(angle);
	v1[1] = sin(angle);
	angle = 2 * M_PI * (s + 1) / CAM_NB_SECTORS;
	v2[0] = cos(angle);
	v2[1] = sin(angle);
	// Test all the points...
	for (y = 0; y < CAM_ORIENTATION_STAMP_SIZE; y++) {
	    for (x = 0; x < CAM_ORIENTATION_STAMP_SIZE; x++) {
		v3[0] = x - (CAM_ORIENTATION_STAMP_SIZE/2 - 0.5); 
		v3[1] = (CAM_ORIENTATION_STAMP_SIZE/2 - 0.5) - y; // y goes upward
		if (v3[0] * v3[0] + v3[1] * v3[1] < 14 * 14) {
		    // This pixel has a good radius
		    // Does it lie within this sector ?
		    dp1 = - v3[0] * v1[1] + v3[1] * v1[0];
		    dp2 = - v3[0] * v2[1] + v3[1] * v2[0];
		    if (dp1 > 0 && dp2 < 0) {
		        // Yes, it does ! Record it !
			xok[nbok] = x; yok[nbok] = y; nbok++;
		    }	
		}
	    }
	}
	// OK. We have our set of points for the current sector.
	// Record the data
	fprintf(handle, "\t{");
	for (i = 0; i < nbok; i++) {
	    fprintf(handle, "%d, ", OFFSET(xok[i], yok[i]));
	}
	fprintf(handle, "-1}");
	if (s != CAM_NB_SECTORS - 1) fprintf(handle, ",\n");
    }
    fprintf(handle, "\n};\n");
    fclose(handle);
    camDeallocateImage(&dummy);
    return 1;
}

#define MAX_PIX_PER_SECTOR 20 
#include "cam_keypoints_sectors_code.c"

int camSortStrength(const void *p1x, const void *p2x)
{
    int *p1 = (int*)p1x;
    int *p2 = (int*)p2x;
    if (p1[0] < p2[0]) return 1;
    if (p1[0] == p2[0]) return 0;
    return -1;
}

int camKeypointOrientation(CamImage *source, CamKeypoint *point, CamImage *filter, CamKeypoints *points)
{
    CamWarpingParams params;
    CamImage scaled, filtered_h, filtered_v;
    CamArithmParams params2;

    int i, c, d, x, y;
    int scale, angle, angle1, angle2, dangle;
    int vx[CAM_NB_SECTORS], vy[CAM_NB_SECTORS];
    int vxc[CAM_NB_SECTORS], vyc[CAM_NB_SECTORS];
    int strength[CAM_NB_SECTORS], stgx[CAM_NB_SECTORS * 2];
    int maximum;

    const int xp[4] = {-1, 1, 1, -1};
    const int yp[4] = {-1, -1, 1, 1};

    camAllocateImage(&scaled, CAM_ORIENTATION_STAMP_SIZE, CAM_ORIENTATION_STAMP_SIZE, source->depth);

    // Scale the image
    params.perspective=0;
    params.interpolation=1;
    scale = (point->scale * camPatchSizeParam * 3) << 9;
    for (i = 0; i < 4; i++) {
	params.p[i].x = xp[i] * scale;
 	params.p[i].y = yp[i] * scale;
	params.p[i].x += (point->x << 16) + 32768;
	params.p[i].y += (point->y << 16) + 32768;
    }
    camWarping(source, &scaled, &params);
    
#if 0
    static int nbx = 0;
    char strx[256];
    sprintf(strx, "output/keypoints_rotation_%d.pgm", nbx++);
    camSavePGM(&scaled, strx);
#endif

    // We now have the scaled image
    // Let's filter it...
    camAllocateImage(&filtered_h, CAM_ORIENTATION_STAMP_SIZE, CAM_ORIENTATION_STAMP_SIZE, CAM_DEPTH_16S);
    camAllocateImage(&filtered_v, CAM_ORIENTATION_STAMP_SIZE, CAM_ORIENTATION_STAMP_SIZE, CAM_DEPTH_16S);
    camFixedFilter(&scaled, &filtered_h, CAM_SCHARR_H);
    camFixedFilter(&scaled, &filtered_v, CAM_SCHARR_V);
    params2.operation = CAM_ARITHM_MUL;
    params2.c1 = 16;
    camDyadicArithm(&filtered_h, filter, &filtered_h, &params2);
    camDyadicArithm(&filtered_v, filter, &filtered_v, &params2);

    // Great. Let's compute the sum of gradient for all sectors...
    for (i = 0; i < CAM_NB_SECTORS; i++) {
	vx[i] = 0; vy[i] = 0;
	c = 0;
	while (CamKeypointSectors[i][c] != -1) {
	    vx[i] += *(((signed short*)filtered_v.imageData) + CamKeypointSectors[i][c]);
	    vy[i] -= *(((signed short*)filtered_h.imageData) + CamKeypointSectors[i][c]);
	    c++;
	}
    }

    // Accumulation into pi/3 sectors
    for (i = 0; i < CAM_NB_SECTORS; i++) {
	vxc[i] = vx[i];
	vyc[i] = vy[i];
	d = i + 1;
	if (d == CAM_NB_SECTORS) d = 0;
	for (c = 1; c < CAM_NB_SECTORS / 6; c++) {
	    vxc[i] += vx[d];
	    vyc[i] += vy[d];
	    d++;
	    if (d == CAM_NB_SECTORS) d = 0;
	}
    }

#if 0
    FILE *handle;
    static int nb = 0;
    char str[256];
    sprintf(str, "output/keypoints_orientation_%d.txt", nb++);
    handle = fopen(str, "wt");
    for (i = 0; i < CAM_NB_SECTORS; i++) {
	fprintf(handle, "%d\n", strength[i]);
    }
    fclose(handle);
#endif

    camDeallocateImage(&scaled);
    camDeallocateImage(&filtered_h);
    camDeallocateImage(&filtered_v);
    
    // What is the best ?
    for (i = 0; i < CAM_NB_SECTORS; i++) {
	x = vxc[i] >> 4;
	y = vyc[i] >> 4;
	strength[i] = stgx[i * 2] = x * x + y * y;
	stgx[i * 2 + 1] = i;
    }
    qsort(stgx, CAM_NB_SECTORS, sizeof(int) * 2, camSortStrength);
    maximum = stgx[1];
    point->angle = (int)floor(atan2(vyc[maximum], vxc[maximum]) * 360 / (2 * M_PI) + 180 + 0.5);

    // Is there another angle that would deserve to be turned into a keypoint ?
    for (i = 1; i < CAM_NB_SECTORS; i++) {
	if (stgx[i * 2] > stgx[0] * 2 / 3) {
	    // Check that it is a local maximum
	    d = stgx[i * 2 + 1]; // Retrieve the index
	    if (d == 0) c = CAM_NB_SECTORS - 1; else c = d - 1;
	    if (stgx[i * 2] > strength[c]) {
		if (d == CAM_NB_SECTORS - 1) c = 0; else c = d + 1;
		if (stgx[i * 2] > strength[c]) {
		    // OK. This is a local maximum
		    angle = (int)floor(atan2(vyc[d], vxc[d]) * 360 / (2 * M_PI) + 180 + 0.5);
		    if (angle > point->angle) {
			angle1 = point->angle;
			angle2 = angle;
		    } else {
			angle1 = angle;
			angle2 = point->angle;
		    }
		    dangle = 360 - angle2 + angle1;
		    if (angle2 - angle1 < dangle) dangle = angle2 - angle1;
		    if (dangle > 60) {
			if (dangle >= 170 && dangle <= 190) {
			    // This is the opposite
			    // Let's keep only one of them
			    if (vyc[maximum] < 0) {
				point->angle = angle;
			    }
			} else {
			    // Add a new keypoint 
			    if (points->keypoint[points->nbPoints - 1] - points->bag != points->allocated) {
				points->keypoint[points->nbPoints] = points->keypoint[points->nbPoints - 1] + 1;
				*points->keypoint[points->nbPoints] = *point;
				points->keypoint[points->nbPoints]->angle = angle;
				points->nbPoints++;
			    }
			}
			break;
		    } 
		}
	    }
	    
	} else break;
    }
    return 1;
}

static int camFPNbAttPoints[20 * 20], camFPAttPoint[20 * 20 * 4], camFPCoeff[20 * 20 * 4];

void camKeypointsInternalsPrepareDescriptor()
{
    int x, y, i;

    for (i = 0; i < 20 * 20 * 4; i++) {
	camFPAttPoint[i] = 0;
	camFPCoeff[i] = 0;
    }
    // For all the camFPAttPoints, find the attraction camFPAttPoints
    // and compute the bilinear interpolation camFPCoefficients
    for (y = 0, i = 0; y < 20; y++) {
	for (x = 0; x < 20; x++, i++) {
	    if (x < 2) {
		if (y <= 2) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = 0;
		    camFPCoeff[i * 4] =  25;
		} else if (y >= 17) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = 12;
		    camFPCoeff[i * 4] = 25;
		} else if ((y - 2) % 5 == 0) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = ((y - 2) / 5) * 4;
		    camFPCoeff[i * 4] = 25;
		} else {
		    camFPNbAttPoints[i] = 2;
		    camFPAttPoint[i * 4] = ((y - 2) / 5) * 4;
		    camFPAttPoint[i * 4 + 1] = ((y - 2) / 5 + 1) * 4;
		    camFPCoeff[i * 4] = 25 - 5 * ((y - 2) % 5);
		    camFPCoeff[i * 4 + 1] = 5 * ((y - 2) % 5);
		}
	    } else if (x > 17) {
		if (y <= 2) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = 3;
		    camFPCoeff[i * 4] =  25;
		} else if (y >= 17) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = 15;
		    camFPCoeff[i * 4] = 25;
		} else if ((y - 2) % 5 == 0) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = ((y - 2) / 5) * 4 + 3;
		    camFPCoeff[i * 4] = 25;
		} else {
		    camFPNbAttPoints[i] = 2;
		    camFPAttPoint[i * 4] = ((y - 2) / 5) * 4 + 3;
		    camFPAttPoint[i * 4 + 1] = ((y - 2) / 5 + 1) * 4 + 3;
		    camFPCoeff[i * 4] = 25 - 5 * ((y - 2) % 5);
		    camFPCoeff[i * 4 + 1] = 5 * ((y - 2) % 5);
		}
	    } else if ((x - 2) % 5 == 0) {
		if (y <= 2) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = (x - 2) / 5;
		    camFPCoeff[i * 4] =  25;
		} else if (y >= 17) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = 12 + (x - 2) / 5;
		    camFPCoeff[i * 4] = 25;
		} else if ((y - 2) % 5 == 0) {
		    camFPNbAttPoints[i] = 1;
		    camFPAttPoint[i * 4] = ((y - 2) / 5) * 4 + (x - 2) / 5;
		    camFPCoeff[i * 4] = 25;
		} else {
		    camFPNbAttPoints[i] = 2;
		    camFPAttPoint[i * 4] = ((y - 2) / 5) * 4 + (x - 2) / 5;
		    camFPAttPoint[i * 4 + 1] = ((y - 2) / 5 + 1) * 4 + (x - 2) / 5;
		    camFPCoeff[i * 4] = 25 - 5 * ((y - 2) % 5);
		    camFPCoeff[i * 4 + 1] = 5 * ((y - 2) % 5);
		}
	    } else {
		if (y <= 2) {
		    camFPNbAttPoints[i] = 2;
		    camFPAttPoint[i * 4] = (x - 2) / 5;
		    camFPAttPoint[i * 4 + 1] = (x - 2) / 5 + 1;
		    camFPCoeff[i * 4] = 25 - 5 * ((x - 2) % 5);
		    camFPCoeff[i * 4 + 1] = 5 * ((x - 2) % 5);
		} else if (y >= 17) {
		    camFPNbAttPoints[i] = 2;
		    camFPAttPoint[i * 4] = 12 + (x - 2) / 5;
		    camFPAttPoint[i * 4 + 1] = 13 + (x - 2) / 5;
		    camFPCoeff[i * 4] = 25 - 5 * ((x - 2) % 5);
		    camFPCoeff[i * 4 + 1] = 5 * ((x - 2) % 5);
		} else if ((y - 2) % 5 == 0) {
		    camFPNbAttPoints[i] = 2;
		    camFPAttPoint[i * 4] = ((y - 2) / 5) * 4 + (x - 2) / 5;
		    camFPAttPoint[i * 4 + 1] = ((y - 2) / 5) * 4 + (x - 2) / 5 + 1;
		    camFPCoeff[i * 4] = 25 - 5 * ((x - 2) % 5);
		    camFPCoeff[i * 4 + 1] = 5 * ((x - 2) % 5);
		} else {
		    camFPNbAttPoints[i] = 4;
		    camFPAttPoint[i * 4] = ((y - 2) / 5) * 4 + (x - 2) / 5;
		    camFPAttPoint[i * 4 + 1] = ((y - 2) / 5) * 4 + (x - 2) / 5 + 1;
		    camFPAttPoint[i * 4 + 2] = ((y - 2) / 5 + 1) * 4 + (x - 2) / 5;
		    camFPAttPoint[i * 4 + 3] = ((y - 2) / 5 + 1) * 4 + (x - 2) / 5 + 1;
		    camFPCoeff[i * 4] = (5 - ((y - 2) % 5)) * (5 - ((x - 2) % 5));
		    camFPCoeff[i * 4 + 1] = (5 - ((y - 2) % 5)) * ((x - 2) % 5);
		    camFPCoeff[i * 4 + 2] = ((y - 2) % 5) * (5 - ((x - 2) % 5));
		    camFPCoeff[i * 4 + 3] = (5 - ((y - 2) % 5)) * (5 - ((x - 2) % 5));
		}
	    }
	}
    }
}

int camKeypointsDescriptor(CamImage *source, CamKeypoint *point, CamImage *filter, int option)
{
    CamWarpingParams params;
    CamImage rotated, filtered_h, filtered_v;
    CamArithmParams params2;

    int i, j, x, y, sum;
    int costheta = (int)(cos(-point->angle * 2 * M_PI / 360) * 65536.0);
    int sintheta = (int)(sin(-point->angle * 2 * M_PI / 360) * 65536.0);
    int scale, channel, start, end;
    CamROI roix;

#ifdef SURF_DESCRIPTOR
    int dx, dy, abs_dx, abs_dy;
    CamROI roi;
#else
    int coeff, idx, dxt[16], dyt[16], abs_dxt[16], abs_dyt[16];
    signed short val_h, val_v, val2_h, val2_v, *imptr_h, *imptr_v, *tmpimptr_h, *tmpimptr_v; 
#endif

    const int xp[4] = {-1, 1, 1, -1};
    const int yp[4] = {-1, -1, 1, 1};

#if 0
    static int nb = 0;
    char filename[256];
    FILE *handle;
#endif
    
    if (source->nChannels == 3) {
	camAllocateYUVImage(&rotated, 20, 20);
    } else {
	camAllocateImage(&rotated, 20, 20, source->depth);
    }

    // Rotate the image
    params.perspective=1;
    params.interpolation=1;
    scale = (point->scale * camPatchSizeParam) >> 5;
    for (i = 0; i < 4; i++) {
	params.p[i].x = costheta * xp[i] - sintheta * yp[i];
	params.p[i].y = sintheta * xp[i] + costheta * yp[i];
	params.p[i].x *= scale;
	params.p[i].y *= scale;
	params.p[i].x += (point->x << 16) + 32768;
	params.p[i].y += (point->y << 16) + 32768;
    }
    camWarping(source, &rotated, &params);
    
#if 0
    char filename[256];
    static int nb = 0;
    sprintf(filename, "output/rotated%d.pgm", nb++);
    camSavePGM(&rotated, filename);
#endif
    
    // We now have the rotated image
    // Let's filter it...
    camAllocateImage(&filtered_h, 20, 20, CAM_DEPTH_16S);
    camAllocateImage(&filtered_v, 20, 20, CAM_DEPTH_16S);
    roix.xOffset = 0; roix.yOffset = 0; roix.width = 20; roix.height = 20;
    rotated.roi=&roix;

    for (channel = 0; channel < source->nChannels; channel++) {
	roix.coi = channel + 1;
	camFixedFilter(&rotated, &filtered_h, CAM_SCHARR_H);
	camFixedFilter(&rotated, &filtered_v, CAM_SCHARR_V);
	params2.operation = CAM_ARITHM_MUL;
	params2.c1 = 16;
	camDyadicArithm(&filtered_h, filter, &filtered_h, &params2);
	camDyadicArithm(&filtered_v, filter, &filtered_v, &params2);

	/*
	   camAbs(&filtered_h, &filtered_h);
	   camAbs(&filtered_v, &filtered_v);
	   camSavePGM(&filtered_h, "output/filtered_h.pgm");
	   camSavePGM(&filtered_v, "output/filtered_v.pgm");
	*/

#ifdef SURF_DESCRIPTOR
	filtered_h.roi = &roi;
	filtered_v.roi = &roi;
	roi.width = 5;
	roi.height = 5;

	i = 0;
	for (y = 0; y < 4; y++) {
	    for (x = 0; x < 4; x++) {
		roi.xOffset = x * 5;
		roi.yOffset = y * 5;
		dx = camSumOfPixels(&filtered_v);
		dy = camSumOfPixels(&filtered_h);
		abs_dx = camAbs(&filtered_v, &filtered_v);
		abs_dy = camAbs(&filtered_h, &filtered_h); 
		point->descriptor[i] = dx;
		point->descriptor[32 + i] = abs_dx; 
		i++;
		point->descriptor[i] = dy;
		point->descriptor[32 + i] = abs_dy;
		i++;
	    }
	}
	point->size = 64;
#else
	for (i = 0; i < 16; i++) {
	    dxt[i] = 0; dyt[i] = 0; abs_dxt[i] = 0; abs_dyt[i] = 0;
	}

	tmpimptr_h = (signed short*)filtered_h.imageData;
	tmpimptr_v = (signed short*)filtered_v.imageData;
	for (y = 0, i = 0; y < 20; y++) {
	    imptr_h = tmpimptr_h;
	    imptr_v = tmpimptr_v;
	    for (x = 0; x < 20; x++, i++, imptr_h++, imptr_v++) {
		val_h = *imptr_h;
		val_v = *imptr_v;
		for (j = 0; j < camFPNbAttPoints[i]; j++) {
		    idx = camFPAttPoint[i * 4 + j];
		    coeff = camFPCoeff[i * 4 + j];
		    val2_h = coeff * val_h;
		    val2_v = coeff * val_v;
		    dxt[idx] += val2_v;
		    dyt[idx] += val2_h;
		    if (channel == 0) {
			abs_dxt[idx] += abs(val2_v);
			abs_dyt[idx] += abs(val2_h);
		    }
		}
	    }
	    tmpimptr_h = (signed short*)(((char*)tmpimptr_h) + filtered_h.widthStep);
	    tmpimptr_v = (signed short*)(((char*)tmpimptr_v) + filtered_v.widthStep);
	}

	if (channel == 0) {
	    for (j = 0, i = 0; j < 16; j++) {
		point->descriptor[i] = dxt[j];
		point->descriptor[32 + i] = abs_dxt[j]; 
		i++;
		point->descriptor[i] = dyt[j];
		point->descriptor[32 + i] = abs_dyt[j];
		i++;
	    }
	    point->size = 64;
	} else {
	    for (j = 0; j < 16; j++) {
		point->descriptor[point->size++] = dxt[j];
		point->descriptor[point->size++] = dyt[j];
	    }
	}
#endif
    }

    // Normalization
#ifdef SEPARATED_NORMALIZATION
    for (j = 0; j < ((source->nChannels == 1)?1:2); j++) {
	start = j << 6;
	end = start + 64;
#else
    {
	start = 0;
	end = point->size;
#endif

	sum = 0;
	for (i = start; i < end; i++) {
	    sum += abs(point->descriptor[i]);
	}
	if (sum != 0) sum = (1 << 30) / sum;
	for (i = start; i < end; i++) {
	    point->descriptor[i] = (point->descriptor[i] * sum) >> 6;
	}	
    }

#if 0
    // Save the signature
    sprintf(filename, "output/signature%d.txt", nb++);
    handle = fopen(filename, "wt");
    for (i = 0; i < point->size; i++) {
	fprintf(handle, "%d\n", point->descriptor[i]);
    }
    fclose(handle);
#endif

    camDeallocateImage(&rotated);
    camDeallocateImage(&filtered_h);
    camDeallocateImage(&filtered_v);

    return 1;
}

int camSortKeypoints(const void *p1x, const void *p2x)
{
    CamKeypoint **p1 = (CamKeypoint**)p1x;
    CamKeypoint **p2 = (CamKeypoint**)p2x;
    if ((*p1)->value < (*p2)->value) return 1;
    if ((*p1)->value == (*p2)->value) return 0;
    return -1;
}

int camFastHessianDetector(CamImage *source, CamKeypoints *points, int threshold, int options)
{
    CamImage integral;
    CamImage results[sizeof(CamScale) / sizeof(CamScale[0])];
    int width, height;
    int i, j, k, scale;
    int pNbPoints;
    CamInternalROIPolicyStruct iROI;
    CamROI *roi, roix;
    const int nbScales = sizeof(CamScale) / sizeof(CamScale[0]);

    int x, y, d, thr;
    int widthClusters, heightClusters;
    int nbClusters, cluster;
    int *nbPerCluster;
    CamKeypoint ***firstPoint; 
    CamKeypoint **ptrPoints;
    CamKeypoint *point1, *point2;
    CamKeypoint *bigOne, *smallOne;
    int nbClusters2scan, nbPoints2scan[5];
    CamKeypoint **first2scan[5];

    int x1, x3, y1, y2, y3, p, num, den;
    signed short *ptr;
    CamImage filter;

    CAM_CHECK(camFastHessianDetector, camInternalROIPolicy(source, NULL, &iROI, 1));
    CAM_CHECK_ARGS(camFastHessianDetector, (source->depth & CAM_DEPTH_MASK) >= 8);
    CAM_CHECK_ARGS(camFastHessianDetector, points->allocated != 0);
    CAM_CHECK_ARGS(camFastHessianDetector, source->nChannels == 1 || ((source->nChannels == 3) && (source->dataOrder == CAM_DATA_ORDER_PLANE)));
    width = iROI.srcroi.width;
    height = iROI.srcroi.height;
    points->width = width;
    points->height = height;

    integral.imageData = NULL;
    roi = source->roi;
    camSetMaxROI(&roix, source);
    roix.coi = 1;
    source->roi = &roix;
    camIntegralImage(source, &integral);
    points->nbPoints = 0;
    integral.roi = &iROI.srcroi;
    source->roi = roi;

    // Bag allocation
    if (points->bag == NULL) {
#ifdef __SSE2__
	points->bag = (CamKeypoint*)_mm_malloc(sizeof(CamKeypoint) * points->allocated, 16);
#else
	points->bag = (CamKeypoint*)malloc(sizeof(CamKeypoint) * points->allocated);
#endif
    }
    for (i = 0; i < points->allocated; i++) {
        points->keypoint[i] = &points->bag[i];
    }
    points->nbPoints = 0;

    // Fast Hessian Detector for all scales
    for (scale = 0; scale < nbScales; scale++) {
	results[scale].imageData = NULL; // In order to use automatic allocation
	camFastHessianDetectorFixedScale(&integral, &results[scale], scale);
	// camSavePGM(&results[scale], "output/features_fast_hessian.pgm");
	pNbPoints = points->nbPoints;
	camFindLocalMaximaCircle5(&results[scale], points, threshold);
	for (i = pNbPoints; i < points->nbPoints; i++) {
	    points->keypoint[i]->scale = scale;
	    points->keypoint[i]->x <<= CamSampling[scale];
	    points->keypoint[i]->y <<= CamSampling[scale];
	    points->keypoint[i]->x += CamSamplingOffset[scale];
	    points->keypoint[i]->y += CamSamplingOffset[scale];
	}
    }	

    // Spatial clustering

    // 1st pass : Count the number of points per cluster
    widthClusters = (((width - 1) >> 5) + 1);
    heightClusters = (((height - 1) >> 5) + 1); 
    nbClusters = widthClusters * heightClusters;
    nbPerCluster = (int*)malloc(nbClusters * sizeof(int));
    for (i = 0; i < nbClusters; i++) nbPerCluster[i] = 0;
    for (i = 0; i < points->nbPoints; i++) {
	cluster = (points->keypoint[i]->y >> 5) * widthClusters + (points->keypoint[i]->x >> 5);
	nbPerCluster[cluster]++;
    }
    firstPoint =  (CamKeypoint***)malloc(nbClusters * sizeof(CamKeypoint**));    
    // Cluster ordered pointers to the points in clusters
    ptrPoints = (CamKeypoint**)malloc(points->nbPoints * sizeof(CamKeypoint*));
    firstPoint[0] = ptrPoints;
    for (i = 1; i < nbClusters; i++) firstPoint[i] = firstPoint[i - 1] + nbPerCluster[i - 1]; 
    // 2nd pass : Start again and register pointers
    for (i = 0; i < points->nbPoints; i++) {
	cluster = (points->keypoint[i]->y >> 5) * widthClusters + (points->keypoint[i]->x >> 5);
	*firstPoint[cluster] = points->keypoint[i];
	firstPoint[cluster]++;	
    }
    firstPoint[0] = ptrPoints;
    for (i = 1; i < nbClusters; i++) firstPoint[i] = firstPoint[i - 1] + nbPerCluster[i - 1]; 
    // OK. clustering is finished.
    
    // Set all points to unmarked
    for (i = 0; i < points->nbPoints; i++) points->keypoint[i]->size = 0;
    
    // Now, scan all the clusters and the points inside
    // for bad points removal
    for (cluster = 0; cluster < nbClusters; cluster++) {
	x = cluster % widthClusters;
	y = cluster / widthClusters;
	for (i = 0; i < nbPerCluster[cluster]; i++) {
	    point1 = *(firstPoint[cluster] + i);
	    
	    // Build the set of points to compare with
	    nbClusters2scan = 0;
	    if (i < nbPerCluster[cluster] - 1) {
		first2scan[0] = firstPoint[cluster] + i + 1;
		nbPoints2scan[0] = nbPerCluster[cluster] - i - 1; 
		nbClusters2scan = 1;
	    }
	    if (x != 0 && y != heightClusters - 1) {
		first2scan[nbClusters2scan] = firstPoint[cluster + widthClusters - 1];
		nbPoints2scan[nbClusters2scan] = nbPerCluster[cluster + widthClusters - 1];
		nbClusters2scan++;
	    }
	    if (y != heightClusters - 1) {
		first2scan[nbClusters2scan] = firstPoint[cluster + widthClusters];
		nbPoints2scan[nbClusters2scan] = nbPerCluster[cluster + widthClusters];
		nbClusters2scan++;
	    }
	    if (x != widthClusters - 1 && y != heightClusters - 1) {
		first2scan[nbClusters2scan] = firstPoint[cluster + widthClusters + 1];
		nbPoints2scan[nbClusters2scan] = nbPerCluster[cluster + widthClusters + 1];
		nbClusters2scan++;
	    }
	    if (x != widthClusters - 1) {
		first2scan[nbClusters2scan] = firstPoint[cluster + 1];
		nbPoints2scan[nbClusters2scan] = nbPerCluster[cluster + 1];
		nbClusters2scan++;
	    }
	    
	    // Now, scan all these points and compare with point1
	    for (j = 0; j < nbClusters2scan; j++) {
		for (k = 0; k < nbPoints2scan[j]; k++) {
		    point2 = *(first2scan[j] + k);
		    // Now I have point1 and point2
		    // They are next to each other...
		    // Let's compare them and possibly keep only one of them

		    // Which one is the bigger ?
		    if (point1->scale > point2->scale) {
			bigOne = point1;
			smallOne = point2;
		    } else {
			bigOne = point2;
			smallOne = point1;
		    }
			
		    // First of all, are these feature points similar in size ?
		    if (bigOne->scale - smallOne->scale != 1) continue;

		    // These two points are interesting
		    d = (point1->x - point2->x) * (point1->x - point2->x) + (point1->y - point2->y) * (point1->y - point2->y);
		    thr = (CamScale[bigOne->scale] - CamScale[smallOne->scale]);
		    thr *= thr;
		    thr <<= 2;
		    if (d < thr) {
			// These are next to each other
			// Are they of similar value ?
			if (abs(point1->value - point2->value) < ((point1->value + point2->value) >> 4)) {
			    // Yes they are. Keep the bigger.
			    smallOne->size = 1;
			} else {
			    // No they are not. Keep the better.
			    if (point1->value > point2->value) {
				point2->size = 1;
			    } else {
				point1->size = 1;
			    }
			}
		    }
		}
	    }	
	}
    }
   
    free(nbPerCluster);
    free(ptrPoints);
    free(firstPoint);

    // Remove all the points that have been marked to be destroyed
    pNbPoints = 0;
    i = 0;
    while (i < points->nbPoints && points->keypoint[i]->size == 0) {
	i++;
	pNbPoints++;
    }
    for (; i < points->nbPoints; i++) {
	if (points->keypoint[i]->size == 0) {
	    points->keypoint[pNbPoints++] = points->keypoint[i];
	}
    }
    points->nbPoints = pNbPoints;
    
    /* Interpolation :
     * Maxima : solve([y1=a*(x1-p)^2+b,y2=a*(-p)^2+b,y3=a*(x3-p)^2+b],[a,b,p])
     * Maxima yields the following formula for parabolic interpolation
                                       2         2     2         2
			             x1  y3 + (x3  - x1 ) y2 - x3  y1
			       p = ------------------------------------
                                   2 x1 y3 + (2 x3 - 2 x1) y2 - 2 x3 y1
    
     */
    for (i = 0; i < points->nbPoints; i++) {
	//if (1) { // Disable parabolic interpolation
	if (points->keypoint[i]->scale == 0 || points->keypoint[i]->scale == nbScales - 1) {
	    points->keypoint[i]->scale = CamScale[points->keypoint[i]->scale] << 2;
	    // Remove this point : its scale can't be interpolated
	    points->keypoint[i]->size = 1;
	} else {
	    x1 = CamScale[points->keypoint[i]->scale - 1] - CamScale[points->keypoint[i]->scale];
	    x3 = CamScale[points->keypoint[i]->scale + 1] - CamScale[points->keypoint[i]->scale];
	    
	    scale = points->keypoint[i]->scale - 1;
	    ptr = (signed short*)(results[scale].imageData + results[scale].widthStep * (points->keypoint[i]->y >> CamSampling[scale])) + (points->keypoint[i]->x >> CamSampling[scale]);
	    y1 = *ptr;

	    scale = points->keypoint[i]->scale;
	    ptr = (signed short*)(results[scale].imageData + results[scale].widthStep * (points->keypoint[i]->y >> CamSampling[scale])) + (points->keypoint[i]->x >> CamSampling[scale]);
	    y2 = *ptr;

	    scale = points->keypoint[i]->scale + 1;
	    ptr = (signed short*)(results[scale].imageData + results[scale].widthStep * (points->keypoint[i]->y >> CamSampling[scale])) + (points->keypoint[i]->x >> CamSampling[scale]);
	    y3 = *ptr;

	    if (y3 > y2) {
		points->keypoint[i]->scale++;
		i--; continue;
	    }
	    if (y1 > y2) {
		points->keypoint[i]->scale--;
		i--; continue;
	    }
	    num = (x1 * x1 * y3 + (x3 * x3 - x1 * x1) * y2 - x3 * x3 * y1);
	    den = x1 * y3 + (x3 - x1) * y2 - x3 * y1;
	    if (den == 0)
		points->keypoint[i]->size = 1; // Destroy
	    else {
		p = (num << 1) / den;	    
		points->keypoint[i]->scale = p + (CamScale[points->keypoint[i]->scale] << 2);
	    }
	}
    }

    // Memory deallocation 
    for (scale = 0; scale < nbScales; scale++) {
	camDeallocateImage(&results[scale]);
    }
    camDeallocateImage(&integral);

    // Remove again all the points that have been marked to be destroyed
    pNbPoints = 0;
    i = 0;
    while (i < points->nbPoints && points->keypoint[i]->size == 0) {
	i++;
	pNbPoints++;
    }
    for (; i < points->nbPoints; i++) {
	if (points->keypoint[i]->size == 0) {
	    points->keypoint[pNbPoints++] = points->keypoint[i];
	}
    }
    points->nbPoints = pNbPoints;
 
    // Angle
    if (options & CAM_UPRIGHT) {
	for (i = 0; i < points->nbPoints; i++) {
	    points->keypoint[i]->angle = 0;
	}
    } else {
        camAllocateImage(&filter, CAM_ORIENTATION_STAMP_SIZE, CAM_ORIENTATION_STAMP_SIZE, CAM_DEPTH_16S);
	camBuildGaussianFilter(&filter, camSigmaParam);
	pNbPoints = points->nbPoints;
	for (i = 0; i < pNbPoints; i++) {
	    camKeypointOrientation(source, points->keypoint[i], &filter, points);
	}
	camDeallocateImage(&filter);
    }

    camAllocateImage(&filter, 20, 20, CAM_DEPTH_16S);
    camBuildGaussianFilter(&filter, camSigmaParam);
    
    camKeypointsInternalsPrepareDescriptor();
    // Get the signature from the selected feature points
    for (i = 0; i < points->nbPoints; i++) {
	camKeypointsDescriptor(source, points->keypoint[i], &filter, options);
    }

    // Finally, set the points' set 
    for (i = 0; i < points->nbPoints; i++) {
	points->keypoint[i]->set = points;
    }

    // Sort the features according to value
    qsort(points->keypoint, points->nbPoints, sizeof(CamKeypoint*), camSortKeypoints);

    camDeallocateImage(&filter);
    camInternalROIPolicyExit(&iROI);
    return 1;
}

int camDrawKeypoints(CamKeypoints *points, CamImage *dest, int color)
{
    int i;
    for (i = 0; i < points->nbPoints; i++) {
	camDrawKeypoint(points->keypoint[i], dest, color);
    }
    return 1;
}

int camDrawKeypoint(CamKeypoint *point, CamImage *dest, int color)
{
    int x, y;
    double costheta, sintheta;

    if (dest->roi) {
	x = dest->roi->xOffset;
	y = dest->roi->yOffset;
    } else {
	x = 0;
	y = 0;
    }

    camDrawCircle(dest, x + point->x, y + point->y, (point->scale >> 2), color);
    if (point->angle != 0) {
	costheta = cos(point->angle * 2 * M_PI / 360);
	sintheta = sin(point->angle * 2 * M_PI / 360);
	camDrawLine(dest, (int)floor(x + point->x + costheta * (point->scale >> 2) + 0.5), (int)floor(y + point->y - sintheta * (point->scale >> 2) + 0.5), x + point->x, y + point->y, color);
    }
    return 1;
}


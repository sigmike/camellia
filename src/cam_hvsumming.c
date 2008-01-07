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

    Copyright (c) 2002-2006, Ecole des Mines de Paris - Centre de Robotique
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

/* Horizontal and vertical summing
 * C code */
#include "camellia.h"
#include "camellia_internals.h"

#ifndef CAM_INLINED
#define CAM_INLINED

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation

#undef CAM_PIXEL
#define CAM_PIXEL unsigned char
#define camSumHV camSumHV8
#define camSumH camSumH8
#define camSumV camSumV8
#include "cam_hvsumming.c"
#undef camSumHV
#undef camSumH
#undef camSumV
  
#undef CAM_PIXEL
#define CAM_PIXEL unsigned short
#define camSumHV camSumHV16
#define camSumH camSumH16
#define camSumV camSumV16
#include "cam_hvsumming.c"
#undef camSumHV
#undef camSumH
#undef camSumV
  
int camSumHV(CamImage *image, CamTable *hsum, CamTable *vsum)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camSumHV16(image,hsum,vsum);
    } else {
	return camSumHV8(image,hsum,vsum);
    }
}

int camSumH(CamImage *image, CamTable *hsum)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camSumH16(image,hsum);
    } else {
	return camSumH8(image,hsum);
    }
}

int camSumV(CamImage *image, CamTable *vsum)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camSumV16(image,vsum);
    } else {
	return camSumV8(image,vsum);
    }
}

#else
#include "cam_hvsumming.c"
#endif

#else

int camSumHV(CamImage *image, CamTable *hsum, CamTable *vsum)
{
    int x,y,i;
    int width,height;
    CAM_PIXEL *imptr,*tmpptr;
    CAM_PIXEL valpix;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camSumHV,camInternalROIPolicy(image, NULL, &iROI, 0));
    CAM_CHECK_ARGS(camSumHV,iROI.nChannels==1);

    imptr=(CAM_PIXEL*)iROI.srcptr;
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    // Algorithm initialization
    hsum->size=height; vsum->size=width;
    if (hsum->size>CAM_TABLE_SIZE) {
        camError("camSumHV","image too big");
        return 0;
    }
    if (vsum->size>CAM_TABLE_SIZE) {
        camError("camSumHV","image too big");
        return 0;
    }
    for (i=0;i<height;i++) hsum->t[i]=0;
    for (i=0;i<width;i++) vsum->t[i]=0;

    for (y=0;y<height;y++) {
	tmpptr=imptr;
	for (x=0;x<width;x++,imptr+=iROI.srcinc) {
	    valpix=*imptr;
	    hsum->t[y]+=valpix;
	    vsum->t[x]+=valpix;
	}
	imptr=(CAM_PIXEL*)(((char*)tmpptr)+image->widthStep);
    }

    return 1;    
}

int camSumV(CamImage *image, CamTable *results)
{
    int x,y,i;
    int width,height;
    CAM_PIXEL *imptr,*tmpptr;
    CAM_PIXEL valpix;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camSumV,camInternalROIPolicy(image, NULL, &iROI, 0));
    CAM_CHECK_ARGS(camSumV,iROI.nChannels==1);

    imptr=(CAM_PIXEL*)iROI.srcptr;
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    // Algorithm initialization
    results->size=width;
    if (results->size>CAM_TABLE_SIZE) {
        camError("camSumV","image too big");
        return 0;
    }
    for (i=0;i<width;i++) results->t[i]=0;

    for (y=0;y<height;y++) {
	tmpptr=imptr;
	for (x=0;x<width;x++,imptr+=iROI.srcinc) {
	    valpix=*imptr;
	    results->t[x]+=(int)valpix;
	}
	imptr=(CAM_PIXEL*)(((char*)tmpptr)+image->widthStep);
    }
    return 1;    
}

int camSumH(CamImage *image, CamTable *results)
{
    int x,y,i;
    int width,height;
    CAM_PIXEL *imptr,*tmpptr;
    CAM_PIXEL valpix;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camSumH,camInternalROIPolicy(image, NULL, &iROI, 0));
    CAM_CHECK_ARGS(camSumH,iROI.nChannels==1);

    imptr=(CAM_PIXEL*)iROI.srcptr;
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    // Algorithm initialization
    results->size=height;
    if (results->size>CAM_TABLE_SIZE) {
        camError("camSumH","image too big");
        return 0;
    }
    for (i=0;i<height;i++) results->t[i]=0;

    for (y=0;y<height;y++) {
	tmpptr=imptr;
	for (x=0;x<width;x++,imptr+=iROI.srcinc) {
	    valpix=*imptr;
	    results->t[y]+=valpix;
	}
	imptr=(CAM_PIXEL*)(((char*)tmpptr)+image->widthStep);
    }
    return 1;    
}
#endif

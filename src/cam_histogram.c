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

#ifndef CAM_INLINED
#define CAM_INLINED
/* Histogram computation
 * C code */
#include <stdlib.h>
#include "camellia.h"
#include "camellia_internals.h"

int camFindThreshold(CamTable *histo, int percent)
{
    int i,s,sp;
    for (i=0, s=0; i<256; i++) {
        s+=histo->t[i];
    }
    s=(int)((s*(100-percent))/100);
    for (i=0, sp=0; i<256; i++) {
        sp+=histo->t[i];
        if (sp>s) break;
    }
    if (i==0) return 1; // Hack not to take the 0 pixels in case of fake image
    return i;
}

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation

#undef CAM_PIXEL
#define CAM_PIXEL unsigned char
#define camHistogram camHistogram8
#define camHistogramEqualization camHistogramEqualization8
#define camHistogram2Channels camHistogram2Channels8
#include "cam_histogram.c"
#undef camHistogram
#undef camHistogramEqualization
#undef camHistogram2Channels
  
#undef CAM_PIXEL
#define CAM_PIXEL unsigned short
#define camHistogram camHistogram16
#define camHistogramEqualization camHistogramEqualization16
#define camHistogram2Channels camHistogram2Channels16
#include "cam_histogram.c"
#undef camHistogram
#undef camHistogramEqualization
#undef camHistogram2Channels
  
int camHistogram(CamImage *image, CamTable *histo)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camHistogram16(image,histo);
    } else {
	return camHistogram8(image,histo);
    }
}

int camHistogramEqualization(CamImage *src, CamImage *dest, CamTable *src_histo, int option, CamImage *work)
{
  if ((src->depth&CAM_DEPTH_MASK)>8) {
	return camHistogramEqualization16(src, dest, src_histo, option, work);
    } else {
	return camHistogramEqualization8(src, dest, src_histo, option, work);
    }
}

int camHistogram2Channels(CamImage *image, int ch1, int ch2, CamImage *result, int size)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camHistogram2Channels16(image,ch1,ch2,result,size);
    } else {
	return camHistogram2Channels8(image,ch1,ch2,result,size);
    }
}

#else
#include "cam_histogram.c"
#endif

#else
int camHistogram(CamImage *image, CamTable *histo)
{
    int x,y,i;
    int width,height;
    CAM_PIXEL *imptr,*tmpptr;
    CamInternalROIPolicyStruct iROI;
    int acc=0,val;
    int mask;
    DECLARE_MASK_MANAGEMENT;

    CAM_CHECK(camHistogram,camInternalROIPolicy(image, NULL, &iROI, 1));
    CAM_CHECK_ARGS(camHistogram,(image->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camHistogram,(image->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    
    histo->size=(1<<(image->depth&CAM_DEPTH_MASK));
    mask=histo->size-1;

    if (histo->size>CAM_TABLE_SIZE) histo->size=CAM_TABLE_SIZE;
    // Algorithm initialization
    for (i=0;i<histo->size;i++) {
	histo->t[i]=0;
    }

    // ROI (Region Of Interest) management
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    imptr=(CAM_PIXEL*)iROI.srcptr;

    INIT_MASK_MANAGEMENT;

    for (y=0;y<height;y++) {
	tmpptr=imptr; 
        BEGIN_MASK_MANAGEMENT(imptr=tmpptr+startx*iROI.srcinc;)
            for (x=startx;x<endx;x++,imptr+=iROI.srcinc) {
                val=*imptr;
                val&=mask;
                if (val<CAM_TABLE_SIZE) {
                    histo->t[val]++;
                } else {
                    histo->t[CAM_TABLE_SIZE-1]++;
                }
                acc+=val;
            }
        END_MASK_MANAGEMENT;
        imptr=(CAM_PIXEL*)(((char*)tmpptr)+image->widthStep);
    }

    // Signed images histogram rectification
    if (image->depth & CAM_DEPTH_SIGN) {
        for (x=0;x<histo->size/2;x++) {
            val=histo->t[x];
            histo->t[x]=histo->t[histo->size/2+x];
            histo->t[histo->size/2+x]=val;
        }
    }
    
    camInternalROIPolicyExit(&iROI);
    return acc;    
}

int camHistogramEqualization(CamImage *src, CamImage *dest, CamTable *src_histo, int option, CamImage *work)
{
    int x, y, i, c, n;
    int width, height;
    int total_pixels = 0;
    int allocated = 0;
    CamImage w;
    CAM_PIXEL *srcptr, *srctmpptr, *dstptr, *dsttmpptr;
    CAM_PIXEL *workptr[CAM_TABLE_SIZE];
    CAM_PIXEL *workend[CAM_TABLE_SIZE];
    CamInternalROIPolicyStruct iROI;
    int val;
    DECLARE_MASK_MANAGEMENT;

    CAM_CHECK(camHistogramEqualization, camInternalROIPolicy(src, dest, &iROI, 1));
    CAM_CHECK_ARGS(camHistogramEqualization, iROI.nChannels == 1);
    CAM_CHECK_ARGS(camHistogramEqualization, (src->depth & CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camHistogramEqualization, (src->depth & CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camHistogramEqualization, (dest->depth & CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camHistogramEqualization, (dest->depth & CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camHistogramEqualization, !(src->depth & CAM_DEPTH_SIGN)); 
    CAM_CHECK_ARGS(camHistogramEqualization, src_histo != NULL);
    CAM_CHECK_ARGS(camHistogramEqualization, src_histo->size == (1<<(src->depth&CAM_DEPTH_MASK)));

    // Algorithm initialization
    for (i = 0; i < src_histo->size; i++) {
	total_pixels += src_histo->t[i];
    }
    // Initialize work image
    if ( work == NULL || (work->imageSize < total_pixels * (int)sizeof(CAM_PIXEL)) ) {
	camAllocateImage(&w, total_pixels, 1, src->depth);
        work = &w;	
	allocated = 1;
    }
    workptr[0] = (CAM_PIXEL*)work->imageData;
    for (i = 0, c = 0; c < src_histo->size; c++) {
	n = ( (c + 1) * total_pixels ) >> src->depth;
	for (; i < n; i++, workptr[0]++) {
	    *workptr[0] = c;
	}
    }	
    // Initialize work pointers
    workptr[0] = (CAM_PIXEL*)work->imageData;
    for (i = 1; i < src_histo->size; i++) {
	workptr[i] = workptr[i-1] + src_histo->t[i-1];
	workend[i-1] = workptr[i]; 
    }
    workend[i-1] = workptr[i-1] + src_histo->t[i-1];

    // ROI (Region Of Interest) management
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    srcptr=(CAM_PIXEL*)iROI.srcptr;
    dstptr=(CAM_PIXEL*)iROI.dstptr;
    
    INIT_MASK_MANAGEMENT;

    if (option & CAM_EQUAL_FAST) {
	for (y=0;y<height;y++) {
	    srctmpptr=srcptr;
	    dsttmpptr=dstptr;	
	    BEGIN_MASK_MANAGEMENT(srcptr=srctmpptr+startx*iROI.srcinc; dstptr=dsttmpptr+startx*iROI.dstinc;)
		for (x=startx;x<endx;x++,srcptr+=iROI.srcinc,dstptr+=iROI.dstinc) {
		    val = *srcptr;
		    *dstptr = *workptr[val];
		    workptr[val]++;
		}
	    END_MASK_MANAGEMENT;
	    srcptr=(CAM_PIXEL*)(((char*)srctmpptr)+src->widthStep);
	    dstptr=(CAM_PIXEL*)(((char*)dsttmpptr)+dest->widthStep);
	}
    } else {
	for (y=0;y<height;y++) {
	    srctmpptr=srcptr;
	    dsttmpptr=dstptr;	
	    BEGIN_MASK_MANAGEMENT(srcptr=srctmpptr+startx*iROI.srcinc; dstptr=dsttmpptr+startx*iROI.dstinc;)
		for (x=startx;x<endx;x++,srcptr+=iROI.srcinc,dstptr+=iROI.dstinc) {
		    val = *srcptr;

		    // Randomly pick a new pixel value
		    n = workend[val] - workptr[val];
		    i = rand() % n;
		    
		    // Apply the pixel value
		    *dstptr = *(workptr[val] + i);
		    *(workptr[val] + i) = *workptr[val];
		    workptr[val]++;
		}
	    END_MASK_MANAGEMENT;
	    srcptr=(CAM_PIXEL*)(((char*)srctmpptr)+src->widthStep);
	    dstptr=(CAM_PIXEL*)(((char*)dsttmpptr)+dest->widthStep);
	}
    }

    camInternalROIPolicyExit(&iROI);
    if (allocated) camDeallocateImage(work);
    return 1;    
}

int camHistogram2Channels(CamImage *image, int ch1, int ch2, CamImage *result, int size)
{
    int x,y,xp,yp;
    int width,height;
    CAM_PIXEL *ch1ptr,*tmpptr1;
    CAM_PIXEL *ch2ptr,*tmpptr2;
    unsigned char *dest8;
    unsigned short *dest16;
    int s,valmax;
    CamInternalROIPolicyStruct iROI1,iROI2;
    CamROI *oldROI,roi;

    oldROI=image->roi;
    if (oldROI) {
        roi=*oldROI;
    } else {
        camSetROI(&roi,0,0,0,image->width,image->height);
    }
    roi.coi=ch1;
    image->roi=&roi;
    CAM_CHECK(camHistogram2Channels,camInternalROIPolicy(image, NULL, &iROI1, 0));
    roi.coi=ch2;
    CAM_CHECK(camHistogram2Channels,camInternalROIPolicy(image, NULL, &iROI2, 0));
    image->roi=oldROI;

    // Automatic memory allocation
    if (result->imageData==NULL) {
        if (!camAllocateImage(result,(int)((1<<(image->depth&CAM_DEPTH_MASK))/size),(int)((1<<(image->depth&CAM_DEPTH_MASK))/size),CAM_DEPTH_16U)) {
            return 0;
        }
    }

    CAM_CHECK_ARGS(camHistogram2Channels, ((result->height==(int)((1<<(image->depth&CAM_DEPTH_MASK))/size))&&(result->width==(int)((1<<(image->depth&CAM_DEPTH_MASK))/size))));
    CAM_CHECK_ARGS(camHistogram2Channels, (((result->depth&CAM_DEPTH_MASK)>=8)&&((result->depth&CAM_DEPTH_MASK)<=16)));

    // Algorithm initialisation
    // Find the power of 2 of size
    s=0;
    while (size>1) {size>>=1;s++;}; 

    // ROI (Region Of Interest) management
    width=iROI1.srcroi.width;
    height=iROI1.srcroi.height;
    ch1ptr=(CAM_PIXEL*)iROI1.srcptr;
    ch2ptr=(CAM_PIXEL*)iROI2.srcptr;

    if ((result->depth&CAM_DEPTH_MASK)==8) {
	// 8 bits deep processing
        // Set the result picture to 0
	camSet(result,0);
	// The histogramming itself
	for (y=0;y<height;y++) {
	    tmpptr1=ch1ptr;
	    tmpptr2=ch2ptr;
	    for (x=0;x<width;x++,ch1ptr+=iROI1.srcinc,ch2ptr+=iROI2.srcinc) {
		yp=(*ch1ptr)>>s;
		xp=(*ch2ptr)>>s;
		dest8=result->imageData+yp*result->widthStep+xp;
		if (*dest8!=255) (*dest8)++;
	    }
	    ch1ptr=(CAM_PIXEL*)(((char*)tmpptr1)+image->widthStep);
	    ch2ptr=(CAM_PIXEL*)(((char*)tmpptr2)+image->widthStep);
	}
    } else {
	// 8 to 16 bits deep processing
	valmax=(1<<(result->depth&CAM_DEPTH_MASK))-1;
	// Set the result picture to 0
	for (y=0;y<result->height;y++) {
	    dest16=(unsigned short*)(result->imageData+y*result->widthStep);
	    for (x=0;x<result->width;x++,dest16++) {
		*dest16=0;
	    }
	}
	// The histogramming itself
	for (y=0;y<height;y++) {
	    tmpptr1=ch1ptr;
	    tmpptr2=ch2ptr;
	    for (x=0;x<width;x++,ch1ptr+=iROI1.srcinc,ch2ptr+=iROI2.srcinc) {
		yp=(*ch1ptr)>>s;
		xp=(*ch2ptr)>>s;
		dest16=(unsigned short*)(result->imageData+yp*result->widthStep+(xp<<1));
		if (*dest16!=valmax) (*dest16)++;
	    }
	    ch1ptr=(CAM_PIXEL*)(((char*)tmpptr1)+image->widthStep);
	    ch2ptr=(CAM_PIXEL*)(((char*)tmpptr2)+image->widthStep);
	}
    }
    return 1;    
}

#endif


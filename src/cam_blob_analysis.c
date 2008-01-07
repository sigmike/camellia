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

/* Blob analysis Kernel
 * C code */
#include <stdlib.h>
#include "camellia.h"

/* v1.1 : 1st of November 2002 
 *	- Divided blob analysis into two functions (1stScan and Refinement)
 *	- Added center of gravity computation
 *	- Added min-max computation
 * v1.2 : 20th of November 2002
 *	- Bug correction by ULP
 * v1.3 : 28th of July 2003
 *	- Added holes parameter to camBlobAnalysis1stScan
 */
// This function can work when "original" parameter is set to NULL. It won't compute the value pixel value for the blobs
int camBlobAnalysis1stScan(CamImage *blobImage, CamImage *original, CamLabellingResults *info, CamBlobAnalysisResults *results)
{
    int x,y,i,p,valblob;
    int width,height;
    CAM_LABEL_PIXEL *blobimptr,*blobimtmpptr;
    CAM_PIXEL *origptr,*origtmpptr,valpix;
    CamBlobInfo *blobInfo;
    int *equivTable=info->equiv;
    int labelToBlob[CAM_LABEL_MAX_BLOBS];
    int nbBlobs=0;

    // Algorithm initialization
    for (i=0;i<info->nbLabels;i++) {
	labelToBlob[i]=-1;
    }

    // ROI (Region Of Interest) management
    if (blobImage->roi) {
	blobimptr=(CAM_LABEL_PIXEL*)(blobImage->imageData+blobImage->roi->yOffset*blobImage->widthStep+blobImage->roi->xOffset*sizeof(CAM_LABEL_PIXEL));
	width=blobImage->roi->width;
	height=blobImage->roi->height;
    } else {
	blobimptr=(CAM_LABEL_PIXEL*)blobImage->imageData;
	width=blobImage->width;
	height=blobImage->height;
    }
    if (original!=NULL) {
	if (original->roi) {
	    origptr=(CAM_PIXEL*)(original->imageData+original->roi->yOffset*original->widthStep+original->roi->xOffset*sizeof(CAM_PIXEL));
	} else {
	    origptr=(CAM_PIXEL*)original->imageData;
	}
    } else origptr=NULL;

    for (y=0;y<height;y++) {
	blobimtmpptr=blobimptr;
	origtmpptr=origptr;
	for (x=0;x<width;x++,blobimptr++,origptr++) {
	    valblob=*blobimptr;
	    // Parent path compression
	    p=equivTable[valblob];
	    if (p!=valblob) {
		while (p!=equivTable[p]) p=equivTable[p];
	    }
	    // Is this label associated to a blob?
	    if (labelToBlob[p]<0) {
		// No! Let's create a new blob
		*blobimptr=nbBlobs;
		labelToBlob[p]=nbBlobs;
		blobInfo=&results->blobInfo[nbBlobs++];
		// Initialize the new blob
		blobInfo->left=x;
		blobInfo->width=1;
		blobInfo->top=y;
		blobInfo->height=1;
		blobInfo->surface=1;
		if (original!=NULL) {
		    valpix=*(origptr);
		    blobInfo->value=valpix;
		    blobInfo->min=valpix;
		    blobInfo->max=valpix;
		} else {
		    blobInfo->value=0;
    		    blobInfo->min=0;
		    blobInfo->max=0;
		}
		blobInfo->cx=x;
		blobInfo->cy=y;
	    } else {
		*blobimptr=labelToBlob[p];
		blobInfo=&results->blobInfo[labelToBlob[p]];
		if (x<blobInfo->left) {
		    blobInfo->width+=blobInfo->left-x;
		    blobInfo->left=x;
		} else if (x>=blobInfo->left+blobInfo->width) {
		    blobInfo->width=x-blobInfo->left+1;
		}
		if (y>=blobInfo->top+blobInfo->height) blobInfo->height=y-blobInfo->top+1;
		blobInfo->surface++;
		if (original!=NULL) {
		    valpix=*(origptr);
		    blobInfo->value+=valpix;
		    if (valpix>blobInfo->max) blobInfo->max=valpix;
		    if (valpix<blobInfo->min) blobInfo->min=valpix;
		}
		blobInfo->cx+=x;
		blobInfo->cy+=y;
	    }
	}
	blobimptr=(CAM_LABEL_PIXEL*)(((char*)blobimtmpptr)+blobImage->widthStep);
	if (original!=NULL) origptr=(CAM_PIXEL*)(((char*)origtmpptr)+original->widthStep);
    }

    // Finish processing for all the blobs
    for (i=0;i<nbBlobs;i++) {
	results->blobInfo[i].value/=results->blobInfo[i].surface;
	results->blobInfo[i].cx/=results->blobInfo[i].surface;
	results->blobInfo[i].cy/=results->blobInfo[i].surface;	
    }
    results->nbBlobs=nbBlobs;
    return 1;    
}

// This function requires a pointer to the original image, so that value, min and max pixel values for the blobs can be computed
int camBlobAnalysisRefinement(CamImage *blobImage, CamImage *original, CamBlobAnalysisResults *results)
{
    int x,y,i,blob;
    int width,height;
    CAM_LABEL_PIXEL *blobimptr,*blobimtmpptr;
    CAM_PIXEL *origptr,*origtmpptr;
    CAM_PIXEL valpix;
    CamBlobInfo *blobInfo;

    // Algorithm initialization
    for (i=0;i<results->nbBlobs;i++) {
	results->blobInfo[i].value=0;
	results->blobInfo[i].min=0xffff;
	results->blobInfo[i].max=0;
    }

    // ROI (Region Of Interest) management
    if (blobImage->roi) {
	blobimptr=(CAM_LABEL_PIXEL*)(blobImage->imageData+blobImage->roi->yOffset*blobImage->widthStep+blobImage->roi->xOffset*sizeof(CAM_LABEL_PIXEL));
	width=blobImage->roi->width;
	height=blobImage->roi->height;
    } else {
	blobimptr=(CAM_LABEL_PIXEL*)blobImage->imageData;
	width=blobImage->width;
	height=blobImage->height;
    }
    if (original->roi) {
        origptr=(CAM_PIXEL*)(original->imageData+original->roi->yOffset*original->widthStep+original->roi->xOffset*sizeof(CAM_PIXEL));
    } else {
	origptr=(CAM_PIXEL*)original->imageData;
    }
    for (y=0;y<height;y++) {
	blobimtmpptr=blobimptr;
	origtmpptr=origptr;
	for (x=0;x<width;x++,blobimptr++,origptr++) {
	    valpix=*(origptr);
	    blob=*blobimptr;
	    blobInfo=&results->blobInfo[blob];
	    blobInfo->value+=valpix;
	    if (valpix>blobInfo->max) blobInfo->max=valpix;
	    if (valpix<blobInfo->min) blobInfo->min=valpix;
	}
	blobimptr=(CAM_LABEL_PIXEL*)(((char*)blobimtmpptr)+blobImage->widthStep);
	origptr=(CAM_PIXEL*)(((char*)origtmpptr)+original->widthStep);
    }
    for (i=0;i<results->nbBlobs;i++) {
	results->blobInfo[i].value/=results->blobInfo[i].surface;
    }
    return 1;    
}



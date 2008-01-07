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

int camSumOfPixels(CamImage *image)
{
    int x,y;
    int width,height;
    unsigned CAM_PIXEL *imptr, *tmpptr;
    signed CAM_PIXEL *imptrS, *tmpptrS;
    int valpix;
    CamInternalROIPolicyStruct iROI;
    int sum = 0;

    DECLARE_MASK_MANAGEMENT;

    // ROI (Region Of Interest) management
    CAM_CHECK(camMeasures,camInternalROIPolicy(image, NULL, &iROI, 1));    
    CAM_CHECK_ARGS(camMeasures,iROI.nChannels==1);

    // ROI (Region Of Interest) management
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    // Mask management
    INIT_MASK_MANAGEMENT;

    if (image->depth & CAM_DEPTH_SIGN) {
        imptrS=(signed CAM_PIXEL*)iROI.srcptr;
	for (y=0;y<height;y++) {
	    tmpptrS=imptrS;
	    
	    BEGIN_MASK_MANAGEMENT(imptrS=tmpptrS+startx*iROI.srcinc;)
		for (x=startx;x<endx;x++,imptrS+=iROI.srcinc) {
		    valpix=*imptrS;
		    sum+=valpix;
		}
		imptrS=tmpptrS+iROI.srclinc;
	    END_MASK_MANAGEMENT;
	}
    } else {
        imptr=(unsigned CAM_PIXEL*)iROI.srcptr;
	for (y=0;y<height;y++) {
	    tmpptr=imptr;
	    
	    BEGIN_MASK_MANAGEMENT(imptr=tmpptr+startx*iROI.srcinc;)
		for (x=startx;x<endx;x++,imptr+=iROI.srcinc) {
		    valpix=*imptr;
		    sum+=valpix;
		}
		imptr=tmpptr+iROI.srclinc;
	    END_MASK_MANAGEMENT;
	}
    }

    camInternalROIPolicyExit(&iROI);
    return sum;    
}

int camMeasures(CamImage *image, CamMeasuresResults *results)
{
    int x,y;
    int width,height;
    unsigned CAM_PIXEL *imptr, *tmpptr;
    signed CAM_PIXEL *imptrS, *tmpptrS;
    int valpix;
    CamInternalROIPolicyStruct iROI;
    int nbpix=0;

    DECLARE_MASK_MANAGEMENT;

    // ROI (Region Of Interest) management
    CAM_CHECK(camMeasures,camInternalROIPolicy(image, NULL, &iROI, 1));    
    CAM_CHECK_ARGS(camMeasures,iROI.nChannels==1);

    // ROI (Region Of Interest) management
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    // Mask management
    INIT_MASK_MANAGEMENT;

    // Algorithm initialization
    results->min=65536;
    results->max=-32769;
    results->sum=0;

    if (image->depth & CAM_DEPTH_SIGN) {
        imptrS=(signed CAM_PIXEL*)iROI.srcptr;
	for (y=0;y<height;y++) {
	    tmpptrS=imptrS;
	    
	    BEGIN_MASK_MANAGEMENT(imptrS=tmpptrS+startx*iROI.srcinc;)
		for (x=startx;x<endx;x++,imptrS+=iROI.srcinc) {
		    valpix=*imptrS;
		    results->sum+=valpix;
		    if (valpix<results->min) {
			results->min=valpix;
			results->xmin=x;
			results->ymin=y;
		    }
		    if (valpix>results->max) {
			results->max=valpix;
			results->xmax=x;
			results->ymax=y;
		    }
		}
		nbpix+=endx-startx;
		imptrS=tmpptrS+iROI.srclinc;
	    END_MASK_MANAGEMENT;
	}
    } else {
        imptr=(unsigned CAM_PIXEL*)iROI.srcptr;
	for (y=0;y<height;y++) {
	    tmpptr=imptr;
	    
	    BEGIN_MASK_MANAGEMENT(imptr=tmpptr+startx*iROI.srcinc;)
		for (x=startx;x<endx;x++,imptr+=iROI.srcinc) {
		    valpix=*imptr;
		    results->sum+=valpix;
		    if (valpix<results->min) {
			results->min=valpix;
			results->xmin=x;
			results->ymin=y;
		    }
		    if (valpix>results->max) {
			results->max=valpix;
			results->xmax=x;
			results->ymax=y;
		    }
		}
		nbpix+=endx-startx;
		imptr=tmpptr+iROI.srclinc;
	    END_MASK_MANAGEMENT;
	}
    }

    if (nbpix) {
	results->average = results->sum / nbpix;
    } else {
	results->average = 0;
    }

    camInternalROIPolicyExit(&iROI);
    return results->sum;    
}

float camMeasureAverageDeviation(CamImage *image, int average)
{
    int x,y;
    int width,height;
    signed CAM_PIXEL *imptrS, *tmpptrS;
    unsigned CAM_PIXEL *imptr, *tmpptr;
    int valpix;
    double result=0;
    CamInternalROIPolicyStruct iROI;
    CamMeasuresResults r;
    int nbpix=0;
    
    DECLARE_MASK_MANAGEMENT;

    // Measure average if necessary
    if (average<=0) {
        if (camMeasures(image,&r)) {
            average=r.average;
        } else return 0;
    }

    // ROI (Region Of Interest) management
    CAM_CHECK(camMeasureAverageDeviation,camInternalROIPolicy(image, NULL, &iROI, 1));
    CAM_CHECK_ARGS(camMeasureAverageDeviation,iROI.nChannels==1);
    
    // ROI (Region Of Interest) management
    imptr=(CAM_PIXEL*)iROI.srcptr;
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    // Mask management
    INIT_MASK_MANAGEMENT;

    if (image->depth & CAM_DEPTH_SIGN) {
        imptrS=(CAM_PIXEL*)iROI.srcptr;
	for (y=0;y<height;y++) {
	    tmpptrS=imptrS;
	    
	    BEGIN_MASK_MANAGEMENT(imptrS=tmpptrS+startx*iROI.srcinc;)
		for (x=startx;x<endx;x++,imptrS+=iROI.srcinc) {
		    valpix=*imptrS;
		    result+=(valpix-average)*(valpix-average);
		}
		nbpix+=endx-startx;
		imptrS=tmpptrS+iROI.srclinc;
	    END_MASK_MANAGEMENT;
	}
    } else {
        imptr=(CAM_PIXEL*)iROI.srcptr;
	for (y=0;y<height;y++) {
	    tmpptr=imptr;
	    
	    BEGIN_MASK_MANAGEMENT(imptr=tmpptr+startx*iROI.srcinc;)
		for (x=startx;x<endx;x++,imptr+=iROI.srcinc) {
		    valpix=*imptr;
		    result+=(valpix-average)*(valpix-average);
		}
		nbpix+=endx-startx;
		imptr=tmpptr+iROI.srclinc;
	    END_MASK_MANAGEMENT;
	}
    }
    
    camInternalROIPolicyExit(&iROI);
    if (nbpix) {
	return (float)sqrt(result/nbpix);
    } else {
	return 0;
    }
}

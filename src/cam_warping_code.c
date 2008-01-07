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

#include <stdio.h>
#include "camellia.h"

// Nota bene : Setting on the perspective parameter for 2D warping is not recommended,
//	because it slows down the whole algorithm

// This function operates a backward mapping from the source image to a destination image
// All the params refer to locations in the source image, whereas the ROI scanned
// by this warping function is set by the roi parameter of the dest image

// Warning : this code assumes sizeof(int) of 4
int camWarping(CamImage *source, CamImage *dest, CamWarpingParams *params)
{
    int i,x,y,width,height;
    int xl,yl,xr,yr; // Position on the left and right side (source image)
    int incxl,incyl,incxr,incyr; // Increment on the left and right sides (source image)
    int dxl,dyl,dxr,dyr; // Delta for the left and right sides
    int xp,yp; // Position of the current point
    int incxp,incyp; // Increment for the current point 
    CAM_PIXEL *dstptr,*cpdstptr,*srcptr,*srcptrref;
    int xx,yy;
    int perspective=params->perspective;
    int zul,zbl,zur,zbr,cl,cr,zl,zr; // Projection parameters
    int I0,I1,S0,S1,S2,S3; // Bilinear interpolation intermediate results
    CamPoint I;
    CamInternalROIPolicyStruct iROI;
    
    // ROI (Region Of Interest) management
    CAM_CHECK(camWarping,camInternalROIPolicy(source, dest, &iROI, CAM_NO_ROI_INTERSECTION));
    CAM_CHECK_ARGS(camWarping,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camWarping,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camWarping,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camWarping,(dest->depth&CAM_DEPTH_MASK)>=8);

    dstptr=(CAM_PIXEL*)iROI.dstptr;
    srcptrref=(CAM_PIXEL*)iROI.srcptr;
    width=iROI.dstroi.width;
    height=iROI.dstroi.height;
    
    xl=params->p[0].x;
    yl=params->p[0].y;
    xr=params->p[1].x;
    yr=params->p[1].y;
    
    dxl=params->p[3].x-xl;
    dyl=params->p[3].y-yl;
    dxr=params->p[2].x-xr;
    dyr=params->p[2].y-yr;
    
    // Perspective warping or not?
    if (perspective) {
	if (!camIntersectionSegments(params->p,&I)) {
	    perspective=0;
	} else {
	    // 32 bits fixed point
	    zul=(int)((((CAM_INT64)1)<<48)/(params->p[0].y-I.y));
	    zur=(int)((((CAM_INT64)1)<<48)/(params->p[1].y-I.y));
	    zbl=(int)((((CAM_INT64)1)<<48)/(params->p[3].y-I.y));
	    zbr=(int)((((CAM_INT64)1)<<48)/(params->p[2].y-I.y));
	    cl=((zul-zbl)/height)>>1; // 31 bits fixed point
	    cr=((zur-zbr)/height)>>1; // Should be positive valued
	}
    }
    if (!perspective) {
	incxl=dxl/height;
	incyl=dyl/height;
	incxr=dxr/height;
	incyr=dyr/height;
	// Move the position one half pixel down
	xl+=(incxl>>1);
	xr+=(incxr>>1);
	yl+=(incyl>>1);
	yr+=(incyr>>1);
    } else {
	incxl=(int)((((CAM_INT64)dxl)<<16)/dyl);
	incxr=(int)((((CAM_INT64)dxr)<<16)/dyr);
    }

    // For all destination pixels
    // Let's go across all the lines
    for (y=0;y<height;y++) {

	if (perspective) {
	    zl=zul-((y<<1)+1)*cl; // 32 bits fixed points
	    zr=zur-((y<<1)+1)*cr;
	    yl=I.y+(int)((((CAM_INT64)1)<<48)/zl);
	    yr=I.y+(int)((((CAM_INT64)1)<<48)/zr);
	    xl=params->p[0].x+(int)(((CAM_INT64)incxl)*(yl-params->p[0].y)>>16);
	    xr=params->p[1].x+(int)(((CAM_INT64)incxr)*(yr-params->p[1].y)>>16);
	}

	cpdstptr=dstptr;
	incxp=(xr-xl)/width;
	incyp=(yr-yl)/width;
	// Move the position one half pixel to the right
	xp=xl+(incxp>>1); yp=yl+(incyp>>1);

	// And then across each horizontal pixel
	for (x=0;x<width;x++,dstptr+=iROI.dstinc) {
            // We can compute the interpolation between them
            switch (params->interpolation) {
            case 0: // Nearest neighbour nterpolation (fast but crude)
            default:
                // Let's retrieve the value of the current pixel in the source image
                xx=xp>>16;
                yy=yp>>16;
                if ((xx>=0)&&(yy>=0)&&(xx<iROI.srcroi.width)&&(yy<iROI.srcroi.height)) {
                    srcptr=(CAM_PIXEL*)(srcptrref+yy*source->widthStep)+xx*iROI.srcinc;
                    for (i=0;i<iROI.nChannels;i++) {
                        // Write the result to the destination image
                        *(dstptr+i)=*(srcptr+i);
                    }
                } else {
                    for (i=0;i<iROI.nChannels;i++) {
                        // Blank pixels
                        *(dstptr+i)=0;
                    }
                }
                break;
            case 1: // Bilinear interpolation
                // Let's retrieve the 4 nearest points in the source image
                xx=(xp-32767)>>16; // Upper left pixel
                yy=(yp-32767)>>16;
                if ((xx>=0)&&(yy>=0)&&(xx<iROI.srcroi.width-1)&&(yy<iROI.srcroi.height-1)) {
                    srcptr=(CAM_PIXEL*)(srcptrref+yy*source->widthStep)+xx*iROI.srcinc;
                    xx=(xx<<16)+32767;
                    yy=(yy<<16)+32767; // Center of the upper left pixel
                    for (i=0;i<iROI.nChannels;i++,srcptr++) {
                        S0=*srcptr;
                        S1=*(srcptr+iROI.srcinc);
                        S2=*(srcptr+iROI.srclinc);
                        S3=*(srcptr+iROI.srcinc+iROI.srclinc);
                        I0=(S0<<8)+(((xp-xx)*(S1-S0))>>8); // Horizontal linear interpolation
                        I1=(S2<<8)+(((xp-xx)*(S3-S2))>>8); // 8 bits fixed point
                        *(dstptr+i)=(I0+(((yp-yy)*(I1-I0))>>16)+128)>>8; // Vertical linear interpolation                    
                    }
                } else {
                    for (i=0;i<iROI.nChannels;i++) {
                        // Blank pixels
                        *(dstptr+i)=0;
                    }
                }
                break;
            }
            
            // Go to the next pixel
            xp+=incxp;
            yp+=incyp;	
	}
	
	// Move the destination pointer
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);

	// Go to the next line
	if (!perspective) {
	    xl+=incxl;
	    xr+=incxr;
	    yl+=incyl;
	    yr+=incyr;
	}
    }
    return 1;
}


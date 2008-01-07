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
#include "camellia_internals.h"

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation
#undef CAM_PIXEL
#define CAM_PIXEL unsigned char
#define camWarping camWarping8
#include "cam_warping_code.c"
#undef camWarping

#undef CAM_PIXEL
#define CAM_PIXEL unsigned short
#define camWarping camWarping16
#include "cam_warping_code.c"
#undef camWarping

int camWarping(CamImage *source, CamImage *dest, CamWarpingParams *params)
{
    CamImage s2,d2;
    CamROI sroi,droi;
    int i,result;
    if (source->dataOrder==CAM_DATA_ORDER_PLANE) {
        if ((source->nChannels!=1)&&((source->roi==NULL)||(source->roi->coi==0))) {
            s2=*source;
            if (source->roi) sroi=*source->roi; else {
                sroi.xOffset=0; sroi.yOffset=0; sroi.width=s2.width; sroi.height=s2.height;
            }
            s2.roi=&sroi;
            d2=*dest;
            if (dest->roi) droi=*dest->roi; else {
                droi.xOffset=0; droi.yOffset=0; droi.width=d2.width; droi.height=d2.height;
            }
            d2.roi=&droi;
            // Process all the planes separately
            for (i=0;i<source->nChannels;i++) {
                sroi.coi=i+1;
                droi.coi=i+1;
                if ((source->depth&CAM_DEPTH_MASK)>8) {
                    result=camWarping16(&s2,&d2,params);
                } else {
                    result=camWarping8(&s2,&d2,params);
                }
                if (result==0) break;
            }
            return result;
        }
    }   
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camWarping16(source,dest,params);
    } else {
	return camWarping8(source,dest,params);
    }
}

#else
#include "cam_warping_code.c"
#endif

int camScale(CamImage *source, CamImage *dest)
{
    CamWarpingParams params;
    CamInternalROIPolicyStruct iROI;
    CamImage src2;
    CamROI roi;

    // ROI (Region Of Interest) management
    CAM_CHECK(camWarping,camInternalROIPolicy(source, NULL, &iROI, 0));
    // Copy the original roi if it exists in order to avoid clipping
    if (source->roi) iROI.srcroi=*source->roi;

    params.perspective=0;
    params.interpolation=1;
    params.p[0].x=(iROI.srcroi.xOffset<<16)-1;
    params.p[0].y=(iROI.srcroi.yOffset<<16)-1;
    params.p[1].x=((iROI.srcroi.xOffset+iROI.srcroi.width)<<16)-1;
    params.p[1].y=(iROI.srcroi.yOffset<<16)-1;
    params.p[2].x=((iROI.srcroi.xOffset+iROI.srcroi.width)<<16)-1;
    params.p[2].y=((iROI.srcroi.yOffset+iROI.srcroi.height)<<16)-1;
    params.p[3].x=(iROI.srcroi.xOffset<<16)-1;
    params.p[3].y=((iROI.srcroi.yOffset+iROI.srcroi.height)<<16)-1;
    
    // Apply the warping on the source image with NULL roi
    camRefCopy(source,&src2);
    src2.roi=&roi;
    if (source->roi) roi.coi=source->roi->coi; else roi.coi=0;
    roi.xOffset=0; roi.yOffset=0; roi.width=source->width; roi.height=source->height;

    return camWarping(&src2,dest,&params);
}

#undef CAM_PIXEL
#define CAM_PIXEL unsigned char

// The source image memory access optimization is efficient only on a CPU
// with a large instruction cache and a rather low "bad condition prediction" penalty.
#define CAM_OPTIMIZE_MEMORY_ACCESS

#undef GET_PIXEL
#ifdef CAM_OPTIMIZE_MEMORY_ACCESS
#define GET_PIXEL(xp,yp,l) \
    if (yptr##l!=yp) { \
    /* Check that the pixel is within the source image boundaries */ \
	if ((yp>=0)&&(yp<source->height)&&(xp>=0)&&(xp<source->width)) { \
    	    srcptr##l=(CAM_PIXEL*)(source->imageData+yp*source->widthStep+xp*sizeof(CAM_PIXEL)); \
	    valpix##l=*srcptr##l; /* Image memory access */ \
	    xptr##l=xp; \
	    yptr##l=yp; \
	} else { \
	    yptr##l=0xdead; /* Out of image memory */ \
	    valpix##l=0; \
	} \
    } else { \
	if (xp>xptr##l) { \
	    if (xp<source->width) { /* This is the critical path */ \
	        srcptr##l+=xp-xptr##l; \
	        valpix##l=*srcptr##l; /* Image memory access */ \
	        xptr##l=xp; \
	    } else { \
	        yptr##l=0xdead; /* Out of image memory */ \
	        valpix##l=0; \
	    } \
	} else if (xp!=xptr##l) { \
	    if (xp>=0) { \
	        srcptr##l+=xp-xptr##l; \
	        valpix##l=*srcptr##l; /* Image memory access */ \
	        xptr##l=xp; \
	    } else { \
	        yptr##l=0xdead; /* Out of image memory */ \
	        valpix##l=0; \
	    } \
	} \
    }
#else
#define GET_PIXEL(xp,yp,l) \
    /* Check that the pixel is within the source image boundaries */ \
    if ((yp>=0)&&(yp<source->height)&&(xp>=0)&&(xp<source->width)) { \
        srcptr##l=(CAM_PIXEL*)(source->imageData+yp*source->widthStep+xp*sizeof(CAM_PIXEL)); \
        valpix##l=*srcptr##l; /* Image memory access */ \
    } else valpix##l=0;
#endif

// This function is buggy, since it doesn't fully implement a super sampling
// Still being worked on...
int camWarpingSuperSampling(CamImage *source, CamImage *dest, CamWarpingParams *params)
{
    int x,y,width,height;
    int xl,yl,xr,yr; // Position on the left and right side (source image)
    int incxl,incyl,incxr,incyr; // Increment on the left and right sides (source image)
    int dxl,dyl,dxr,dyr; // Delta for the left and right sides
    int xp,yp; // Position of the current point
    int xpp,ypp; // Position of the previous point
    int incxp,incyp; // Increment for the current point 
    int xpl[CAM_MAX_SCANLINE+1],ypl[CAM_MAX_SCANLINE+1]; // (x,y) points for the previous scanline
    CAM_PIXEL scanline[CAM_MAX_SCANLINE+1],valpix1,valpixp;
    CAM_PIXEL *dstptr,*srcptr1,*cpdstptr;
    int xptr1,yptr1=0xdead; // Special value. Generally inaccessible.
    int result,xx,yy;
    int perspective=params->perspective;
    int zul,zbl,zur,zbr,cl,cr,zl,zr; // Projection parameters
    int w1,w2,w3,w4,sum; // Weights for linear "square" interpolation
    CamPoint I;
    
    CAM_CHECK_ARGS(camWarpingSuperSampling,source->nChannels==1);
    CAM_CHECK_ARGS(camWarpingSuperSampling,dest->nChannels==1);
    CAM_CHECK_ARGS(camWarpingSuperSampling,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camWarpingSuperSampling,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camWarpingSuperSampling,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camWarpingSuperSampling,(dest->depth&CAM_DEPTH_MASK)>=8);

    // ROI (Region Of Interest) management
    if (dest->roi) {
	dstptr=(CAM_PIXEL*)(dest->imageData+dest->roi->yOffset*dest->widthStep+dest->roi->xOffset*sizeof(CAM_PIXEL));
	width=dest->roi->width;
	height=dest->roi->height;
    } else {
	dstptr=(CAM_PIXEL*)dest->imageData;
	width=dest->width;
	height=dest->height;
    }
    
    xl=params->p[0].x;
    yl=params->p[0].y;
    xr=params->p[1].x;
    yr=params->p[1].y;
    
    dxl=params->p[3].x-xl;
    dyl=params->p[3].y-yl;
    dxr=params->p[2].x-xr;
    dyr=params->p[2].y-yr;
    
    // perspective warping or not?
    if (perspective) {
	if (!camIntersectionSegments(params->p,&I)) {
	    perspective=0;
	} else {
	    // 32 bits fixed point
	    zul=(int)((((CAM_INT64)1)<<48)/(params->p[0].y-I.y));
	    zur=(int)((((CAM_INT64)1)<<48)/(params->p[1].y-I.y));
	    zbl=(int)((((CAM_INT64)1)<<48)/(params->p[3].y-I.y));
	    zbr=(int)((((CAM_INT64)1)<<48)/(params->p[2].y-I.y));
	    cl=(zul-zbl)/height; // 32 bits fixed point
	    cr=(zur-zbr)/height; // Should be positive valued
	}
    }
    if (!perspective) {
	incxl=dxl/height;
	incyl=dyl/height;
	incxr=dxr/height;
	incyr=dyr/height;
    } else {
	incxl=(int)((((CAM_INT64)dxl)<<16)/dyl);
	incxr=(int)((((CAM_INT64)dxr)<<16)/dyr);
    }

    // First scan-line
    // Let's go across each horizontal pixel
    xp=xl; yp=yl;
    incxp=(xr-xl)/width;
    incyp=(yr-yl)/width;
    for (x=0;x<=width;x++,xp+=incxp,yp+=incyp) {
	xpl[x]=xp;
	ypl[x]=yp;		
	// Let's retrieve the pixel at that position in the source image
	xx=xp>>16;
	yy=yp>>16;
	GET_PIXEL(xx,yy,1);
	scanline[x]=valpix1;
    }
    
    // For all destination pixels
    // Let's go across all the lines
    for (y=0;y<height;y++) {
	if (perspective) {
	    zl=zul-(int)(((CAM_INT64)y)*cl); // 32 bits fixed points
	    zr=zur-(int)(((CAM_INT64)y)*cr);
	    yl=I.y+(int)((((CAM_INT64)1)<<48)/zl);
	    yr=I.y+(int)((((CAM_INT64)1)<<48)/zr);
	    xl=params->p[0].x+(int)(((CAM_INT64)incxl)*(yl-params->p[0].y)>>16);
	    xr=params->p[1].x+(int)(((CAM_INT64)incxr)*(yr-params->p[1].y)>>16);
	} else {
	    // Go to the next line
	    xl+=incxl;
	    xr+=incxr;
	    yl+=incyl;
	    yr+=incyr;
	}
	cpdstptr=dstptr;
	xp=xl; yp=yl;
	incxp=(xr-xl)/width;
	incyp=(yr-yl)/width;
	// Let's retrieve the value of the first pixel in the source image
	xx=xp>>16;
	yy=yp>>16;
	GET_PIXEL(xx,yy,1);
	// And then across each horizontal pixel
	for (x=0;x<width;x++,dstptr++) {
	    // Manage the positions of pixels
	    xpp=xp;
	    ypp=yp;
	    xp+=incxp;
	    yp+=incyp;
	    // Manage the values of pixels
	    valpixp=valpix1;
	    // Let's retrieve the value of the current pixel in the source image
	    xx=xp>>16;
	    yy=yp>>16;
	    GET_PIXEL(xx,yy,1);
	    // OK. Now we do have our four source points
	    // These are pl[x], pl[x+1], pp and p
	    // (pl = previous line, pp = previous point, p = actual point)
	    // ((x,y) access trough xpl and ypl variables for pl)
	    // Their pixel values are respectively
	    // scanline[x], scanline[x+1], valpixp and valpix1
	    
	    // We can compute the interpolation between them
	    switch (params->interpolation) {
	    case 1: // linear "square" interpolation (slow but nice for resampling)
		w1=((0xffff-(xpl[x]&0xffff))>>8)*((0xffff-(ypl[x]&0xffff))>>8);
		w2=((xpl[x+1]&0xffff)>>8)*((0xffff-(ypl[x+1]&0xffff))>>8);
		w3=((0xffff-(xpp&0xffff))>>8)*((ypp&0xffff)>>8);
		w4=((xp&0xffff)>>8)*((yp&0xffff)>>8);
		sum=w1+w2+w3+w4;
		if (sum) {
    		    result=(((int)scanline[x])*w1+
			((int)scanline[x+1])*w2+
			((int)valpixp)*w3+
			((int)valpix1)*w4)/(w1+w2+w3+w4);
			break;
		}
	    case 0: // Very crude (but fast) interpolation scheme
	    default:
		result=((int)scanline[x]+
			(int)scanline[x+1]+
			(int)valpixp+
			(int)valpix1)>>2;
		break;
	    }
	    // Write the result to the destination image
	    *dstptr=(CAM_PIXEL)result;
	    
	    // Go to the next point
	    // Copy to pl (previous line) in order to prepare for the next horizontal scan
	    xpl[x]=xpp;
	    ypl[x]=ypp; 		
	    // And copy the value of the previous pixel into the scanline buffer
	    scanline[x]=valpixp;
	}
	// Copy to pl (previous line) to prepare for the next horizontal scan	
	xpl[x]=xp;
	ypl[x]=yp;
	// And copy the value of the last pixel into the scanline buffer
	scanline[x]=valpix1;
	
	// Move the destination pointer
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);
    }
    return 1;
}

int camIntersectionSegments(CamPoint p[4], CamPoint *res)
{
    CamPoint A=p[0],B=p[3],C=p[1],D=p[2];
    CAM_INT64 c1,c2;
    if ((B.y==A.y)||(D.y==C.y)) {
	if ((B.x==A.x)||(D.x==C.x)) {
	    return 0;
	}
	c1=(((CAM_INT64)(B.y-A.y))<<16)/(B.x-A.x); // 16 bits fixed point
	c2=(((CAM_INT64)(D.y-C.y))<<16)/(D.x-C.x);
	if (c1==c2) {
	    // Parallel lines
	    return 0;
	}
	res->x=(int)(((C.y-A.y+((A.x*c1-C.x*c2)>>16))<<16)/(c1-c2));
	res->y=(int)(A.y+(((res->x-A.x)*c1)>>16));
	return 1;
    }
    c1=(((CAM_INT64)(B.x-A.x))<<16)/(B.y-A.y); // 16 bits fixed point
    c2=(((CAM_INT64)(D.x-C.x))<<16)/(D.y-C.y);
    if (c1==c2) {
	// Parallel lines
	return 0;
    }
    res->y=(int)(((C.x-A.x+((A.y*c1-C.y*c2)>>16))<<16)/(c1-c2));
    res->x=(int)(A.x+(((res->y-A.y)*c1)>>16));
    return 1;
}


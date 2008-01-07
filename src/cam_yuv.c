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

// Fast yuv2rgb conversion

static long int LUTYUV2RGB[256]={-1};
static long int BU[256];
static long int GU[256];
static long int GV[256];
static long int to76309[256];
static unsigned char clip[1024]; // For clipping in CCIR601

static void camInitLUTYUV2RGB();

int camYUV2RGB(CamImage* source, CamImage *dest)
{
    unsigned char *srcptr,*dstptr;
    int x,y;
    int y1,u,v; 
    int c1,c2,c3,c4;
    int width, height;
    CamInternalROIPolicyStruct iROI;
    DECLARE_MASK_MANAGEMENT;  

    CAM_CHECK_ARGS2(camYUV2RGB,source->imageData!=NULL,"source image is not allocated");
    if (dest->imageData==NULL) {
        // Automatic allocation
        camAllocateRGBImage(dest,source->width,source->height);
    }
    CAM_CHECK(camYUV2RGB,camInternalROIPolicy(source, dest, &iROI, CAM_MASK_SUPPORT | CAM_IGNORE_COI_MISMATCH));
    CAM_CHECK_ARGS(camYUV2RGB,source->nChannels==3);
    CAM_CHECK_ARGS(camYUV2RGB,dest->nChannels==3);
    CAM_CHECK_ARGS(camYUV2RGB,dest->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camYUV2RGB,source->depth==CAM_DEPTH_8U);
    CAM_CHECK_ARGS(camYUV2RGB,dest->depth==CAM_DEPTH_8U);
    CAM_CHECK_ARGS(camYUV2RGB,(*((int*)source->colorModel)==*((int*)"YUV")));
    CAM_CHECK_ARGS(camYUV2RGB,(*((int*)dest->colorModel)==*((int*)"RGB")));
    CAM_CHECK_ARGS(camYUV2RGB,(*((int*)dest->channelSeq)==*((int*)"RGB")));
   
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    INIT_MASK_MANAGEMENT;

    // Initialize tables if need be
    if (LUTYUV2RGB[0]==-1) camInitLUTYUV2RGB();
    
    if (source->dataOrder==CAM_DATA_ORDER_PIXEL) {
        for (y = 0; y < height; y ++) { 
            srcptr=(unsigned char*)(iROI.srcptr+y*source->widthStep);
            dstptr=(unsigned char*)(iROI.dstptr+y*dest->widthStep);
            BEGIN_MASK_MANAGEMENT(
		srcptr+=startx*3;
		dstptr+=startx*3;
	    )                                        

            for (x=startx;x<endx;x++) {
                
                y1 = to76309[*srcptr++];	
                u = *srcptr++;
                v = *srcptr++;
                
                c1 = LUTYUV2RGB[v];
                c2 = GU[u];
                c3 = GV[v];
                c4 = BU[u];
                
                *dstptr++ = clip[384+((y1 + c1)>>16)];  
                *dstptr++ = clip[384+((y1 - c2 - c3)>>16)];
                *dstptr++ = clip[384+((y1 + c4)>>16)];
            }
	    END_MASK_MANAGEMENT;
        }
    } else {
        int offset=source->imageSize/3;
        int doffset=offset*2;
        for (y = 0; y < height; y ++) { 
            srcptr=(unsigned char*)(iROI.srcptr+y*source->widthStep);
            dstptr=(unsigned char*)(iROI.dstptr+y*dest->widthStep);
            BEGIN_MASK_MANAGEMENT(
		srcptr+=startx;
		dstptr+=startx*3;
	    )                                        

            for (x=startx;x<endx;x++) {
                
                y1 = to76309[*srcptr];	
                u = *(srcptr+offset);
                v = *(srcptr+doffset);
                
                c1 = LUTYUV2RGB[v];
                c2 = GU[u];
                c3 = GV[v];
                c4 = BU[u];
                
                *dstptr++ = clip[384+((y1 + c1)>>16)];  
                *dstptr++ = clip[384+((y1 - c2 - c3)>>16)];
                *dstptr++ = clip[384+((y1 + c4)>>16)];

                srcptr++;
            }
	    END_MASK_MANAGEMENT;
        }    
    }
    camInternalROIPolicyExit(&iROI);
    return 1;
}

void camInitLUTYUV2RGB()
{
    long int crv,cbu,cgu,cgv;
    int i,ind;   
    
    crv = 104597; cbu = 132201;
    cgu = 25675;  cgv = 53279;
    
    for (i = 0; i < 256; i++) {
        LUTYUV2RGB[i] = (i-128) * crv;
        BU[i] = (i-128) * cbu;
        GU[i] = (i-128) * cgu;
        GV[i] = (i-128) * cgv;
        to76309[i] = 76309*(i-16);
    }
    
    for (i=0; i<384; i++)
        clip[i] =0;
    ind=384;
    for (i=0;i<256; i++)
        clip[ind++]=i;
    ind=640;
    for (i=0;i<384;i++)
        clip[ind++]=255;
}

static int YR[256], YG[256], YB[256];
static int UR[256], UG[256], UBVR[256];
static int          VG[256], VB[256]={-1};

static void camInitLUTRGB2YUV();

int camRGB2YUV(CamImage *source, CamImage *dest)
{
    int x,y,width,height;
    unsigned char *r, *g, *b;
    unsigned char *rx,*gx,*bx;
    unsigned char *yy, *u, *v;
    unsigned char *yx,*ux,*vx;
    CamInternalROIPolicyStruct iROI;
    DECLARE_MASK_MANAGEMENT;  

    CAM_CHECK_ARGS2(camRGB2YUV,source->imageData!=NULL,"source image is not allocated");
    if (dest->imageData==NULL) {
        // Automatic allocation
        camAllocateYUVImage(dest,source->width,source->height);
    }
    CAM_CHECK(camRGB2YUV,camInternalROIPolicy(source, dest, &iROI, CAM_MASK_SUPPORT | CAM_IGNORE_COI_MISMATCH));
    CAM_CHECK_ARGS(camRGB2YUV,source->nChannels>=3);
    CAM_CHECK_ARGS(camRGB2YUV,dest->nChannels==3);
    CAM_CHECK_ARGS(camRGB2YUV,source->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camRGB2YUV,dest->dataOrder==CAM_DATA_ORDER_PLANE);
    CAM_CHECK_ARGS(camRGB2YUV,source->depth==CAM_DEPTH_8U);
    CAM_CHECK_ARGS(camRGB2YUV,dest->depth==CAM_DEPTH_8U);
    CAM_CHECK_ARGS(camRGB2YUV,(*((int*)dest->colorModel)==*((int*)"YUV")));
    CAM_CHECK_ARGS(camRGB2YUV,(*((int*)source->colorModel)==*((int*)"RGB"))||(*((int*)source->colorModel)==*((int*)"RGBA")));
    
    if (VB[0]==-1) camInitLUTRGB2YUV();
    
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    INIT_MASK_MANAGEMENT;

    if (*((int*)source->channelSeq)==*((int*)"BGR") || *((int*)source->channelSeq)==*((int*)"BGRA")) {
        bx=(unsigned char*)iROI.srcptr;
        gx=bx+1;
        rx=bx+2;
    } else if (*((int*)source->channelSeq)==*((int*)"GRB") || *((int*)source->channelSeq)==*((int*)"GRBA")) {
        gx=(unsigned char*)iROI.srcptr;
        rx=gx+1;
        bx=gx+2;
    } else { /* Assumes RGB */
        rx=(unsigned char*)iROI.srcptr;
        gx=rx+1;
        bx=rx+2;
    }
    
    yx=(unsigned char*)iROI.dstptr;
    ux=yx+dest->imageSize/3;
    vx=yx+2*dest->imageSize/3;
    for (y=0;y<height;y++){
        r=rx;b=bx;g=gx;
        yy=yx;u=ux;v=vx;
        
	BEGIN_MASK_MANAGEMENT(
	    r=rx+startx*iROI.srcinc;
            b=bx+startx*iROI.srcinc;
            g=gx+startx*iROI.srcinc;
            yy=yx+startx;
            u=ux+startx;
            v=vx+startx;
        )                                        

            for (x=startx;x<endx;x++) {
                *yy++ = ( YR[*r]  +YG[*g]+YB[*b]+1048576)>>16;
                *u++  = (-UR[*r]  -UG[*g]+UBVR[*b]+8388608)>>16;
                *v++  = ( UBVR[*r]-VG[*g]-VB[*b]+8388608)>>16;

                r+=iROI.srcinc;
                g+=iROI.srcinc;
                b+=iROI.srcinc;
            }
	END_MASK_MANAGEMENT

        rx+=source->widthStep;
        gx+=source->widthStep;
        bx+=source->widthStep;
        yx+=dest->widthStep;
        ux+=dest->widthStep;
        vx+=dest->widthStep;
    }
    camInternalROIPolicyExit(&iROI);
    return 1;
}

int camRGB2Y(CamImage *source, CamImage *dest)
{
    int x,y,width,height;
    unsigned char *r, *g, *b;
    unsigned char *rx,*gx,*bx;
    unsigned char *yy;
    unsigned char *yx;
    CamInternalROIPolicyStruct iROI;
    DECLARE_MASK_MANAGEMENT;  

    CAM_CHECK_ARGS2(camRGB2Y,source->imageData!=NULL,"source image is not allocated");
    if (dest->imageData==NULL) {
        // Automatic allocation
        camAllocateImage(dest,source->width,source->height,CAM_DEPTH_8U);
    }
    CAM_CHECK(camRGB2Y,camInternalROIPolicy(source, dest, &iROI, CAM_MASK_SUPPORT | CAM_IGNORE_COI_MISMATCH));
    CAM_CHECK_ARGS(camRGB2Y,source->nChannels>=3);
    CAM_CHECK_ARGS(camRGB2Y,dest->nChannels==1);
    CAM_CHECK_ARGS(camRGB2Y,source->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camRGB2Y,source->depth==CAM_DEPTH_8U);
    CAM_CHECK_ARGS(camRGB2Y,dest->depth==CAM_DEPTH_8U);
    CAM_CHECK_ARGS(camRGB2Y,(*((int*)source->colorModel)==*((int*)"RGB"))||(*((int*)source->colorModel)==*((int*)"RGBA")));
    
    if (VB[0]==-1) camInitLUTRGB2YUV();
    
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    INIT_MASK_MANAGEMENT;

    if (*((int*)source->channelSeq)==*((int*)"BGR") || *((int*)source->channelSeq)==*((int*)"BGRA")) {
        bx=(unsigned char*)iROI.srcptr;
        gx=bx+1;
        rx=bx+2;
    } else if (*((int*)source->channelSeq)==*((int*)"GRB") || *((int*)source->channelSeq)==*((int*)"GRBA")) {
        gx=(unsigned char*)iROI.srcptr;
        rx=gx+1;
        bx=gx+2;
    } else { /* Assumes RGB */
        rx=(unsigned char*)iROI.srcptr;
        gx=rx+1;
        bx=rx+2;
    }
    
    yx=(unsigned char*)iROI.dstptr;
    for (y=0;y<height;y++){
        r=rx;b=bx;g=gx;
        yy=yx;
        
	BEGIN_MASK_MANAGEMENT(
	    r=rx+startx*iROI.srcinc;
            b=bx+startx*iROI.srcinc;
            g=gx+startx*iROI.srcinc;
            yy=yx+startx;
        )                                        

            for (x=startx;x<endx;x++) {
                *yy++ = ( YR[*r]  +YG[*g]+YB[*b]+1048576)>>16;

                r+=iROI.srcinc;
                g+=iROI.srcinc;
                b+=iROI.srcinc;
            }
	END_MASK_MANAGEMENT

        rx+=source->widthStep;
        gx+=source->widthStep;
        bx+=source->widthStep;
        yx+=dest->widthStep;
    }
    camInternalROIPolicyExit(&iROI);
    return 1;
}

void camInitLUTRGB2YUV()
{
    int i;
    
    for (i = 0; i < 256; i++) YR[i] = (int)((float)65.481 * (i<<8));
    for (i = 0; i < 256; i++) YG[i] = (int)((float)128.553 * (i<<8));
    for (i = 0; i < 256; i++) YB[i] = (int)((float)24.966 * (i<<8));
    for (i = 0; i < 256; i++) UR[i] = (int)((float)37.797 * (i<<8));
    for (i = 0; i < 256; i++) UG[i] = (int)((float)74.203 * (i<<8));
    for (i = 0; i < 256; i++) VG[i] = (int)((float)93.786 * (i<<8));
    for (i = 0; i < 256; i++) VB[i] = (int)((float)18.214 * (i<<8));
    for (i = 0; i < 256; i++) UBVR[i] = (int)((float)112 * (i<<8));
}





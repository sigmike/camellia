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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "camellia.h"
#include "camellia_internals.h"

#ifndef CAM_INLINED
#define CAM_INLINED

static int camDrawCircleData[1024]={0};

int camDrawCircleInit(void)
{
    int i;
    if (camDrawCircleData[0]==0) {
        for (i=0;i<1024;i++) {
            double x=((double)i)/1024;
            camDrawCircleData[i]=(int)(sqrt(1-x*x)*1024+0.5);
        }
    }
    return 1;
}

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation

#undef CAM_PIXEL
#define CAM_PIXEL unsigned char
#define camDrawLine camDrawLine8
#define camAccumulateLine camAccumulateLine8
#define camDrawRectangle camDrawRectangle8
#define camDrawText16s camDrawText16s8
#define camDrawCircle camDrawCircle8
#define camDrawEllipse camDrawEllipse8
#define camPlot camPlot8
#define camFillColor camFillColor8
#include "cam_draw.c"
#undef camDrawLine
#undef camAccumulateLine
#undef camDrawRectangle
#undef camDrawText16s
#undef camDrawCircle
#undef camDrawEllipse
#undef camPlot
#undef camFillColor
 
#undef CAM_PIXEL
#define CAM_PIXEL unsigned short
#define camDrawLine camDrawLine16
#define camAccumulateLine camAccumulateLine16
#define camDrawRectangle camDrawRectangle16
#define camDrawText16s camDrawText16s16
#define camDrawCircle camDrawCircle16
#define camDrawEllipse camDrawEllipse16
#define camPlot camPlot16
#define camFillColor camFillColor16
#include "cam_draw.c"
#undef camDrawLine
#undef camAccumulateLine
#undef camDrawRectangle
#undef camDrawText16s
#undef camDrawCircle
#undef camDrawEllipse
#undef camPlot
#undef camFillColor
 
int camDrawLine(CamImage *image, int x1, int y1, int x2, int y2, int color)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camDrawLine16(image,x1,y1,x2,y2,color);
    } else {
	return camDrawLine8(image,x1,y1,x2,y2,color);
    }
}

int camAccumulateLine(CamImage *image, int x1, int y1, int x2, int y2, int acc)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camAccumulateLine16(image,x1,y1,x2,y2,acc);
    } else {
	return camAccumulateLine8(image,x1,y1,x2,y2,acc);
    }
}

int camDrawRectangle(CamImage *image, int x1, int y1, int x2, int y2, int color)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camDrawRectangle16(image,x1,y1,x2,y2,color);
    } else {
	return camDrawRectangle8(image,x1,y1,x2,y2,color);
    }
}

int camDrawText16s(CamImage *image, char *text, int x, int y, int cwidth, int cheight, int orientation, int color)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camDrawText16s16(image,text,x,y,cwidth,cheight,orientation,color);
    } else {
	return camDrawText16s8(image,text,x,y,cwidth,cheight,orientation,color);
    }
}

int camDrawCircle(CamImage *image, int x, int y, int r, int color)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camDrawCircle16(image,x,y,r,color);
    } else {
	return camDrawCircle8(image,x,y,r,color);
    }
}

int camDrawEllipse(CamImage *image, int x, int y, int rx, int ry, int color)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camDrawEllipse16(image,x,y,rx,ry,color);
    } else {
	return camDrawEllipse8(image,x,y,rx,ry,color);
    }
}

int camPlot(CamImage *image, int x, int y, int color, int kind)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camPlot16(image,x,y,color,kind);
    } else {
	return camPlot8(image,x,y,color,kind);
    }
}

int camFillColor(CamImage *image, int x, int y, int fillcolor, int tolerance)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camFillColor16(image,x,y,fillcolor,tolerance);
    } else {
	return camFillColor8(image,x,y,fillcolor,tolerance);
    }
}

int camDrawTextBitmap(CamImage *image, char *text, int x, int y, CamBitmapFont *font)
{
    CamImage imx=*image;
    CamROI roi;
    char *s=text;
    int n;

    CAM_CHECK_ARGS(camDrawTextBitmapFont,image->imageData!=NULL);
    CAM_CHECK_ARGS(camDrawTextBitmapFont,image->nChannels>=3);

    roi.coi=0;
    imx.roi=&roi;
    for (;*s;s++) {
        n=*s-font->first_char;
        if (n>=0) {
            if (n>=font->nb_chars) {
                n-='a'-'A';
            }
            if ((n>=0)&&(n<font->nb_chars)) {
                roi.xOffset=x;
                roi.yOffset=y;
                roi.width=font->letters[n].width;
                roi.height=font->letters[n].height;
                camCopy(&font->letters[n],&imx);
                x+=font->letters[n].width;
            }
        } else x+=font->height/2;
    }
    return 0;
}

int camLoadBitmapFont(CamBitmapFont *font, char *filename)
{
    CamImage bitmap;
    CamROI roi;
    int delimiter_color,x,i,px;
    unsigned char *ptr;
    CamTable clusters,LUT;
    CamRLEImage temp;
    char s[256];

    if (!camLoadBMP(&bitmap,filename)) {
        sprintf(s,"Couldn't load bitmap file %s.",filename);
        camError("camBitmapFontLoad",s);
        return 0;
    }
    font->height=bitmap.height-1;

    // Try to figure out how many letters are there in this font image
    font->nb_chars=0;
    font->first_char=33;
#define GET_COLOR_PIXEL(ptr) (((int)*(ptr))+(((int)*((ptr)+1))<<8)+(((int)*((ptr)+2))<<16))   
    ptr=bitmap.imageData;
    delimiter_color=GET_COLOR_PIXEL(ptr);
    for (x=0;x<bitmap.width;x++,ptr+=3) {
        if (GET_COLOR_PIXEL(ptr)==delimiter_color) font->nb_chars++;
    }

    // OK, now we can allocate memory for these
    font->masks=(CamRLEImage*)malloc(sizeof(CamRLEImage)*font->nb_chars);
    font->letters=(CamImage*)malloc(sizeof(CamImage)*font->nb_chars);
    
    // Let's prepare the data for font loading
    ptr=bitmap.imageData;
    px=0;
    roi.coi=0;
    bitmap.roi=&roi;
    clusters.size=12;
    for (i=0;i<6;i++) clusters.t[i*2]=clusters.t[i*2+1]=*(ptr+i);
    i=0;
    LUT.size=3;
    LUT.t[0]=1; LUT.t[1]=0; LUT.t[2]=0;
    camRLEAllocate(&temp,10000);

    // Ready. Let's go for all the characters
    for (x=1,ptr+=3;x<bitmap.width;x++,ptr+=3) {
        if (GET_COLOR_PIXEL(ptr)==delimiter_color) {
            // We've found the next character
            camRLEAllocate(&font->masks[i],(x-px)*font->height+2);
            camAllocateRGBImage(&font->letters[i],x-px,font->height);
            roi.xOffset=px; roi.yOffset=1;
            roi.width=x-px;
            roi.height=font->height;
            camCopy(&bitmap,&font->letters[i]);
            camRLEEncodeColor(&bitmap,&temp,&clusters);
            camRLEApplyLUT(&temp,&font->masks[i],&LUT);
            px=x;
            i++;
        }
    }
    // We've found the last character
    camRLEAllocate(&font->masks[i],(x-px)*font->height+2);
    camAllocateRGBImage(&font->letters[i],x-px,font->height);
    roi.xOffset=px; roi.yOffset=1;
    roi.width=x-px;
    roi.height=font->height;
    camCopy(&bitmap,&font->letters[i]);
    camRLEEncodeColor(&bitmap,&temp,&clusters);
    camRLEApplyLUT(&temp,&font->masks[i],&LUT);

    // Set the masks to all letters
    for (i=0;i<font->nb_chars;i++) {
        font->letters[i].mask=&font->masks[i];
    }
    camDeallocateImage(&bitmap);
    camRLEDeallocate(&temp);
    return 1;
}

int camFreeBitmapFont(CamBitmapFont *font)
{
    int i;
    if (font->nb_chars) {
        for (i=0;i<font->nb_chars;i++) {
            camRLEDeallocate(&font->masks[i]);
            camDeallocateImage(&font->letters[i]);
        }
        free(font->masks);
        free(font->letters);
        font->masks=NULL;
        font->letters=NULL;
        font->nb_chars=0;
    }
    return 1;
}

#else
#include "cam_draw.c"
#endif

#else
// Drawing functions
int camDrawLine(CamImage *image, int x1, int y1, int x2, int y2, int color)
{
    int dx, dy, xincr, yincr, x, y, xp, yp, i, j, c, d;
    int inc, error, correction, acc, temp, ptrincr, runl, runl2;
    int blue, green, red;
    CAM_PIXEL *ptrB,*ptrG,*ptrX;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camDrawLine,camInternalROIPolicy(image, NULL, &iROI, 0));
    
    /* Clipping */
    if (x1<x2) {
        x=x1; y=y1; xp=x2; yp=y2;
    } else {
        x=x2; y=y2; xp=x1; yp=y1;
    }
    if (x<iROI.srcroi.xOffset) {
        if (x==xp) return 1;
        y=y+(y-yp)*(iROI.srcroi.xOffset-x)/(x-xp);
        x=iROI.srcroi.xOffset;
        if (xp<iROI.srcroi.xOffset) return 1; /* The line is completely outside the ROI */
    }
    if (xp>=(iROI.srcroi.xOffset+iROI.srcroi.width)) {
        if (x==xp) return 1;
        yp=y+(y-yp)*((iROI.srcroi.xOffset+iROI.srcroi.width)-x-1)/(x-xp);
        xp=(iROI.srcroi.xOffset+iROI.srcroi.width)-1;
        if (x>=(iROI.srcroi.xOffset+iROI.srcroi.width)) return 1; /* The line is completely outside the ROI */
    }
    if (y<yp) {
        x1=x; y1=y; x2=xp; y2=yp;
    } else {
        x1=xp; y1=yp; x2=x; y2=y;
    }
    if (y1<iROI.srcroi.yOffset) {
        if (y1==y2) return 1;
        x1=x1+(x1-x2)*(iROI.srcroi.yOffset-y1)/(y1-y2);
        y1=iROI.srcroi.yOffset;
        if (y2<iROI.srcroi.yOffset) return 1; /* The line is completely outside the ROI */
    }
    if (y2>=(iROI.srcroi.yOffset+iROI.srcroi.height)) {
        if (y1==y2) return 1;
        x2=x1+(x1-x2)*((iROI.srcroi.yOffset+iROI.srcroi.height)-y1-1)/(y1-y2);
        y2=(iROI.srcroi.yOffset+iROI.srcroi.height)-1;
        if (y1>=(iROI.srcroi.yOffset+iROI.srcroi.height)) return 1; /* The line is completely outside the ROI */
    }
    
    if ((iROI.nChannels==1)||(image->dataOrder==CAM_DATA_ORDER_PIXEL)) {
#define LINE
#define SETPIXEL for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[c]=(d&0xff);
#define INITPOINTERS ptrX=((CAM_PIXEL*)(image->imageData+iROI.srcchoffset+y*image->widthStep))+x*iROI.srcinc
#include "cam_draw_code.c"
    } else {
        blue=(color>>16)&0xff;
        green=(color>>8)&0xff;
        red=color&0xff;
#define COLOR
#undef INITPOINTERS
#undef SETPIXEL
#define INITPOINTERS \
    ptrX=((CAM_PIXEL*)(image->imageData+y*image->widthStep))+x; \
    ptrG=((CAM_PIXEL*)(image->imageData+(image->height+y)*image->widthStep))+x; \
    ptrB=((CAM_PIXEL*)(image->imageData+(2*image->height+y)*image->widthStep))+x;
#define SETPIXEL *ptrX=red; *ptrG=green; *ptrB=blue
#include "cam_draw_code.c"
#undef COLOR
#undef INITPOINTERS
#undef SETPIXEL
    }

    return 1;
}

int camAccumulateLine(CamImage *image, int x1, int y1, int x2, int y2, int accumulator)
{
    int dx, dy, xincr, yincr, x, y, xp, yp, i, j;
    int inc, error, correction, acc, temp, ptrincr, runl, runl2;
    CAM_PIXEL *ptrX;
    CamInternalROIPolicyStruct iROI;
#ifdef CAM_DEBUG
    int valmax=(1<<(sizeof(CAM_PIXEL)*8))-1;
    int value;
#endif

    // ROI (Region Of Interest) management
    CAM_CHECK(camAccumulateLine,camInternalROIPolicy(image, NULL, &iROI, 0));
    CAM_CHECK_ARGS(camAccumulateLine,iROI.nChannels==1);
    CAM_CHECK_ARGS2(camAccumulateLine,(image->depth&CAM_DEPTH_SIGN)==0,"Supports only unsigned images");
    /* Clipping */
    if (x1<x2) {
        x=x1; y=y1; xp=x2; yp=y2;
    } else {
        x=x2; y=y2; xp=x1; yp=y1;
    }
    if (x<iROI.srcroi.xOffset) {
        if (x==xp) return 1;
        y=y+(y-yp)*(iROI.srcroi.xOffset-x)/(x-xp);
        x=iROI.srcroi.xOffset;
        if (xp<iROI.srcroi.xOffset) return 1; /* The line is completely outside the ROI */
    }
    if (xp>=(iROI.srcroi.xOffset+iROI.srcroi.width)) {
        if (x==xp) return 1;
        yp=y+(y-yp)*((iROI.srcroi.xOffset+iROI.srcroi.width)-x-1)/(x-xp);
        xp=(iROI.srcroi.xOffset+iROI.srcroi.width)-1;
        if (x>=(iROI.srcroi.xOffset+iROI.srcroi.width)) return 1; /* The line is completely outside the ROI */
    }
    if (y<yp) {
        x1=x; y1=y; x2=xp; y2=yp;
    } else {
        x1=xp; y1=yp; x2=x; y2=y;
    }
    if (y1<iROI.srcroi.yOffset) {
        if (y1==y2) return 1;
        x1=x1+(x1-x2)*(iROI.srcroi.yOffset-y1)/(y1-y2);
        y1=iROI.srcroi.yOffset;
        if (y2<iROI.srcroi.yOffset) return 1; /* The line is completely outside the ROI */
    }
    if (y2>=(iROI.srcroi.yOffset+iROI.srcroi.height)) {
        if (y1==y2) return 1;
        x2=x1+(x1-x2)*((iROI.srcroi.yOffset+iROI.srcroi.height)-y1-1)/(y1-y2);
        y2=(iROI.srcroi.yOffset+iROI.srcroi.height)-1;
        if (y1>=(iROI.srcroi.yOffset+iROI.srcroi.height)) return 1; /* The line is completely outside the ROI */
    }
    
#define LINE
#ifdef CAM_DEBUG
#define SETPIXEL value=*ptrX; value+=accumulator; if ((value<0)||(value>valmax)) {camError("camAccumulateLane","Saturation"); return 0;} else *ptrX=(CAM_PIXEL)value;
#else
#define SETPIXEL *ptrX+=accumulator;
#endif
#define INITPOINTERS ptrX=((CAM_PIXEL*)(image->imageData+iROI.srcchoffset+y*image->widthStep))+x*iROI.srcinc
#include "cam_draw_code.c"
#undef INITPOINTERS
#undef SETPIXEL
#undef LINE
    return 1;
}

int camDrawRectangle(CamImage *image, int x1, int y1, int x2, int y2, int color)
{
    camDrawLine(image,x1,y1,x2,y1,color);
    camDrawLine(image,x2,y1,x2,y2,color);
    camDrawLine(image,x2,y2,x1,y2,color);
    camDrawLine(image,x1,y2,x1,y1,color);
    return 1;
}

int camDrawText16s(CamImage *image, char *text, int x, int y, int cwidth, int cheight, int orientation, int color)
{
    static const int characters_table[][16]={
        {1,1,1,1,1,1,1,1,0,0,1,0,0,0,1,0}, /* 0 */
        {1,0,0,0,1,1,0,0,0,1,0,0,0,1,0,0}, /* {0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0}, */
        {1,1,1,0,1,1,1,0,0,0,0,1,0,0,0,1},
        {1,1,1,1,1,1,0,0,0,0,0,1,0,0,0,0},
        {0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1},
        {1,1,0,1,1,1,0,1,0,0,0,1,0,0,0,1},
        {1,1,0,1,1,1,1,1,0,0,0,1,0,0,0,1},
        {1,1,0,0,0,0,0,0,0,0,1,0,0,0,1,0},
        {1,1,1,1,1,1,1,1,0,0,0,1,0,0,0,1},
        {1,1,1,1,1,1,0,1,0,0,0,1,0,0,0,1}, /* 9 */
        {1,1,1,1,0,0,1,1,0,0,0,1,0,0,0,1}, /* A */
        {1,1,1,1,1,1,0,0,0,1,0,1,0,1,0,0},
        {1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,0},
        {1,1,1,1,1,1,0,0,0,1,0,0,0,1,0,0},
        {1,1,0,0,1,1,1,1,0,0,0,0,0,0,0,1},
        {1,1,0,0,0,0,1,1,0,0,0,0,0,0,0,1},
        {1,1,0,1,1,1,1,1,0,0,0,1,0,0,0,0},
        {0,0,1,1,0,0,1,1,0,0,0,1,0,0,0,1},
        {0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0},
        {0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,1,1,0,0,1,0,1,0,0,1},
        {0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0},
        {0,0,1,1,0,0,1,1,1,0,1,0,0,0,0,0},
        {0,0,1,1,0,0,1,1,1,0,0,0,1,0,0,0},
        {1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
        {1,1,1,0,0,0,1,1,0,0,0,1,0,0,0,1},
        {1,1,1,1,1,1,1,1,0,0,0,0,1,0,0,0},
        {1,1,1,0,0,0,1,1,0,0,0,1,1,0,0,1},
        {1,1,0,1,1,1,0,1,0,0,0,1,0,0,0,1},
        {1,1,0,0,0,0,0,0,0,1,0,0,0,1,0,0},
        {0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
        {0,0,0,0,0,0,1,1,0,0,1,0,0,0,1,0},
        {0,0,1,1,0,0,1,1,0,0,0,0,1,0,1,0},
        {0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0},
        {0,0,0,0,0,0,0,0,1,0,1,0,0,1,0,0},
        {1,1,0,0,1,1,0,0,0,0,1,0,0,0,1,0}, /* Z */
        {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
        
    static const int segments[16][4]={
        {0,0,1,0},{1,0,2,0},{2,0,2,1},{2,1,2,2},
        {1,2,2,2},{0,2,1,2},{0,1,0,2},{0,0,0,1},
        {0,0,1,1},{1,0,1,1},{2,0,1,1},{2,1,1,1},
        {2,2,1,1},{1,2,1,1},{0,2,1,1},{0,1,1,1}
    };
        
    int i,j,l;
    int xp[3],yp[3];
    const int *character;
    int special_character;
    char carac;
    
    CAM_CHECK_ARGS(camDrawText16s,image->imageData!=NULL);
    
    if (orientation) {
        xp[0]=0; xp[1]=2*cwidth/5; xp[2]=4*cwidth/5;
        yp[0]=cheight; yp[1]=cheight-cheight/3; yp[2]=cheight-2*cheight/3;
    } else {
        xp[0]=0; xp[1]=cwidth/3; xp[2]=2*cwidth/3;
        yp[0]=0; yp[1]=2*cheight/5; yp[2]=4*cheight/5;
    }
    
    l=strlen(text);
    
    for (i=0;i<l;i++) {
        if (orientation) {
            carac=text[l-1-i];
        } else {
            carac=text[i];
        }
        special_character=0;
        if ((carac>='0')&&(carac<='9')) {
            character=characters_table[carac-'0'];
        } else if ((carac>='A')&&(carac<='Z')) {
            character=characters_table[carac-'A'+10];
        } else if ((carac>='a')&&(carac<='z')) {
            character=characters_table[carac-'a'+10];
        } else if (carac==' ') {
            /* Space character */
            special_character=1;
        } else if (carac==':') {
            special_character=1;
            if (orientation) {
                camDrawLine(image,x+xp[1],y+yp[1],x+xp[1],y+yp[1],color);
                camDrawLine(image,x+xp[2],y+yp[1],x+xp[2],y+yp[1],color);
            } else {
                camDrawLine(image,x+xp[1],y+yp[1],x+xp[1],y+yp[1],color);
                camDrawLine(image,x+xp[1],y+yp[2],x+xp[1],y+yp[2],color);
            }
        } else if (carac=='.') {
            special_character=1;
            if (orientation) {
                camDrawLine(image,x+xp[2],y+yp[1],x+xp[2],y+yp[1],color);
            } else {
                camDrawLine(image,x+xp[1],y+yp[2],x+xp[1],y+yp[2],color);
            }
        } else if (carac=='=') {
            special_character=1;
            if (orientation) {
                camDrawLine(image,x+xp[1],y+yp[0],x+xp[1],y+yp[2],color);
                camDrawLine(image,x+xp[2],y+yp[0],x+xp[2],y+yp[2],color);
            } else {
                camDrawLine(image,x+xp[0],y+yp[1],x+xp[2],y+yp[1],color);
                camDrawLine(image,x+xp[0],y+yp[2],x+xp[2],y+yp[2],color);
            }
        } else if (carac=='-') {
            special_character=1;
            if (orientation) {
                camDrawLine(image,x+xp[1],y+yp[0],x+xp[1],y+yp[2],color);
            } else {
                camDrawLine(image,x+xp[0],y+yp[1],x+xp[2],y+yp[1],color);
            }
        }
        
        if (!special_character) {
            if (orientation) {
                for (j=0;j<16;j++) {
                    if (character[j]) {
                        camDrawLine(image,x+xp[segments[j][1]],y+yp[segments[j][0]],x+xp[segments[j][3]],y+yp[segments[j][2]],color);
                    }
                }
            } else {
                for (j=0;j<16;j++) {
                    if (character[j]) {
                        camDrawLine(image,x+xp[segments[j][0]],y+yp[segments[j][1]],x+xp[segments[j][2]],y+yp[segments[j][3]],color);
                    }
                }
            }
        }
        if (orientation) {
            y+=cheight;
        } else {
            x+=cwidth;
        }
    }
    return 1;
}

int camDrawCircle(CamImage *image, int xc, int yc, int radius, int color)
{
    int x,y,run,error,pos,acc,val,curval,incx,incy,yp,c,d;
    int clipping;
    int rx=radius,ry=radius;
    CAM_PIXEL *ptrB[4], *ptrG[4], *ptrX[4];
    CAM_PIXEL blue,red,green;    
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camDrawLine,camInternalROIPolicy(image, NULL, &iROI, 0));
    if (radius==0) return 1;
    
    if (radius>0) {
        if (camDrawCircleData[0]==0) {
            camDrawCircleInit();
        }
        
        incx=iROI.srcinc;
        incy=iROI.srclinc;
        
        clipping=((xc-radius<iROI.srcroi.xOffset)||(xc+radius>=iROI.srcroi.yOffset+iROI.srcroi.width)||(yc-radius<iROI.srcroi.yOffset)||(yc+radius>=iROI.srcroi.yOffset+iROI.srcroi.height));
        
#define CIRCLE
#define TEST(sgnx,sgny) if ((xc+sgnx*x>=iROI.srcroi.xOffset)&&(xc+sgnx*x<iROI.srcroi.yOffset+iROI.srcroi.width)&&(yc+sgny*y>=iROI.srcroi.yOffset)&&(yc+sgny*y<iROI.srcroi.yOffset+iROI.srcroi.height))
        if ((iROI.nChannels==1)||(image->dataOrder==CAM_DATA_ORDER_PIXEL)) {
            if (clipping) {
#define SETPIXEL0 TEST(1,-1) for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[0][c]=(d&0xff);
#define SETPIXEL1 TEST(1,1) for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[1][c]=(d&0xff);
#define SETPIXEL2 TEST(-1,-1) for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[2][c]=(d&0xff);
#define SETPIXEL3 TEST(-1,1) for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[3][c]=(d&0xff);
#define INITPOINTERS(i,sgny) \
    yp=yc+sgny*radius; \
    ptrX[i]=((CAM_PIXEL*)(image->imageData+iROI.srcchoffset+yp*image->widthStep))+xc*iROI.srcinc
#include "cam_draw_code.c"
#undef SETPIXEL0
#undef SETPIXEL1
#undef SETPIXEL2
#undef SETPIXEL3
            } else {
#define SETPIXEL0 for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[0][c]=(d&0xff);
#define SETPIXEL1 for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[1][c]=(d&0xff);
#define SETPIXEL2 for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[2][c]=(d&0xff);
#define SETPIXEL3 for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[3][c]=(d&0xff);
#include "cam_draw_code.c"
#undef SETPIXEL0
#undef SETPIXEL1
#undef SETPIXEL2
#undef SETPIXEL3
#undef INITPOINTERS
            }
        } else {
#define COLOR
            blue=(color>>16)&0xff;
            green=(color>>8)&0xff;
            red=color&0xff;
            if (clipping) {
#define INITPOINTERS(i,sgny) \
    yp=yc+sgny*radius; \
    ptrX[i]=((CAM_PIXEL*)(image->imageData+yp*image->widthStep))+xc; \
    ptrG[i]=((CAM_PIXEL*)(image->imageData+(image->height+yp)*image->widthStep))+xc; \
    ptrB[i]=((CAM_PIXEL*)(image->imageData+(2*image->height+yp)*image->widthStep))+xc;
#define SETPIXEL0 TEST(1,-1) {*ptrX[0]=red; *ptrG[0]=green; *ptrB[0]=blue;}
#define SETPIXEL1 TEST(1,1) {*ptrX[1]=red; *ptrG[1]=green; *ptrB[1]=blue;}
#define SETPIXEL2 TEST(-1,-1) {*ptrX[2]=red; *ptrG[2]=green; *ptrB[2]=blue;}
#define SETPIXEL3 TEST(-1,1) {*ptrX[3]=red; *ptrG[3]=green; *ptrB[3]=blue;}
#include "cam_draw_code.c"
#undef SETPIXEL0
#undef SETPIXEL1
#undef SETPIXEL2
#undef SETPIXEL3
            } else {
#define SETPIXEL0 *ptrX[0]=red; *ptrG[0]=green; *ptrB[0]=blue
#define SETPIXEL1 *ptrX[1]=red; *ptrG[1]=green; *ptrB[1]=blue
#define SETPIXEL2 *ptrX[2]=red; *ptrG[2]=green; *ptrB[2]=blue
#define SETPIXEL3 *ptrX[3]=red; *ptrG[3]=green; *ptrB[3]=blue
#include "cam_draw_code.c"
#undef INITPOINTERS
#undef SETPIXEL0
#undef SETPIXEL1
#undef SETPIXEL2
#undef SETPIXEL3
            }
#undef COLOR
        }
#undef CIRCLE
    }
    return 1;
}	

int camDrawEllipse(CamImage *image, int xc, int yc, int rx, int ry, int color)
{
    int x,y,run,error,pos,acc,val,curval,incx,incy,yp,c,d;
    int clipping;
    CAM_PIXEL *ptrB[4], *ptrG[4], *ptrX[4];
    CAM_PIXEL blue,red,green;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camDrawLine,camInternalROIPolicy(image, NULL, &iROI, 0));
    if ((rx==0)||(ry==0)) return 1;    

    if (camDrawCircleData[0]==0) {
        camDrawCircleInit();
    }
    
    incx=iROI.srcinc;
    incy=iROI.srclinc;
        
    clipping=((xc-rx<iROI.srcroi.xOffset)||(xc+rx>=iROI.srcroi.yOffset+iROI.srcroi.width)||(yc-ry<iROI.srcroi.yOffset)||(yc+ry>=iROI.srcroi.yOffset+iROI.srcroi.height));
    
#define CIRCLE
    if ((iROI.nChannels==1)||(image->dataOrder==CAM_DATA_ORDER_PIXEL)) {
        if (clipping) {
#define SETPIXEL0 TEST(1,-1) for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[0][c]=(d&0xff);
#define SETPIXEL1 TEST(1,1) for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[1][c]=(d&0xff);
#define SETPIXEL2 TEST(-1,-1) for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[2][c]=(d&0xff);
#define SETPIXEL3 TEST(-1,1) for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[3][c]=(d&0xff);
#define INITPOINTERS(i,sgny) \
    yp=yc+sgny*ry; \
    ptrX[i]=((CAM_PIXEL*)(image->imageData+iROI.srcchoffset+yp*image->widthStep))+xc*iROI.srcinc
#include "cam_draw_code.c"
#undef SETPIXEL0
#undef SETPIXEL1
#undef SETPIXEL2
#undef SETPIXEL3
        } else {
#define SETPIXEL0 for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[0][c]=(d&0xff);
#define SETPIXEL1 for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[1][c]=(d&0xff);
#define SETPIXEL2 for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[2][c]=(d&0xff);
#define SETPIXEL3 for(c=0,d=color;c<iROI.nChannels;c++,d>>=8) ptrX[3][c]=(d&0xff);
#include "cam_draw_code.c"
#undef SETPIXEL0
#undef SETPIXEL1
#undef SETPIXEL2
#undef SETPIXEL3
#undef INITPOINTERS
        }
    } else {
#define COLOR
        blue=(color>>16)&0xff;
        green=(color>>8)&0xff;
        red=color&0xff;
        if (clipping) {
#define INITPOINTERS(i,sgny) \
    yp=yc+sgny*ry; \
    ptrX[i]=((CAM_PIXEL*)(image->imageData+yp*image->widthStep))+xc; \
    ptrG[i]=((CAM_PIXEL*)(image->imageData+(image->height+yp)*image->widthStep))+xc; \
    ptrB[i]=((CAM_PIXEL*)(image->imageData+(2*image->height+yp)*image->widthStep))+xc;
#define SETPIXEL0 TEST(1,-1) {*ptrX[0]=red; *ptrG[0]=green; *ptrB[0]=blue;}
#define SETPIXEL1 TEST(1,1) {*ptrX[1]=red; *ptrG[1]=green; *ptrB[1]=blue;}
#define SETPIXEL2 TEST(-1,-1) {*ptrX[2]=red; *ptrG[2]=green; *ptrB[2]=blue;}
#define SETPIXEL3 TEST(-1,1) {*ptrX[3]=red; *ptrG[3]=green; *ptrB[3]=blue;}
#include "cam_draw_code.c"
#undef SETPIXEL0
#undef SETPIXEL1
#undef SETPIXEL2
#undef SETPIXEL3
        } else {
#define SETPIXEL0 *ptrX[0]=red; *ptrG[0]=green; *ptrB[0]=blue
#define SETPIXEL1 *ptrX[1]=red; *ptrG[1]=green; *ptrB[1]=blue
#define SETPIXEL2 *ptrX[2]=red; *ptrG[2]=green; *ptrB[2]=blue
#define SETPIXEL3 *ptrX[3]=red; *ptrG[3]=green; *ptrB[3]=blue
#include "cam_draw_code.c"
#undef INITPOINTERS
#undef SETPIXEL0
#undef SETPIXEL1
#undef SETPIXEL2
#undef SETPIXEL3
        }
#undef COLOR
    }
#undef CIRCLE
    return 1;
}	

int camPlot(CamImage *image, int x, int y, int color, int kind)
{
    if (kind&1) {
        camDrawLine(image,x,y,x,y,color);
    }
    if (kind&2) {
	camDrawLine(image,x-2,y-2,x+2,y+2,color);
        camDrawLine(image,x+2,y-2,x-2,y+2,color);
    }
    if (kind&4) {
	camDrawCircle(image,x,y,2,color);
    }
    return 1;
}

// Queue management
#define FIFO_SIZE	CAM_MAX_SCANLINE*4
#define FIFO_NEXT()	first++; if (first==FIFO_SIZE) first=0
#define FIFO_ADD(x,y)	queuex[last]=(x); queuey[last++]=(y); if (last==FIFO_SIZE) last=0
#define FIFO_EMPTY()	(first==last)

int camFillColor(CamImage *image, int x, int y, int fillcolor, int tolerance)
{
    int first=0,last=0;
    int i,j,d,xp,yp;
    CAM_PIXEL *ptr,*ptrx;
    const int nx[4]={-1,0,+1,0},ny[4]={0,-1,0,+1};
    CAM_PIXEL pcolor[4],initcolor[4]; // 4 is the maximum number of channels
    CamInternalROIPolicyStruct iROI;
    int acc=1;

    int queuex[FIFO_SIZE];
    int queuey[FIFO_SIZE];

    // ROI (Region Of Interest) management
    CAM_CHECK(camFillColor,camInternalROIPolicy(image, NULL, &iROI, 0));
    
    CAM_CHECK_ARGS(camFillColor, ((iROI.nChannels==1)||(image->dataOrder==CAM_DATA_ORDER_PIXEL)));

    if ((x>=iROI.srcroi.xOffset)&&(y>=iROI.srcroi.yOffset)&&(x<iROI.srcroi.xOffset+iROI.srcroi.width)&&(y<iROI.srcroi.yOffset+iROI.srcroi.height)) {
        for (i=0;i<iROI.nChannels;i++) {
            pcolor[i]=(fillcolor>>(i*8))&0xff;	
        }
        ptr=ptrx=(CAM_PIXEL*)(image->imageData+iROI.srcchoffset+y*image->widthStep)+x*iROI.srcinc;
        if (tolerance>=0) {            
            for (i=0;i<iROI.nChannels;i++) {
                initcolor[i]=*ptrx++;	
            }
            FIFO_ADD(x,y);
            for (ptrx=ptr,i=0;i<iROI.nChannels;i++,ptrx++) {
                *ptrx=pcolor[i];
            }
            while (!FIFO_EMPTY()) {
                x=queuex[first];
                y=queuey[first];
                FIFO_NEXT();
                for (j=0;j<4;j++) {
                    xp=x+nx[j];
                    yp=y+ny[j];
                    if ((xp>=iROI.srcroi.xOffset)&&(yp>=iROI.srcroi.yOffset)&&(xp<iROI.srcroi.xOffset+iROI.srcroi.width)&&(yp<iROI.srcroi.yOffset+iROI.srcroi.height)) {
                        // Get the color at (xp,yp)
                        ptr=ptrx=(CAM_PIXEL*)(image->imageData+iROI.srcchoffset+yp*image->widthStep)+xp*iROI.srcinc;
                        // Is it the same color as the initial color?
                        // Compute distance between colors
                        d=0;
                        for (i=0;i<iROI.nChannels;i++,ptrx++) {
                            if (*ptrx>initcolor[i]) d+=*ptrx-initcolor[i];
                            else d+=initcolor[i]-*ptrx;
                        }
                        if (d<=tolerance) {
                            // Yes, then this pixel should be repainted and added to the queue
                            FIFO_ADD(xp,yp);
                            for (ptrx=ptr,i=0;i<iROI.nChannels;i++,ptrx++) {
                                *ptrx=pcolor[i];
                            }
                            acc++;
                        }	    
                    }
                }
            }
        } else {
            FIFO_ADD(x,y);
            for (ptrx=ptr,i=0;i<iROI.nChannels;i++,ptrx++) {
                *ptrx=pcolor[i];
            }
            while (!FIFO_EMPTY()) {
                x=queuex[first];
                y=queuey[first];
                FIFO_NEXT();
                for (j=0;j<4;j++) {
                    xp=x+nx[j];
                    yp=y+ny[j];
                    if ((xp>=iROI.srcroi.xOffset)&&(yp>=iROI.srcroi.yOffset)&&(xp<iROI.srcroi.xOffset+iROI.srcroi.width)&&(yp<iROI.srcroi.yOffset+iROI.srcroi.height)) {
                        // Get the color at (xp,yp)
                        ptr=ptrx=(CAM_PIXEL*)(image->imageData+iROI.srcchoffset+yp*image->widthStep)+xp*iROI.srcinc;
                        for (i=0;i<iROI.nChannels;i++,ptrx++) if (*ptrx!=pcolor[i]) break;
                        // Is it the same color as the fill color?
                        if (i!=iROI.nChannels) {
                            // Yes, then this pixel should be repainted and added to the queue
                            FIFO_ADD(xp,yp);
                            for (ptrx=ptr,i=0;i<iROI.nChannels;i++,ptrx++) {
                                *ptrx=pcolor[i];
                            }
                            acc++;
                        }	    
                    }
                }
            }
        }
    }

    return acc;
}

#endif // CAM_INLINED

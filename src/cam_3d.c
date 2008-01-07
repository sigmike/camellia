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

#include "camellia.h"
#include "camellia_internals.h"
#include <math.h>

void camProject(const double extr[4][4], const double fc[2], const double cc[2], double x, double y, double z, int *xp, int *yp)
{
    // Refer to http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
    int i;
    double Xc[3],Xn[2];
    for (i=0;i<3;i++) {
        Xc[i]=extr[i][0]*x+extr[i][1]*y+extr[i][2]*z+extr[i][3];
    }
    Xn[0]=Xc[0]/Xc[2];
    Xn[1]=Xc[1]/Xc[2];
    *xp=(int)(fc[0]*Xn[0]+cc[0]);
    *yp=(int)(fc[1]*Xn[1]+cc[1]);
}

void camBackproject(const double extr[4][4], const double fc[2], const double cc[2], int xp, int yp, double z, double *x, double *y)
{
    *x = (fc[0] * extr[0][1] * cc[1] * extr[2][2] * z + fc[0] * extr[0][1] * fc[1] * extr[1][2] * z + fc[0] * extr[0][1] * cc[1] * extr[2][3] + fc[0] * extr[0][1] * fc[1] * extr[1][3] - fc[0] * extr[0][2] * z * fc[1] * extr[1][1] - fc[0] * extr[0][2] * z * cc[1] * extr[2][1] - fc[0] * extr[0][1] * yp * extr[2][2] * z - fc[0] * extr[0][1] * yp * extr[2][3] + fc[0] * extr[0][3] * yp * extr[2][1] - fc[0] * extr[0][3] * fc[1] * extr[1][1] - fc[0] * extr[0][3] * cc[1] * extr[2][1] + fc[0] * extr[0][2] * z * yp * extr[2][1] + cc[0] * extr[2][1] * fc[1] * extr[1][2] * z + cc[0] * extr[2][1] * fc[1] * extr[1][3] - xp * extr[2][1] * fc[1] * extr[1][2] * z - xp * extr[2][1] * fc[1] * extr[1][3] + xp * extr[2][2] * z * fc[1] * extr[1][1] + xp * extr[2][3] * fc[1] * extr[1][1] - cc[0] * extr[2][2] * z * fc[1] * extr[1][1] - cc[0] * extr[2][3] * fc[1] * extr[1][1]) / (-xp * extr[2][0] * fc[1] * extr[1][1] - fc[0] * extr[0][0] * yp * extr[2][1] + fc[0] * extr[0][0] * fc[1] * extr[1][1] + fc[0] * extr[0][0] * cc[1] * extr[2][1] + cc[0] * extr[2][0] * fc[1] * extr[1][1] + yp * extr[2][0] * fc[0] * extr[0][1] + fc[1] * extr[1][0] * xp * extr[2][1] - fc[1] * extr[1][0] * fc[0] * extr[0][1] - fc[1] * extr[1][0] * cc[0] * extr[2][1] - cc[1] * extr[2][0] * fc[0] * extr[0][1]);
    *y = -1.0 / (-xp * extr[2][0] * fc[1] * extr[1][1] - fc[0] * extr[0][0] * yp * extr[2][1] + fc[0] * extr[0][0] * fc[1] * extr[1][1] + fc[0] * extr[0][0] * cc[1] * extr[2][1] + cc[0] * extr[2][0] * fc[1] * extr[1][1] + yp * extr[2][0] * fc[0] * extr[0][1] + fc[1] * extr[1][0] * xp * extr[2][1] - fc[1] * extr[1][0] * fc[0] * extr[0][1] - fc[1] * extr[1][0] * cc[0] * extr[2][1] - cc[1] * extr[2][0] * fc[0] * extr[0][1]) * (-xp * extr[2][0] * fc[1] * extr[1][3] - fc[0] * extr[0][0] * yp * extr[2][3] + fc[0] * extr[0][0] * fc[1] * extr[1][3] + fc[0] * extr[0][0] * cc[1] * extr[2][3] + cc[0] * extr[2][0] * fc[1] * extr[1][3] + yp * extr[2][0] * fc[0] * extr[0][3] + fc[1] * extr[1][0] * xp * extr[2][3] - fc[1] * extr[1][0] * fc[0] * extr[0][3] - fc[1] * extr[1][0] * cc[0] * extr[2][3] - cc[1] * extr[2][0] * fc[0] * extr[0][3] + fc[1] * extr[1][0] * xp * extr[2][2] * z - xp * extr[2][0] * fc[1] * extr[1][2] * z - fc[1] * extr[1][0] * fc[0] * extr[0][2] * z - fc[0] * extr[0][0] * yp * extr[2][2] * z - fc[1] * extr[1][0] * cc[0] * extr[2][2] * z + fc[0] * extr[0][0] * fc[1] * extr[1][2] * z - cc[1] * extr[2][0] * fc[0] * extr[0][2] * z + fc[0] * extr[0][0] * cc[1] * extr[2][2] * z + cc[0] * extr[2][0] * fc[1] * extr[1][2] * z + yp * extr[2][0] * fc[0] * extr[0][2] * z);
}

int camUndistortBuildLUT(CamImage *source,
                         const float* intrinsic_matrix,
                         const float* dist_coeffs,
                         CamImage *LUTX, CamImage *LUTY)
{
    int u,v;
    float u0=intrinsic_matrix[2], v0=intrinsic_matrix[5];
    float fx=intrinsic_matrix[0], fy=intrinsic_matrix[4];
    float _fx=1.f/fx, _fy=1.f/fy;
    float k1=dist_coeffs[0], k2=dist_coeffs[1];
    float p1=dist_coeffs[2], p2=dist_coeffs[3];

    int width,height,xOffset,yOffset;
    float y,y2,ky,k2y,_2p1y,_3p1y2,p2y2;
    float x,x2,kx,d,usrc,vsrc;
    int xp,yp;

    short *ptrLUTX,*ptrLUTY;
    unsigned char *cptrLUTX,*cptrLUTY;

    // ROI (Region Of Interest) management
    if (source->roi) {
	width=source->roi->width;
	height=source->roi->height;
        xOffset=source->roi->xOffset;
        yOffset=source->roi->yOffset;
   } else {
	width=source->width;
	height=source->height;
        xOffset=0; yOffset=0;
    }

    // LUT images allocation
    camAllocateImage(LUTX,width,height,CAM_DEPTH_16S);
    camAllocateImage(LUTY,width,height,CAM_DEPTH_16S);
    ptrLUTX=(short*)LUTX->imageData;
    ptrLUTY=(short*)LUTY->imageData;
    cptrLUTX=LUTX->imageData;
    cptrLUTY=LUTY->imageData;

    for (v=yOffset;v<yOffset+height;v++)
    {
        y = (v - v0)*_fy;
        y2 = y*y;
        ky = 1 + (k1 + k2*y2)*y2;
        k2y = 2*k2*y2;
        _2p1y = 2*p1*y;
        _3p1y2 = 3*p1*y2;
        p2y2 = p2*y2;

        for (u=xOffset;u<xOffset+width;u++)
        {
            x = (u - u0)*_fx;
            x2 = x*x;
            kx = (k1 + k2*x2)*x2;
            d = kx + ky + k2y*x2;
            usrc = fx*(x*(d + _2p1y) + p2y2 + (3*p2)*x2) + u0;
            vsrc = fy*(y*(d + (2*p2)*x) + _3p1y2 + p1*x2) + v0;
            xp = (int)(usrc*(1 << 16))+32767;
            yp = (int)(vsrc*(1 << 16))+32767;
            *ptrLUTX++=(xp-(u<<16))>>8;
            *ptrLUTY++=(yp-(v<<16))>>8;
        }

        cptrLUTX+=LUTX->widthStep;
        cptrLUTY+=LUTY->widthStep;
        ptrLUTX=(short*)cptrLUTX;
        ptrLUTY=(short*)cptrLUTY;
    }

    return 1;
}

#define CAM_DOT_POS 20

#define FIXED_INV(x) \
    ((CAM_FIXED_POINT)((((CAM_INT64)1)<<(2*CAM_DOT_POS))/(x)))
#define INT2FIXED(x) \
    ((x)<<CAM_DOT_POS)
#define MUL(x,y) \
    ((CAM_FIXED_POINT)(tmp=(x), ((tmp*(y))>>CAM_DOT_POS)))

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation
#undef CAM_PIXEL
#define CAM_PIXEL unsigned char
#define camUndistort camUndistort8
#define camUndistortFixed camUndistortFixed8
#define camUndistortLUT camUndistortLUT8
#include "cam_3d_code.c"
#undef camUndistort
#undef camUndistortFixed
#undef camUndistortLUT

#undef CAM_PIXEL
#define CAM_PIXEL unsigned short
#define camUndistort camUndistort16
#define camUndistortFixed camUndistortFixed16
#define camUndistortLUT camUndistortLUT16
#include "cam_3d_code.c"
#undef camUndistort
#undef camUndistortFixed
#undef camUndistortLUT

int camUndistort(CamImage *source, CamImage *dest,
                 const float* intrinsic_matrix,
                 const float* dist_coeffs)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camUndistort16(source,dest,intrinsic_matrix,dist_coeffs);
    } else {
	return camUndistort8(source,dest,intrinsic_matrix,dist_coeffs);
    }
}

int camUndistortFixed(CamImage *source, CamImage *dest,
                      const CAM_FIXED_POINT* intrinsic_matrix,
                      const CAM_FIXED_POINT* dist_coeffs)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camUndistortFixed16(source,dest,intrinsic_matrix,dist_coeffs);
    } else {
	return camUndistortFixed8(source,dest,intrinsic_matrix,dist_coeffs);
    }
}

int camUndistortLUT(CamImage *source, CamImage *dest,
                    CamImage *LUTX, CamImage *LUTY)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camUndistortLUT16(source,dest,LUTX,LUTY);
    } else {
	return camUndistortLUT8(source,dest,LUTX,LUTY);
    }
}

#else
#include "cam_3d_code.c"
#endif





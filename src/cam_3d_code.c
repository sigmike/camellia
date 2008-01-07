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

int camUndistort(CamImage *source, CamImage *dest,
                 const float* intrinsic_matrix,
                 const float* dist_coeffs)
{
    int u,v,i;
    float u0=intrinsic_matrix[2], v0=intrinsic_matrix[5];
    float fx=intrinsic_matrix[0], fy=intrinsic_matrix[4];
    float _fx=1.f/fx, _fy=1.f/fy;
    float k1=dist_coeffs[0], k2=dist_coeffs[1];
    float p1=dist_coeffs[2], p2=dist_coeffs[3];

    int width,height,xOffset,yOffset,nChannels;
    float y,y2,ky,k2y,_2p1y,_3p1y2,p2y2;
    float x,x2,kx,d,usrc,vsrc;
    int xp,yp,xx,yy,I0,I1,S0,S1,S2,S3,result;
    CAM_PIXEL *dstptr,*srcptr,*cpdstptr;

    CAM_CHECK_ARGS(camUndistort,source->nChannels==dest->nChannels);
    CAM_CHECK_ARGS(camUndistort,source->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camUndistort,dest->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camUndistort,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camUndistort,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camUndistort,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camUndistort,(dest->depth&CAM_DEPTH_MASK)>=8);

    // ROI (Region Of Interest) management
    nChannels=source->nChannels;
    if (dest->roi) {
	dstptr=(CAM_PIXEL*)(dest->imageData+dest->roi->yOffset*dest->widthStep+dest->roi->xOffset*sizeof(CAM_PIXEL)*nChannels);
	width=dest->roi->width;
	height=dest->roi->height;
    } else {
	dstptr=(CAM_PIXEL*)dest->imageData;
	width=dest->width;
	height=dest->height;
    }
    if (source->roi) {
        CAM_CHECK_ARGS(camUndistort,(source->roi->width==width));
        CAM_CHECK_ARGS(camUndistort,(source->roi->height==height));
        xOffset=source->roi->xOffset;
        yOffset=source->roi->yOffset;
    } else {
        xOffset=0; yOffset=0;
    }

    for (v=yOffset;v<yOffset+height;v++)
    {
        cpdstptr=dstptr;

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

            xx=(xp-32767)>>16; // Upper left pixel
            yy=(yp-32767)>>16;
            if ((xx>=0)&&(xx<source->width-1)&&(yy>=0)&&(yy<source->height-1)) {
                srcptr=(CAM_PIXEL*)(source->imageData+yy*source->widthStep+xx*sizeof(CAM_PIXEL)*nChannels);
                xx=(xx<<16)+32767;
                yy=(yy<<16)+32767; // Center of the upper left pixel
                for (i=0;i<nChannels;i++) {                   
                    S0=*srcptr;
                    S1=*(srcptr+nChannels);
                    S2=*(srcptr+source->widthStep);
                    S3=*(srcptr+source->widthStep+nChannels);
                    I0=(S0<<8)+(((xp-xx)*(S1-S0))>>8); // Horizontal linear interpolation
                    I1=(S2<<8)+(((xp-xx)*(S3-S2))>>8); // 8 bits fixed point
                    result=(I0+(((yp-yy)*(I1-I0))>>16)+128)>>8; // Vertical linear interpolation
                    // Write the result to the destination image
                    *dstptr++=(CAM_PIXEL)result;
                    srcptr++;
                }
            } else {
                for (i=0;i<nChannels;i++) {                   
                    *dstptr++=0;
                }
            }
        }

        // Move the destination pointer
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);
    }

    return 1;
}

int camUndistortFixed(CamImage *source, CamImage *dest,
                      const CAM_FIXED_POINT* intrinsic_matrix,
                      const CAM_FIXED_POINT* dist_coeffs)
{
    int u,v,i;
    CAM_INT64 tmp;
    CAM_FIXED_POINT u0=intrinsic_matrix[2], v0=intrinsic_matrix[5];
    CAM_FIXED_POINT fx=intrinsic_matrix[0], fy=intrinsic_matrix[4];
    CAM_FIXED_POINT _fx=FIXED_INV(fx), _fy=FIXED_INV(fy);
    CAM_FIXED_POINT k1=dist_coeffs[0], k2=dist_coeffs[1];
    CAM_FIXED_POINT p1=dist_coeffs[2], p2=dist_coeffs[3];

    int width,height,xOffset,yOffset,nChannels;
    CAM_FIXED_POINT y,y2,ky,k2y,_2p1y,_3p1y2,p2y2;
    CAM_FIXED_POINT x,x2,kx,d;
    CAM_FIXED_POINT t1,t2,t3,t4,t5,usrc,vsrc;

    int xp,yp,xx,yy,I0,I1,S0,S1,S2,S3,result;
    CAM_PIXEL *dstptr,*srcptr,*cpdstptr;

    CAM_CHECK_ARGS(camUndistort,source->nChannels==dest->nChannels);
    CAM_CHECK_ARGS(camUndistort,source->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camUndistort,dest->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camUndistort,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camUndistort,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camUndistort,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camUndistort,(dest->depth&CAM_DEPTH_MASK)>=8);

    // ROI (Region Of Interest) management
    nChannels=source->nChannels;
    if (dest->roi) {
	dstptr=(CAM_PIXEL*)(dest->imageData+dest->roi->yOffset*dest->widthStep+dest->roi->xOffset*sizeof(CAM_PIXEL));
	width=dest->roi->width;
	height=dest->roi->height;
    } else {
	dstptr=(CAM_PIXEL*)dest->imageData;
	width=dest->width;
	height=dest->height;
    }
    if (source->roi) {
        CAM_CHECK_ARGS(camUndistort,(source->roi->width==width));
        CAM_CHECK_ARGS(camUndistort,(source->roi->height==height));
        xOffset=source->roi->xOffset;
        yOffset=source->roi->yOffset;
    } else {
        xOffset=0; yOffset=0;
    }

    for (v=yOffset;v<yOffset+height;v++)
    {
        cpdstptr=dstptr;

        y = MUL(_fy,(INT2FIXED(v) - v0));
        y2 = MUL(y,y);
        ky = INT2FIXED(1) + MUL((k1 + MUL(k2,y2)),y2);
        k2y = MUL(MUL(INT2FIXED(2),k2),y2);
        _2p1y = MUL(MUL(INT2FIXED(2),p1),y);
        _3p1y2 = MUL(MUL(INT2FIXED(3),p1),y2);
        p2y2 = MUL(p2,y2);

        for (u=xOffset;u<xOffset+width;u++)
        {
            x = MUL((INT2FIXED(u) - u0),_fx);
            x2 = MUL(x,x);
            kx = MUL((k1 + MUL(k2,x2)),x2);
            d = kx + ky + MUL(k2y,x2);

            t1 = MUL(MUL(INT2FIXED(3),p2),x2);
            t2 = MUL(x,(d + _2p1y));
            usrc =  MUL(fx, t2 + p2y2 + t1) + u0;
            t3 = MUL(MUL(INT2FIXED(2),p2),x);
            t4 = MUL(y,d + t3);
            t5 = MUL(p1,x2);
            vsrc = MUL(fy, t4 + _3p1y2 + t5) + v0;            

            xp = (usrc>>(CAM_DOT_POS-16))+32767;
            yp = (vsrc>>(CAM_DOT_POS-16))+32767;
            xx=(xp-32767)>>16; // Upper left pixel
            yy=(yp-32767)>>16;
            if ((xx>=0)&&(xx<source->width-1)&&(yy>=0)&&(yy<source->height-1)) {
                srcptr=(CAM_PIXEL*)(source->imageData+yy*source->widthStep+xx*sizeof(CAM_PIXEL)*nChannels);
                xx=(xx<<16)+32767;
                yy=(yy<<16)+32767; // Center of the upper left pixel
                for (i=0;i<nChannels;i++) {                   
                    S0=*srcptr;
                    S1=*(srcptr+nChannels);
                    S2=*(srcptr+source->widthStep);
                    S3=*(srcptr+source->widthStep+nChannels);
                    I0=(S0<<8)+(((xp-xx)*(S1-S0))>>8); // Horizontal linear interpolation
                    I1=(S2<<8)+(((xp-xx)*(S3-S2))>>8); // 8 bits fixed point
                    result=(I0+(((yp-yy)*(I1-I0))>>16)+128)>>8; // Vertical linear interpolation
                    // Write the result to the destination image
                    *dstptr++=(CAM_PIXEL)result;
                    srcptr++;
                }
            } else {
                for (i=0;i<nChannels;i++) {                   
                    *dstptr++=0;
                }
            }
        }

        // Move the destination pointer
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);
    }

    return 1;
}


int camUndistortLUT(CamImage *source, CamImage *dest,
                    CamImage *LUTX, CamImage *LUTY)
{
    int u,v,i;
    int width,height,xOffset,yOffset,nChannels;
    int xp,yp,xx,yy,I0,I1,S0,S1,S2,S3,result;
    CAM_PIXEL *dstptr,*srcptr,*cpdstptr;
    short *ptrLUTX,*ptrLUTY;
    unsigned char *cptrLUTX,*cptrLUTY;

    CAM_CHECK_ARGS(camUndistortLUT,source->nChannels==dest->nChannels);
    CAM_CHECK_ARGS(camUndistortLUT,source->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camUndistortLUT,dest->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camUndistortLUT,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camUndistortLUT,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camUndistortLUT,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camUndistortLUT,(dest->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camUndistortLUT,LUTX->width==LUTY->width);
    CAM_CHECK_ARGS(camUndistortLUT,LUTX->height==LUTY->height);

    // ROI (Region Of Interest) management
    nChannels=source->nChannels;
    if (dest->roi) {
	dstptr=(CAM_PIXEL*)(dest->imageData+dest->roi->yOffset*dest->widthStep+dest->roi->xOffset*sizeof(CAM_PIXEL)*nChannels);
	width=dest->roi->width;
	height=dest->roi->height;
    } else {
	dstptr=(CAM_PIXEL*)dest->imageData;
	width=dest->width;
	height=dest->height;
    }
    if (source->roi) {
        CAM_CHECK_ARGS(camUndistortLUT,(source->roi->width==width));
        CAM_CHECK_ARGS(camUndistortLUT,(source->roi->height==height));
        xOffset=source->roi->xOffset;
        yOffset=source->roi->yOffset;
    } else {
        xOffset=0; yOffset=0;
    }

    if ((LUTX->width==width)&&(LUTX->height==height)) {
        ptrLUTX=(short*)LUTX->imageData;
        ptrLUTY=(short*)LUTY->imageData;
        cptrLUTX=LUTX->imageData;
        cptrLUTY=LUTY->imageData;
    } else {
        CAM_CHECK_ARGS(camUndistortLUT,source->width==LUTX->width);
        CAM_CHECK_ARGS(camUndistortLUT,source->height==LUTX->height);
        cptrLUTX=(LUTX->imageData+yOffset*LUTX->widthStep+xOffset*sizeof(short));
        cptrLUTY=(LUTY->imageData+yOffset*LUTX->widthStep+xOffset*sizeof(short));
        ptrLUTX=(short*)cptrLUTX;
        ptrLUTY=(short*)cptrLUTY;
    }
    for (v=yOffset;v<yOffset+height;v++)
    {
        cpdstptr=dstptr;
        for (u=xOffset;u<xOffset+width;u++,ptrLUTX++,ptrLUTY++)
        {
            xp=(u<<16)+((*ptrLUTX)<<8);
            yp=(v<<16)+((*ptrLUTY)<<8);
            xx=(xp-32767)>>16; // Upper left pixel
            yy=(yp-32767)>>16;
            if ((xx>=0)&&(xx<source->width-1)&&(yy>=0)&&(yy<source->height-1)) {
                srcptr=(CAM_PIXEL*)(source->imageData+yy*source->widthStep+xx*sizeof(CAM_PIXEL)*nChannels);
                xx=(xx<<16)+32767;
                yy=(yy<<16)+32767; // Center of the upper left pixel
                for (i=0;i<nChannels;i++) {                   
                    S0=*srcptr;
                    S1=*(srcptr+nChannels);
                    S2=*(srcptr+source->widthStep);
                    S3=*(srcptr+source->widthStep+nChannels);
                    I0=(S0<<8)+(((xp-xx)*(S1-S0))>>8); // Horizontal linear interpolation
                    I1=(S2<<8)+(((xp-xx)*(S3-S2))>>8); // 8 bits fixed point
                    result=(I0+(((yp-yy)*(I1-I0))>>16)+128)>>8; // Vertical linear interpolation
                    // Write the result to the destination image
                    *dstptr++=(CAM_PIXEL)result;
                    srcptr++;
                }
            } else {
                for (i=0;i<nChannels;i++) {                   
                    *dstptr++=0;
                }
            }
        }

        // Move the destination pointer
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);

        // Move the LUT pointers
        cptrLUTX+=LUTX->widthStep;
        cptrLUTY+=LUTY->widthStep;
        ptrLUTX=(short*)cptrLUTX;
        ptrLUTY=(short*)cptrLUTY;
    }

    return 1;
}




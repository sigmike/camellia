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

#include <stdlib.h>
#include "camellia.h"

/* This is the original resampling algorithm by volberg
 */
#ifdef CAM_VOLBERG_ORIGINAL
void volbergfvd(double f[], int in[] , int out[] , int inlen, int outlen)
{
#else
void camVolbergFwdScanline(CAM_PIXEL *in, int inl, CAM_PIXEL *out, int outlen, double f[])
{
    int inlen=inl-1;
#endif
    int u,	/* index into input image */
	x,	/* index into output image */
	ul,	/* index of the leftmost input pixel */
	ur,	/* index of the rightmost input pixel */
	ix0,	/* index of the left side of interval */
	ix1;	/* index of the rightside of interval */
    double x0,	/* index of the left side of interval */
	x1,	/* index of the rightside of interval */
	dI;	/* intensity increment over interval */

    /* clear output/accumulator array */
    for (x=0; x<outlen; x++) out[x] = 0;
    
    /* check if no input scanline pixel projects to [0,outlen] range */
    if (f[inlen] < 0 || f[0] > outlen) return; /* 100% clipping */
    
    /* init ul, the left-extent of the input scanline */
    for(u=0; f[u]<0; u++);  /* advance u */
    ul = u;		    /* init ul */
    
    /* process first interval: clip left straddle */
    if(u > 0) {
	u = ul - 1;
	x0 = f[u];
	x1 = f[u+1];
	ix0 = (int)x0 - 1;
	ix1 = (int)x1;

	/* central interval; will be clipped for x<0 */
	dI = (in[u+1] - in[u]) / (x1 - x0);
	for (x=0; x<ix1 && x<outlen; x++)
	    out[x] = in[u] + (CAM_PIXEL)(dI*(x-x0));

	/* right straddle */
	if (ix1!=x1 && ix1<outlen)
	    out[ix1] = (in[u] + (CAM_PIXEL)(dI*(ix1-x0))) * (CAM_PIXEL)(x1-ix1);
    }

    /* init, ur the right-extent of the input scanline */
    for(u=inlen; f[u]>outlen; u--);	/* advance u */
    ur = (u == inlen) ? inlen-1 : u;	/* init ur */
    
    /* check if only one input pixel covers scanline */
    if(u == ul) return;
    
    /* process last interval: clip right straddle */
    if(u < inlen) {
	x0 = f[u];    /* real-valued left index */
	x1 = f[u+1];  /* real-valued right index */
	ix0 = (int)x0;    /* int-valued left index */
	ix1 = (int)x1;  /* int-valued right index */
	/* left straddle */
	out[ix0] += in[u] * (CAM_PIXEL)(ix0-(int)x0+1);

	/* central interval: will be clipped for x>=outlen */
	dI = (in[u+1] - in[u]) / (x1 - x0);
	for(x=ix0+1; x<ix1 && x<outlen; x++)
	    out[x] = in[u] + (CAM_PIXEL)(dI*(x-x0));
    }

    /* main loop */
    for(u=ul; u<=ur; u++) {
	x0 = f[u];    /* real-valued left index */
	x1 = f[u+1];  /* real-valued right index */
	ix0 = (int)x0;    /* int-valued left index */
	ix1 = (int)x1;  /* int-valued right index */
	/* check if interval is embedded in one output pixel */
	if(ix0 == ix1) {
	    out[ix1] += in[u] * (CAM_PIXEL)(x1-x0);    /* accumulate pixel */
	    continue;			    /* next input pixel */
	}

	/* left straddle */
	out[ix0] += in[u] * (CAM_PIXEL)(ix0-(int)x0+1); /* add input fragment */
	
	/* central interval */
	dI = (in[u+1] - in[u]) / (x1 - x0); /* for linear intrp */
	for(x=ix0+1; x<ix1; x++)	/* visit all pixels */
	    out[x] = in[u] + (CAM_PIXEL)(dI*(x-x0)); /* init output pixel */
	
	/* right straddle */
	if (x1 != ix1)	    
	    out[ix1] = (in[u] + (CAM_PIXEL)(dI*(ix1-x0)) * (CAM_PIXEL)(x1-ix1));
    }
}

/* Warping
 * using Volberg's algorithm
 */

void camVolbergFwd(CamImage *source, CamImage *dest, CamVolbergFwdParams *params)
{
    int x,y;
    CamImage intermediate;
    CAM_PIXEL *scanlinesrc,*scanlinedst,*intptr,*dstptr;
    double *mapping;

    camAllocateImage(&intermediate,dest->width,source->height,source->depth);
    
    // First resampling : horizontal
    mapping=(double*)malloc((source->width+1)*sizeof(double));
    for (y=0;y<intermediate.height;y++) {
	for (x=0;x<=source->width;x++) {
	    params->hfwd(x,y,&mapping[x]);
	}
	// Call Volberg's algorithm
	camVolbergFwdScanline((CAM_PIXEL*)(source->imageData+y*source->widthStep), source->width,
	   (CAM_PIXEL*)(intermediate.imageData+y*intermediate.widthStep), intermediate.width,
	   mapping);
    }
    free(mapping);

    // Second resampling : vertical
    mapping=(double*)malloc((intermediate.height+1)*sizeof(double));
    scanlinesrc=(CAM_PIXEL*)malloc(intermediate.height*sizeof(CAM_PIXEL));
    scanlinedst=(CAM_PIXEL*)malloc(dest->height*sizeof(CAM_PIXEL));
    for (x=0;x<dest->width;x++) {
	intptr=((CAM_PIXEL*)intermediate.imageData)+x;
	for (y=0;y<intermediate.height;y++) {
	    params->vfwd(x,y,&mapping[y]);
	    scanlinesrc[y]=*intptr;
	    intptr=((CAM_PIXEL*)(((char*)intptr)+intermediate.widthStep));
	}
	params->vfwd(x,y,&mapping[y]);
	// Call Volberg's algorithm
	camVolbergFwdScanline(scanlinesrc, intermediate.height,
	   scanlinedst, dest->height,
	   mapping);	
	dstptr=((CAM_PIXEL*)dest->imageData)+x;
	for (y=0;y<dest->height;y++) {
	    *dstptr=scanlinedst[y];
	    dstptr=((CAM_PIXEL*)(((char*)dstptr)+dest->widthStep));
	}
    }
    free(scanlinesrc);
    free(scanlinedst);
    free(mapping);

    camDeallocateImage(&intermediate);
}

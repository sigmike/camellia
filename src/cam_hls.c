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

#include <stdio.h>

/* RGB2HLS color conversion
 * Authors : Charles et Pierre Wolfers
 *
 * Ref. for color representation : http://www.ipsi.fraunhofer.de/Kueppersfarbe/fr/theorie33.html
 * Sketch of implemented algorithm :

#define SQR(V)   (V * V)

fonction rgb2hls(double r, double g, double b)
{
  double Reel, Imaginaire, h, l, s;

  Reel = r - (g + b) / 2;
  Imaginaire = (g - b) * sqrt(3) / 2;
  h = argument(Imaginaire, Reel); //Voir fonction phasearg
  l = (r + g + b) / sqrt(3);
  s = racine(SQR(r) + SQR(g) + SQR(b) - SQR(l));
} 
*/

// Fast rgb2hls conversion

#include "cam_hls_hypot_code.c"
#include "cam_hls_phasearg_code.c"
#include "cam_hls_sqrt_code.c"

/* The parameter is put back into [0;360[ */
static unsigned short phasearg(long ip, long rp, long md)
{
    if (md == 0)
	return 0;
    if (abs(ip) > 255)
    {
	ip = (ip > 0) ? 255 : -255;
	fprintf(stderr, "TRUNC IM !!\n");
	fflush(stderr);
    }
    if (abs(rp) > 255)
    { 
	rp = (rp > 0) ? 255 : -255;
	fprintf(stderr, "TRUNC RE !!\n");
	fflush(stderr);
    }
    if (ip > 0)
	if (rp > 0)
	    return phasearg_tab[ip][rp];
	else
	    return 180 - phasearg_tab[ip][-rp];
    else
	if (rp > 0)
	    return 360 - phasearg_tab[-ip][rp];
	else
	    return 180 + phasearg_tab[-ip][-rp];
}

static unsigned short hypotw(long re, long im)
{
    long abs_re = abs(re);
    long abs_im = abs(im);

    if (abs_re > 255)
    {
	abs_re = 255;
	fprintf(stderr, "TRUNC RE !!\n");
	fflush(stderr);
    }
    if (abs_im > 255)
    { 
	abs_im = 255;
	fprintf(stderr, "TRUNC IM !!\n");
	fflush(stderr);
    }
    return hypot_tab[abs_re][abs_im];
}

static unsigned short cam_sqrt(long carre)
{
    if (carre < 0)
    {
	fprintf(stderr, "sqrt(-x) !!\n");
	fflush(stderr);
	return 0;
    }
    if (carre >= SMPL_SQRT)
    { 
	fprintf(stderr, "TRUNC sqrt(%li) !!\n", carre);
	fflush(stderr);
	return sqrt_tab[SMPL_SQRT - 1];
    }
    return sqrt_tab[carre];
}

int camRGB2HLS(CamImage *source, CamImage *dest)
{
    int x,y,width,height;
    int re,im,md;
    int pr,pb,pg;
    unsigned char *r, *g, *b;
    unsigned char *rx,*gx,*bx;
    unsigned short *h, *l, *s;
    unsigned short *hx,*lx,*sx;
    CamInternalROIPolicyStruct iROI;
    DECLARE_MASK_MANAGEMENT;  

    CAM_CHECK_ARGS2(camRGB2HLS,source->imageData!=NULL,"source image is not allocated");
    if (dest->imageData==NULL) {
        // Automatic allocation
        camAllocateHLSImage(dest,source->width,source->height);
    }
    CAM_CHECK(camRGB2HLS,camInternalROIPolicy(source, dest, &iROI, CAM_MASK_SUPPORT | CAM_IGNORE_COI_MISMATCH));
    CAM_CHECK_ARGS(camRGB2HLS, source->nChannels>=3);
    CAM_CHECK_ARGS(camRGB2HLS, dest->nChannels==3);
    CAM_CHECK_ARGS(camRGB2HLS, source->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camRGB2HLS, dest->dataOrder==CAM_DATA_ORDER_PLANE);
    CAM_CHECK_ARGS(camRGB2HLS, source->depth==CAM_DEPTH_8U);
    CAM_CHECK_ARGS(camRGB2HLS, dest->depth > CAM_DEPTH_8U);
    CAM_CHECK_ARGS(camRGB2HLS, (*((int*)dest->colorModel)==*((int*)"HLS")));
    CAM_CHECK_ARGS(camRGB2HLS, (*((int*)source->colorModel)==*((int*)"RGB"))||(*((int*)source->colorModel)==*((int*)"RGBA")));
    
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
    
    hx=(unsigned short*)iROI.dstptr;
    lx=hx+dest->imageSize/6;
    sx=hx+dest->imageSize/3;
    for (y=0;y<height;y++){
        r=rx;b=bx;g=gx;
        h=hx;l=lx;s=sx;
        
	BEGIN_MASK_MANAGEMENT(
	    r=rx+startx*iROI.srcinc;
            b=bx+startx*iROI.srcinc;
            g=gx+startx*iROI.srcinc;
            h=hx+startx;
            l=lx+startx;
            s=sx+startx;
        )                                        

            for (x = startx; x < endx; x++, h++, l++, s++) {
		pr=*r; pb=*b; pg=*g;
		
		re = pr - ((pg + pb) >> 1);
		im = ((pg - pb) * 14529495) >> 24; 
		md = hypotw(re, im);
		*h = phasearg(im, re, md);
		*l = pr + pg + pb;
		//s = cam_sqrt(pr*pr + pg*pg + pb*pb - (l*l) / 3);
		*s = cam_sqrt(pr*pr + pg*pg + pb*pb - ((((*l) * (*l)) * 10) >> 5));

		r+=iROI.srcinc;
                g+=iROI.srcinc;
                b+=iROI.srcinc;
            }
	END_MASK_MANAGEMENT

        rx+=source->widthStep;
        gx+=source->widthStep;
        bx+=source->widthStep;
        hx+=dest->widthStep >> 1;
        lx+=dest->widthStep >> 1;
        sx+=dest->widthStep >> 1;
    }
    camInternalROIPolicyExit(&iROI);
    return 1;
}

/* HLS to pseudo colors conversion

#define SAT_MIN_RED	25
#define SAT_MIN_GREEN	30
#define SAT_MIN_BLUE	60

#define SAT_MIN_0	SAT_MIN_RED
#define SAT_MIN_30	((SAT_MIN_0 + SAT_MIN_60) >> 1)
#define SAT_MIN_60	((SAT_MIN_RED + SAT_MIN_GREEN) >> 1)
#define SAT_MIN_120	SAT_MIN_GREEN
#define SAT_MIN_180	((SAT_MIN_GREEN + SAT_MIN_BLUE) >> 1)
#define SAT_MIN_240	SAT_MIN_BLUE
#define SAT_MIN_300	((SAT_MIN_BLUE + SAT_MIN_RED) >> 1)

#define SAT_MIN		SAT_MIN_RED

{
    if (s >= SAT_MIN)
    {
	if (((h < 20) || (h >= 330)) && (s >= SAT_MIN_0))
	{
	    *d = red; //0 deg
	    continue;
	}
	if ((h >= 20) && (h < 40) && (l < 383) && (s >= SAT_MIN_30))
	{
	    *d = brown; //30 deg
	    continue;
	}
	if ((h >= 20) && (h < 45) && (l >= 383) && (s >= SAT_MIN_30))
	{
	    *d = orange; //30 deg (+ brighter than brown)
	    continue;
	}
	if ((h >= 45) && (h < 65) && (l >= 383) && (s >= SAT_MIN_60))
	{
	    *d = yellow; //60 deg
	    continue;
	}
	if ((h >= 90) && (h < 150) && (s >= SAT_MIN_120))
	{
	    *d = green; //120 deg
	    continue;
	}
	if ((h >= 150) && (h < 210) && (s >= SAT_MIN_180))
	{
	    *d = cyan; //180 deg
	    continue;
	}
	if ((h >= 210) && (h <= 270) && (l < 383) && (s >= SAT_MIN_240))
	{
	    *d = blue; //240 deg
	    continue;
	}
	if ((h >= 210) && (h <= 270) && (l >= 383) && (s >= SAT_MIN_240))
	{
	    *d = sky_blue; //240 deg
	    continue;
	}
	if ((h >= 270) && (h < 330) && (l < 574) && (s >= SAT_MIN_300))
	{
	    *d = magenta; //300 deg
	    continue;
	}
	if ((h >= 270) && (h < 330) && (l >= 574) && (s >= SAT_MIN_300))
	{
	    *d = pink; //300 deg (+ brighter than magenta)
	    continue;
	}
    }
    if (l >= 383)
    {
	*d = white;
	continue;
    }
    *d = black;
} 
*/

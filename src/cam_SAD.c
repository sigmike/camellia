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
#ifdef __INTEL_COMPILER
#include <mmintrin.h>
#include <xmmintrin.h>
#endif

int camSAD8x8(CamImage *image1, CamImage *image2, int bleft, int btop, int dx, int dy)
{
#define BLOCK_SIZE 8
    int y	    = 0;		/* Line Index				*/
    int x	    = 0;		/* Column Index				*/
    
    int sad	    = 0;		/* The AD Accumulator */
    int d           = 0;                /* Pixel difference */
    int num_computations= 0;		/* The number of participating pixels */
    
    int truncated   = 0;		/* Indicates that the current block is out of borders */
    
    int bx,ex,by,ey;	                /* Begin and end for x and y counters */
      
    CAM_PIXEL *im1ptr, *im1ptr2, *im2ptr, *im2ptr2; /* Pointers to source and destination pixels */
    
#ifdef __INTEL_COMPILER
    __m64 set1,set2,res;
#endif
    
    // Compute bx, ex, by and ey
    by = (btop+dy >= 0) ? 0 : -(btop+dy);
    ey = BLOCK_SIZE + ((btop+BLOCK_SIZE+dy < image1->height) ? 0 : image1->height-(btop+BLOCK_SIZE+dy));
    if (ey<by) ey=by;
    
    if (bleft+dx < 0) {
        bx = -(bleft+dx);
        truncated = 1;
    } else bx=0;
    
    if (bleft+BLOCK_SIZE+dx >= image1->width) {
        ex = BLOCK_SIZE+image1->width-(bleft+BLOCK_SIZE+dx);
        truncated = 1;
    } else ex=BLOCK_SIZE;
    
    // Prepare the pointers
    im1ptr2 = ((CAM_PIXEL*)(image1->imageData + (btop+by)*image1->widthStep)) + bleft + bx;
    im2ptr2 = ((CAM_PIXEL*)(image2->imageData + (btop+by+dy)*image2->widthStep)) + bleft + bx + dx;
    
#ifdef __INTEL_COMPILER
    if (truncated) {
#endif
        // Iterate over all the pixels
        for(y = by; y < ey; y++)
        {
            im2ptr = im2ptr2;
            im1ptr = im1ptr2;
            for(x = bx; x< ex; x++)
            {
                // Update Sad
                d = (int)*(im1ptr++) - (int)*(im2ptr++); 
                sad += (d<0)?-d:d;
            }
            im1ptr2 = (CAM_PIXEL*)((char*)im1ptr2 + image1->widthStep);
            im2ptr2 = (CAM_PIXEL*)((char*)im2ptr2 + image2->widthStep);
        }
        num_computations = (ey-by)*(ex-bx);
#ifdef __INTEL_COMPILER
    } else {
        // Iterate over all the pixels
        for(y = by; y < ey; y++)
        {
            im2ptr = im2ptr2;
            im1ptr = im1ptr2;
            set1=_mm_set_pi8(
                *(im2ptr++), *(im2ptr++), *(im2ptr++), *(im2ptr++),
                *(im2ptr++), *(im2ptr++), *(im2ptr++), *(im2ptr++)
                );
            set2=_mm_set_pi8(
                *(im1ptr++), *(im1ptr++), *(im1ptr++), *(im1ptr++),
                *(im1ptr++), *(im1ptr++), *(im1ptr++), *(im1ptr++)
                );
            res=_mm_sad_pu8(set1,set2);
            sad+=_m_pextrw(res,0);
            im1ptr2 = (CAM_PIXEL*)((char*)im1ptr2 + image1->widthStep);
            im2ptr2 = (CAM_PIXEL*)((char*)im2ptr2 + image2->widthStep);
        }	    
        num_computations = (ey-by)<<3;
    }
    _mm_empty();
#endif
    
    if (num_computations==8*8) {
        return sad;
    } else if (num_computations) {
        return (sad<<6)/num_computations;
    }
    return 256*8*8;
}

int camSAD16x16(CamImage *image1, CamImage *image2, int bleft, int btop, int dx, int dy)
{
#undef BLOCK_SIZE
#define BLOCK_SIZE 16
    int y	    = 0;		/* Line Index				*/
    int x	    = 0;		/* Column Index				*/
    
    int sad	    = 0;		/* The AD Accumulator */
    int d           = 0;                /* Pixel difference */
    int num_computations= 0;		/* The number of participating pixels */
    
    int truncated   = 0;		/* Indicates that the current block is out of borders */
    
    int bx,ex,by,ey;	                /* Begin and end for x and y counters */
    
    CAM_PIXEL *im1ptr, *im1ptr2, *im2ptr, *im2ptr2; /* Pointers to source and destination pixels */
    
#ifdef __INTEL_COMPILER
    __m64 set1,set2,res;
#endif
    
    // Compute bx, ex, by and ey
    by = (btop+dy >= 0) ? 0 : -(btop+dy);
    ey = BLOCK_SIZE + ((btop+BLOCK_SIZE+dy < image1->height) ? 0 : image1->height-(btop+BLOCK_SIZE+dy));
    if (ey<by) ey=by;
    
    if (bleft+dx < 0) {
        bx = -(bleft+dx);
        truncated = 1;
    } else bx = 0;
    
    if (bleft+BLOCK_SIZE+dx >= image1->width) {
        ex = BLOCK_SIZE+image1->width-(bleft+BLOCK_SIZE+dx);
        truncated = 1;
    } else ex = BLOCK_SIZE;
    
    // Prepare the pointers
    im1ptr2 = ((CAM_PIXEL*)(image1->imageData + (btop+by)*image1->widthStep)) + bleft + bx;
    im2ptr2 = ((CAM_PIXEL*)(image2->imageData + (btop+by+dy)*image2->widthStep)) + bleft + bx + dx;
    
#ifdef __INTEL_COMPILER
    if (truncated) {
#endif
        // Iterate over all the pixels
        for(y = by; y < ey; y++)
        {
            im2ptr = im2ptr2;
            im1ptr = im1ptr2;
            for(x = bx; x < ex; x++)
            {
                // Update Sad
                d = (int)*(im1ptr++) - (int)*(im2ptr++);
                sad += (d<0)?-d:d;
            }
            im1ptr2 = (CAM_PIXEL*)((char*)im1ptr2 + image1->widthStep);
            im2ptr2 = (CAM_PIXEL*)((char*)im2ptr2 + image2->widthStep);
        }
        num_computations = (ey-by)*(ex-bx);
#ifdef __INTEL_COMPILER
    } else {
        // Iterate over all the pixels
        for(y = by; y < ey; y++)
        {
            im2ptr = im2ptr2;
            im1ptr = im1ptr2;
            set1=_mm_set_pi8(
                *(im2ptr++), *(im2ptr++), *(im2ptr++), *(im2ptr++),
                *(im2ptr++), *(im2ptr++), *(im2ptr++), *(im2ptr++)
                );
            set2=_mm_set_pi8(
                *(im1ptr++), *(im1ptr++), *(im1ptr++), *(im1ptr++),
                *(im1ptr++), *(im1ptr++), *(im1ptr++), *(im1ptr++)
                );
            res=_mm_sad_pu8(set1,set2);
            sad+=_m_pextrw(res,0);
            set1=_mm_set_pi8(
                *(im2ptr++), *(im2ptr++), *(im2ptr++), *(im2ptr++),
                *(im2ptr++), *(im2ptr++), *(im2ptr++), *(im2ptr++)
                );
            set2=_mm_set_pi8(
                *(im1ptr++), *(im1ptr++), *(im1ptr++), *(im1ptr++),
                *(im1ptr++), *(im1ptr++), *(im1ptr++), *(im1ptr++)
                );
            res=_mm_sad_pu8(set1,set2);
            sad+=_m_pextrw(res,0);
            im1ptr2 = (CAM_PIXEL*)((char*)im1ptr2 + image1->widthStep);
            im2ptr2 = (CAM_PIXEL*)((char*)im2ptr2 + image2->widthStep);
        }	    
        num_computations = (ey-by)<<4;
    }
    _mm_empty();
#endif
    
    if (num_computations==16*16) {
        return sad;
    } else if (num_computations) {
        return (sad<<8)/num_computations;
    }
    return 256*16*16;
}

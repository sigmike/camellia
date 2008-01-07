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

/* Apply-a-LUT-on-image Kernel
 * C code */
#include "camellia.h"

int camApplyLUT1U(CamImage *source, CamImage *dest, CamLUT *LUT);

int camApplyLUT(CamImage *source, CamImage *dest, CamLUT *LUT)
{
    int x, y;
    int width, height;
    CAM_PIXEL *srcptr, *tmpsrcptr;
    CAM_PIXEL *dstptr, *tmpdstptr;
    CamInternalROIPolicyStruct iROI;
    int acc=0;
    DECLARE_MASK_MANAGEMENT;

    if (dest->depth == 1) {
	return camApplyLUT1U(source,dest,LUT);
    }
    
    CAM_CHECK(camApplyLUT, camInternalROIPolicy(source, dest, &iROI, 1));
    CAM_CHECK_ARGS(camApplyLUT, (source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camApplyLUT, (source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camApplyLUT, (dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camApplyLUT, (dest->depth&CAM_DEPTH_MASK)>=8);

    // ROI (Region Of Interest) management
    width = iROI.srcroi.width;
    height = iROI.srcroi.height;
    srcptr = (CAM_PIXEL*)iROI.srcptr;
    dstptr = (CAM_PIXEL*)iROI.dstptr;

    INIT_MASK_MANAGEMENT;

    for (y=0;y<height;y++) {
       	tmpsrcptr = srcptr;
	tmpdstptr = dstptr;
	BEGIN_MASK_MANAGEMENT(
	    srcptr = tmpsrcptr + startx*iROI.srcinc;
	    dstptr = tmpdstptr + startx*iROI.dstinc;
	)
            for (x = startx; x < endx; x++, srcptr += iROI.srcinc, dstptr += iROI.dstinc) {
		acc += (*dstptr=LUT->t[*srcptr]);
	    }
        END_MASK_MANAGEMENT;
	srcptr = (CAM_PIXEL*)(((char*)tmpsrcptr) + source->widthStep);
	dstptr = (CAM_PIXEL*)(((char*)tmpdstptr) + dest->widthStep);
    }

    camInternalROIPolicyExit(&iROI);
    return acc;    
}

#define FOREACH_PIXEL	    \
for (y=0;y<height;y++) {    \
    bit_offset=sol_offset;  \
    bit_block=*dstptr;	    \
    bit_block=CAM_BIT_BLOCK_SWAP(bit_block); \
    bit_block>>=bit_offset; \
    cpsrcptr=srcptr; cpdstptr=dstptr; \
    for (x=0;x<width;x++,srcptr++)

#define END_FOREACH_PIXEL   \
    srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep); \
    if (bit_offset) {	    \
	eol_block=*dstptr;  \
	eol_block=CAM_BIT_BLOCK_SWAP(eol_block); \
	eol_block&=((	    \
	    ((CAM_BIT_BLOCK)1)<< \
	    (CAM_BIT_BLOCK_SIZE-bit_offset) \
	)-1);		    \
	eol_block|=(bit_block<<(CAM_BIT_BLOCK_SIZE-bit_offset)); \
	eol_block=CAM_BIT_BLOCK_SWAP(eol_block); \
	*dstptr=eol_block;  \
    } \
    dstptr=(CAM_BIT_BLOCK*)(((char*)cpdstptr)+dest->widthStep); \
}

#define STORE_BIT(x)	    \
    bit_block<<=1;	    \
    bit_block|=(x);	    \
    bit_offset++;	    \
    if (bit_offset==CAM_BIT_BLOCK_SIZE) { \
	bit_block=CAM_BIT_BLOCK_SWAP(bit_block); \
	*dstptr=bit_block;  \
	bit_offset=0;	    \
	dstptr++;	    \
    }
	
// Binary images processing
int camApplyLUT1U(CamImage *source, CamImage *dest, CamLUT *LUT)
{
    int x,y;
    int width,height;
    CAM_PIXEL *srcptr,*cpsrcptr;
    CAM_BIT_BLOCK *dstptr,*cpdstptr;
    CAM_BIT_BLOCK bit_block,eol_block;
    int sol_offset,bit_offset;

    CAM_CHECK_ARGS(camApplyLUT,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camApplyLUT,(source->depth&CAM_DEPTH_MASK)>=8);

    // ROI (Region Of Interest) management
    if (source->roi) {
	srcptr=(CAM_PIXEL*)(source->imageData+source->roi->yOffset*source->widthStep+source->roi->xOffset*sizeof(CAM_PIXEL));
	width=source->roi->width;
	height=source->roi->height;
    } else {
	srcptr=(CAM_PIXEL*)source->imageData;
	width=source->width;
	height=source->height;
    }
    if (dest->roi) {
	sol_offset=dest->roi->xOffset%CAM_BIT_BLOCK_SIZE;
	dstptr=(CAM_BIT_BLOCK*)(dest->imageData+dest->roi->yOffset*dest->widthStep)+(dest->roi->xOffset/CAM_BIT_BLOCK_SIZE);
    } else {
	sol_offset=0;
	dstptr=(CAM_BIT_BLOCK*)dest->imageData;
    }

    FOREACH_PIXEL {
	STORE_BIT(LUT->t[*srcptr]);	    
    } END_FOREACH_PIXEL
	
    return 1;
}

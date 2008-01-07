/** \file camellia_internals.h
 *  \brief Camellia Image Processing Library header file
 *  \author Bruno STEUX (ENSMP)
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

#ifndef _CAMELLIA_INTERNALS_H_
#define _CAMELLIA_INTERNALS_H_

#ifdef __cplusplus
extern "C" {
#endif

// Binary images management
#ifdef CAM_64BITS
#ifdef _WIN32
#define CAM_BIT_BLOCK unsigned __int64
#else
#define CAM_BIT_BLOCK unsigned long long int
#endif
#define CAM_BIT_BLOCK_SIZE_SHIFT 6
#ifndef CAM_BIG_ENDIAN
#ifdef _WIN32
#define CAM_BIT_BLOCK_SWAP(x) (	\
    (((x)&0x00000000000000ffi64)<<56)| \
    (((x)&0x000000000000ff00i64)<<40)| \
    (((x)&0x0000000000ff0000i64)<<24)| \
    (((x)&0x00000000ff000000i64)<< 8)| \
    (((x)&0x000000ff00000000i64)>> 8)| \
    (((x)&0x0000ff0000000000i64)>>24)| \
    (((x)&0x00ff000000000000i64)>>40)| \
    (((x)&0xff00000000000000i64)>>56))
#else
#define CAM_BIT_BLOCK_SWAP(x) (	\
    (((x)&0x00000000000000ffull)<<56)| \
    (((x)&0x000000000000ff00ull)<<40)| \
    (((x)&0x0000000000ff0000ull)<<24)| \
    (((x)&0x00000000ff000000ull)<< 8)| \
    (((x)&0x000000ff00000000ull)>> 8)| \
    (((x)&0x0000ff0000000000ull)>>24)| \
    (((x)&0x00ff000000000000ull)>>40)| \
    (((x)&0xff00000000000000ull)>>56))
#endif //_WIN32
#else
#define CAM_BIT_BLOCK_SWAP(x) (x)
#endif
#else
#define CAM_BIT_BLOCK unsigned int
#define CAM_BIT_BLOCK_SIZE_SHIFT 5
#ifndef CAM_BIG_ENDIAN
#define CAM_BIT_BLOCK_SWAP(x) ((((x)&0x000000ff)<<24)|(((x)&0x0000ff00)<<8)|(((x)&0x00ff0000)>>8)|(((x)&0xff000000)>>24))
#else
#define CAM_BIT_BLOCK_SWAP(x) (x)
#endif
#endif
#define CAM_BIT_BLOCK_SIZE (sizeof(CAM_BIT_BLOCK)*8)

// Internal functions
typedef struct {
    unsigned char *srcptr;
    unsigned char *dstptr;
    CamROI srcroi;   // Source ROI after clipping
    CamROI dstroi;   // Dest ROI after clipping
    int srcchoffset; // Channel offset for the source
    int dstchoffset; // Channel offset for the destination image
    int nChannels;   // Number of effective channels (considering ROIs)
    int srcinc;      // Pixel to pixel increment
    int dstinc;      // Pixel to pixel increment
    int srclinc;     // Line to line increment
    int dstlinc;     // Line to line increment
    int srcpinc;     // Plane to plane increment
    int dstpinc;     // Plane to plane increment

    int mask_xOffset;// Offset when mask doesn't fit within image boundaries
    int mask_yOffset;// Offset when mask doesn't fit within image boundaries
    CamRLEImage *mask; // Mask to be applied to the operation
    CamRLEImage auto_mask; // Automatically allocated if provided mask is not RLE
} CamInternalROIPolicyStruct;

#define CAM_MASK_SUPPORT 1
#define CAM_IGNORE_COI_MISMATCH 2
#define CAM_NO_ROI_INTERSECTION 4

int camInternalROIPolicy(CamImage* src, CamImage *dst, CamInternalROIPolicyStruct *res, int options);
void camInternalROIPolicyExit(CamInternalROIPolicyStruct *res); // Call to this function is necessary only when the caller manages image bit-masking

const char *camGetErrorStr();
void camSetErrorStr(const char *s);

// Check arguments
#ifndef CAM_OPTIMIZE
#define CAM_CHECK_ARGS(function, condition) \
if (!(condition)) {camError(#function, "Bad argument : " #condition);return 0;}
#else
#define CAM_CHECK_ARGS(function, condition)
#endif

#ifndef CAM_OPTIMIZE
#define CAM_CHECK_ARGS2(function, condition, text) \
if (!(condition)) {camError(#function, "Bad argument : " #text);return 0;}
#else
#define CAM_CHECK_ARGS2(function, condition, text)
#endif

#ifndef CAM_OPTIMIZE
#define CAM_CHECK(function, condition) \
if (!(condition)) {camError(#function, NULL);return 0;}
#else
#define CAM_CHECK(function, condition)
#endif

// Mask management macros

#define DECLARE_MASK_MANAGEMENT \
    CamRun *run; \
    int startx,endx
 
#define INIT_MASK_MANAGEMENT \
    if (iROI.mask) { \
        run=iROI.mask->runs+1; \
        while (run->line!=iROI.mask_yOffset) run++; \
    } else { \
        run=NULL; \
    }

#define BEGIN_MASK_MANAGEMENT(code) \
    startx=-iROI.mask_xOffset; \
do { \
    if (!iROI.mask) { \
        endx=width; \
    } else { \
        do { \
            while ((run->value==0)&&(run->line==y+iROI.mask_yOffset)) { \
                startx+=run->length; \
                run++; \
            } \
            if (run->line!=y+iROI.mask_yOffset) break; \
            endx=startx+run->length; \
        } while ((endx<=0)&&(run++)); \
        if (run->line!=y+iROI.mask_yOffset) break; \
        if (startx<0) startx=0; \
        if (startx>width) startx=width; \
        if (endx>width) endx=width; \
        code \
    }

#define END_MASK_MANAGEMENT \
    if (iROI.mask) { \
        startx=endx; \
        run++; \
    } \
} while ((run)&&(run->line==y+iROI.mask_yOffset));

#ifdef __cplusplus
}
#endif


#endif


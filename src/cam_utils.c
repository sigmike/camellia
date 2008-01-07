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
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "camellia.h"
#include "camellia_internals.h"

const char *camVersion()
{
    return CAM_VERSION;
}

FILE *camProfilingDataHandle=NULL;

/* Warning : This code is not reentrant. Use only for debugging purpose */
int camLogProfilingData(const char *module, int nbiter) 
{
    if (camProfilingDataHandle == NULL) {
	camProfilingDataHandle = fopen("cam_profiling.txt","wt");
    }
    fprintf(camProfilingDataHandle, "%s\t%d\n", module, nbiter);
    return 1;
}

int camInternalROIPolicy(CamImage* src, CamImage *dest, CamInternalROIPolicyStruct *res, int options)
{
    CamRLEImage *rle_mask;
    int retval=1;

    res->mask_xOffset=0;
    res->mask_yOffset=0;

    if (src->imageData==NULL) {
        camSetErrorStr("source image is not initialized");
        return 0;
    }

    // Initializes dest ROI
    if (dest) {
        // Smart dest allocation
        if (dest->imageData==NULL) {
            dest->roi=NULL;
            if (src->roi) {
                dest->width=src->roi->width;
                dest->height=src->roi->height;
            } else {
                dest->width=src->width;
                dest->height=src->height;
            }
        }

        if (dest->roi) {
            res->dstroi=*dest->roi;
        } else {
            res->dstroi.coi=0;
            res->dstroi.xOffset=0;
            res->dstroi.yOffset=0;
            res->dstroi.width=dest->width;
            res->dstroi.height=dest->height;
        }
    }

    // Set srcroi
    if (src->roi) {
        res->srcroi=*src->roi;
    } else {
        res->srcroi.coi=0;
        res->srcroi.xOffset=0;
        res->srcroi.yOffset=0;
        res->srcroi.width=src->width;
        res->srcroi.height=src->height;
    }

    // Mask management
    res->auto_mask.allocated=0;
    res->auto_mask.runs=NULL;
    if (options & CAM_MASK_SUPPORT) {
        if (src->mask) {
            rle_mask=(CamRLEImage*)src->mask;
            if (rle_mask->nSize==sizeof(CamRLEImage)) {
                // OK. This is a RLE encoded mask. Best case.
                res->mask=rle_mask;
            } else if (rle_mask->nSize==sizeof(CamImage)) {
                // This is a uncompressed image
                // Let's build a RLE image out of it
                camRLEEncodeThreshold((CamImage*)rle_mask,&res->auto_mask,1);
                res->mask=&res->auto_mask;
            } else {
                camSetErrorStr("src mask is set to an unknown type.");
                res->mask=NULL;
                return 0;
            }
            
            // A few checkings
            if (res->mask->width!=res->srcroi.width) {
                camSetErrorStr("Incorrect mask image width. Should match the ROI.");
                retval=0;
            } else if (res->mask->height!=res->srcroi.height) {
                camSetErrorStr("Incorrect mask image height. Should match the ROI.");
                retval=0;
            }
        } else {
            res->mask=NULL;
        }
    } else {
        res->mask=NULL;
    }

    if (src->roi) {
        // Clip ROI if need be
        if (res->srcroi.xOffset<0) {
            res->mask_xOffset=-res->srcroi.xOffset;
	    res->srcroi.width+=res->srcroi.xOffset;
            if ((dest)&&(!(options & CAM_NO_ROI_INTERSECTION))) {
                // Change res->dstroi accordingly
                res->dstroi.xOffset-=res->srcroi.xOffset;
                res->dstroi.width+=res->srcroi.xOffset;
            }
            res->srcroi.xOffset=0;
	} else if (res->srcroi.xOffset >= src->width) {
	    res->srcroi.xOffset = src->width;
	    res->srcroi.width = 0;
            if ((dest)&&(!(options & CAM_NO_ROI_INTERSECTION))) {
                // Change res->dstroi accordingly
                res->dstroi.width=0;;
            }
	}
	if (res->srcroi.yOffset<0) {
            res->mask_yOffset=-res->srcroi.yOffset;
	    res->srcroi.height+=res->srcroi.yOffset;
            if ((dest)&&(!(options & CAM_NO_ROI_INTERSECTION))) {
                // Change res->dstroi accordingly
                res->dstroi.yOffset-=res->srcroi.yOffset;
                res->dstroi.height+=res->srcroi.yOffset;
            }
            res->srcroi.yOffset=0;
	} else if (res->srcroi.yOffset >= src->height) {
	    res->srcroi.yOffset = src->height;
	    res->srcroi.height = 0;
            if ((dest)&&(!(options & CAM_NO_ROI_INTERSECTION))) {
                // Change res->dstroi accordingly
                res->dstroi.height=0;;
            }
	}
	if (res->srcroi.xOffset+res->srcroi.width>src->width) {
	    res->srcroi.width=src->width-res->srcroi.xOffset;
	}
	if (res->srcroi.yOffset+res->srcroi.height>src->height) {
	    res->srcroi.height=src->height-res->srcroi.yOffset;
	}
    }

    if (dest) {
        // Clip dest ROI
        if (res->dstroi.xOffset<0) {
            res->dstroi.width+=res->dstroi.xOffset;
            if (!(options&CAM_NO_ROI_INTERSECTION)) {
                // Change res->srcroi accordingly
                res->srcroi.xOffset-=res->dstroi.xOffset;
                res->srcroi.width+=res->dstroi.xOffset;
                res->mask_xOffset-=res->dstroi.xOffset;
            }
            res->dstroi.xOffset=0;
        }
        if (res->dstroi.yOffset<0) {
            res->dstroi.height+=res->dstroi.yOffset;
            if (!(options&CAM_NO_ROI_INTERSECTION)) {
                // Change res->srcroi accordingly
                res->srcroi.yOffset-=res->dstroi.yOffset;
                res->srcroi.height+=res->dstroi.yOffset;
                res->mask_yOffset-=res->dstroi.yOffset;
            }
            res->dstroi.yOffset=0;
        }
        if (res->dstroi.xOffset+res->dstroi.width>dest->width) {
            res->dstroi.width=dest->width-res->dstroi.xOffset;
        }
        if (res->dstroi.yOffset+res->dstroi.height>dest->height) {
            res->dstroi.height=dest->height-res->dstroi.yOffset;
        }
            
        if (!(options&CAM_NO_ROI_INTERSECTION)) {
            // Finally, clip source ROI if need be
            if (res->srcroi.width>res->dstroi.width) res->srcroi.width=res->dstroi.width;
            if (res->srcroi.height>res->dstroi.height) res->srcroi.height=res->dstroi.height;
        }
    }
    
    if ((res->srcroi.width<0)||(res->srcroi.height<0)) {
        camSetErrorStr("incorrect source and/or dest region of interest");
        retval=0;
    }

    if (src->dataOrder==CAM_DATA_ORDER_PIXEL) {
        res->srcinc=src->nChannels;
        res->srcpinc=0;
    } else {
        res->srcinc=1;
        res->srcpinc=src->widthStep*src->height;
    }
    res->srclinc=src->widthStep;

    // Set pointers to the right place
    if ((src->depth&CAM_DEPTH_MASK)>8) {
        res->srcptr=src->imageData+res->srcroi.yOffset*src->widthStep+(res->srcroi.xOffset<<((src->depth&CAM_DEPTH_MASK)>>4))*res->srcinc;
        // For 16 bits, divide by 2 (shift right of 1)
	// For 32 bits, divide by 4 (shift right of 2)
        res->srcpinc >>= ((src->depth&CAM_DEPTH_MASK)>>4);
	res->srclinc >>= ((src->depth&CAM_DEPTH_MASK)>>4);
    } else {
        res->srcptr=src->imageData+res->srcroi.yOffset*src->widthStep+res->srcroi.xOffset*res->srcinc;
    }
    if ((src->nChannels!=1)&&(res->srcroi.coi>0)&&(res->srcroi.coi<=src->nChannels)) {
        if (src->dataOrder==CAM_DATA_ORDER_PLANE) {
            res->srcchoffset=src->height*src->widthStep*(res->srcroi.coi-1);
        } else res->srcchoffset=res->srcroi.coi-1;
        res->srcptr+=res->srcchoffset;
        res->nChannels=1;
    } else {
        res->srcchoffset=0;
        res->nChannels=src->nChannels;
    }

    if (dest) {
        // Smart dest allocation
        if (dest->imageData==NULL) {
            if (res->nChannels==1) {
                if (!camAllocateImage(dest,res->srcroi.width,res->srcroi.height,src->depth)) return 0;
            } else if (res->nChannels==3) {
                if (src->dataOrder==CAM_DATA_ORDER_PLANE) {
                    if (!camAllocateRGBImage(dest,res->srcroi.width,res->srcroi.height)) return 0;
                } else {
                    if (!camAllocateYUVImage(dest,res->srcroi.width,res->srcroi.height)) return 0;
                }
            } else {
                camSetErrorStr("Automatic allocation failed. Frame type not supported");
                return 0;
            }
        }
        
        if (dest->dataOrder==CAM_DATA_ORDER_PIXEL) {
            res->dstinc=dest->nChannels;
            res->dstpinc=0;
        } else {
            res->dstinc=1;
            res->dstpinc=dest->widthStep*dest->height;
        }
        res->dstlinc=dest->widthStep;
        
        if ((dest->depth&CAM_DEPTH_MASK)>8) {
            res->dstptr=dest->imageData+res->dstroi.yOffset*dest->widthStep+(res->dstroi.xOffset<<((dest->depth&CAM_DEPTH_MASK)>>4))*res->dstinc;
            res->dstpinc>>=((dest->depth&CAM_DEPTH_MASK)>>4);
            res->dstlinc>>=((dest->depth&CAM_DEPTH_MASK)>>4);
        } else {
            res->dstptr=dest->imageData+res->dstroi.yOffset*dest->widthStep+res->dstroi.xOffset*res->dstinc;
        }
        if ((dest->nChannels!=1)&&(res->dstroi.coi>0)&&(res->dstroi.coi<=dest->nChannels)) {
            if (dest->dataOrder==CAM_DATA_ORDER_PLANE) {
                res->dstchoffset=dest->height*dest->widthStep*(res->dstroi.coi-1);
            } else res->dstchoffset=res->dstroi.coi-1;
            res->dstptr+=res->dstchoffset;
            if ((res->nChannels!=1)&&(!(options&CAM_IGNORE_COI_MISMATCH))) {
                camSetErrorStr("source and destination have a different number of channels of interest");
                retval=0;
            }
        } else {
            res->dstchoffset=0;
            if ((res->nChannels!=dest->nChannels)&&(!(options&CAM_IGNORE_COI_MISMATCH)))  {
                camSetErrorStr("source and destination have a different number of channels of interest");
                retval=0;
            }
        }
    } else {
	res->dstptr = NULL;
    }

    return retval;
}

void camInternalROIPolicyExit(CamInternalROIPolicyStruct *s)
{
    if (s->auto_mask.runs!=NULL) {
        camRLEDeallocate(&s->auto_mask);
    }
}

/* Image allocation utility routine
 */
int camAllocateImageEx(CamImage *image, int width, int height, int depth, int channelseq)
{
    switch (channelseq) {
    case 0:
    case CAM_CHANNELSEQ_GREY:
        return camAllocateImage(image, width, height, depth);
    case 1:
    case CAM_CHANNELSEQ_RGB:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateRGBImage(image, width, height);
        break;
    case 2:
    case CAM_CHANNELSEQ_RGBA:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateRGBAImage(image, width, height);
        break;
    case 3:
    case CAM_CHANNELSEQ_YUV:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateYUVImage(image, width, height);
        break;
    case 4:
    case CAM_CHANNELSEQ_BGR:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateBGRImage(image, width, height);
        break;
    case 5:
    case CAM_CHANNELSEQ_BGRA:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateBGRAImage(image, width, height);
        break;
    default:
        image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
    }
    return 0;
}

int camFillImageHeader(CamImage *image, int width, int height, int depth, int channelseq)
{
    switch (channelseq) {
    case 0:
    case CAM_CHANNELSEQ_GREY:
        return camAllocateImage(image, width | CAM_HEADER_ONLY, height, depth);
    case 1:
    case CAM_CHANNELSEQ_RGB:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateRGBImage(image, width | CAM_HEADER_ONLY, height);
        break;
    case 2:
    case CAM_CHANNELSEQ_RGBA:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateRGBAImage(image, width | CAM_HEADER_ONLY, height);
        break;
    case 3:
    case CAM_CHANNELSEQ_YUV:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateYUVImage(image, width | CAM_HEADER_ONLY, height);
        break;
    case 4:
    case CAM_CHANNELSEQ_BGR:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateBGRImage(image, width | CAM_HEADER_ONLY, height);
        break;
    case 5:
    case CAM_CHANNELSEQ_BGRA:
        if (depth!=CAM_DEPTH_8U) {
            image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
        } else return camAllocateBGRAImage(image, width | CAM_HEADER_ONLY, height);
        break;
    default:
        image->imageData=NULL; image->roi=NULL; image->imageSize=0; image->mask=NULL; image->imageDataOrigin=NULL; image->depth=CAM_DEPTH_8U; image->nChannels=1; 
    }
    return 0;
}

int camAllocateImage(CamImage *image, int width, int height, int depth)
{
    int i,psize;
    image->nSize=sizeof(CamImage);
    image->id=0;
    image->channelSeq[0]='G';
    image->channelSeq[1]=0;
    image->channelSeq[2]=0;
    image->channelSeq[3]=0;
    image->colorModel[0]='G';
    image->colorModel[1]=0;
    image->colorModel[2]=0;
    image->colorModel[3]=0;
    image->nChannels=1;
    image->alphaChannel=0;
    image->depth=depth;
    image->dataOrder=CAM_DATA_ORDER_PIXEL;
    image->origin=CAM_ORIGIN_TL;
    // Align 16-bits deep images to 16 bytes boundaries instead of 8 bytes (default)
    if ((depth&CAM_DEPTH_MASK)>8) {
        image->align=CAM_ALIGN_16BYTES; // Aligned on 16 bytes boundaries
    } else {
        image->align=CAM_ALIGN_QWORD;   // Aligned on 8 bytes boundaries
    }
    image->width=width & ~CAM_HEADER_ONLY;
    image->height=height;
    image->roi=NULL;
    image->mask=NULL;
    image->imageId=NULL;
    image->misc=NULL;
    image->imageData=NULL;
    for (i=0;i<4;i++) {
	image->borderMode[i]=CAM_BORDER_REPLICATE;
	image->borderConst[i]=0;
    }    
    psize=depth&CAM_DEPTH_MASK; // Pixel size in bits (1, 8, 16, 32 bits)
    if ((psize>8)&&(psize<16)) psize=16; // For 10 or 12 bits images
    if (psize==1) {
	image->widthStep=((((image->width*image->nChannels-1)/8+image->align)/image->align)*image->align);
    } else {
        image->widthStep=(((image->width*image->nChannels*psize/8+image->align-1)/image->align)*image->align);
    }
    image->imageSize=image->widthStep*image->height;
    if (width & CAM_HEADER_ONLY) {
        image->imageDataOrigin = NULL;
        image->imageData = NULL;
    } else {
        image->imageDataOrigin=(unsigned char*)malloc(image->imageSize+image->align);
        image->imageData=(unsigned char*)(((int)(image->imageDataOrigin-1)/image->align)*image->align+image->align);
        if (image->imageData == NULL) {
	    camSetErrorStr("Memory allocation failed");
	    return 0;
        }
    }
    return 1;
}

int camAllocateYUVImage(CamImage *image, int width, int height)
{
    int i,psize;
    image->nSize=sizeof(CamImage);
    image->channelSeq[0]='Y';
    image->channelSeq[1]='U';
    image->channelSeq[2]='V';
    image->channelSeq[3]=0;
    image->colorModel[0]='Y';
    image->colorModel[1]='U';
    image->colorModel[2]='V';
    image->colorModel[3]=0;
    image->nChannels=3;
    image->alphaChannel=0;
    image->depth=8;
    image->dataOrder=CAM_DATA_ORDER_PLANE;
    image->origin=CAM_ORIGIN_TL;
    image->align=CAM_ALIGN_QWORD; // Aligned on 8 bytes boundaries
    image->width=width & ~CAM_HEADER_ONLY;
    image->height=height;
    image->roi=NULL;
    image->mask=NULL;
    image->imageId=NULL;
    image->misc=NULL;
    image->imageData=NULL;
    for (i=0;i<4;i++) {
	image->borderMode[i]=CAM_BORDER_REPLICATE;
	image->borderConst[i]=0;
    }    
    psize=8;
    image->widthStep=(((image->width+image->align-1)*psize/8/image->align)*image->align);
    image->imageSize=image->widthStep*3*image->height*psize/8;
    if (width & CAM_HEADER_ONLY) {
        image->imageDataOrigin = NULL;
        image->imageData = NULL;
    } else {
        image->imageDataOrigin=(unsigned char*)malloc(image->imageSize+image->align);
        image->imageData=(unsigned char*)(((int)(image->imageDataOrigin-1)/image->align)*image->align+image->align);
        if (image->imageData == NULL) {
            camSetErrorStr("Memory allocation failed");
            return 0;
        }
    }
    return 1;
}

int camAllocateHLSImage(CamImage *image, int width, int height)
{
    int i,psize;
    image->nSize=sizeof(CamImage);
    image->channelSeq[0]='H';
    image->channelSeq[1]='L';
    image->channelSeq[2]='S';
    image->channelSeq[3]=0;
    image->colorModel[0]='H';
    image->colorModel[1]='L';
    image->colorModel[2]='S';
    image->colorModel[3]=0;
    image->nChannels=3;
    image->alphaChannel=0;
    image->depth=16;
    image->dataOrder=CAM_DATA_ORDER_PLANE;
    image->origin=CAM_ORIGIN_TL;
    image->align=CAM_ALIGN_16BYTES; // Aligned on 16 bytes boundaries
    image->width=width & ~CAM_HEADER_ONLY;
    image->height=height;
    image->roi=NULL;
    image->mask=NULL;
    image->imageId=NULL;
    image->misc=NULL;
    image->imageData=NULL;
    for (i=0;i<4;i++) {
	image->borderMode[i]=CAM_BORDER_REPLICATE;
	image->borderConst[i]=0;
    }    
    psize=8;
    image->widthStep=(((image->width+image->align-1)*psize/8/image->align)*image->align);
    image->imageSize=image->widthStep*3*image->height*psize/8;
    if (width & CAM_HEADER_ONLY) {
        image->imageDataOrigin = NULL;
        image->imageData = NULL;
    } else {
        image->imageDataOrigin=(unsigned char*)malloc(image->imageSize+image->align);
        image->imageData=(unsigned char*)(((int)(image->imageDataOrigin-1)/image->align)*image->align+image->align);
        if (image->imageData == NULL) {
            camSetErrorStr("Memory allocation failed");
            return 0;
        }
    }
    return 1;
}

int camAllocateRGBImage(CamImage *image, int width, int height)
{
    int i,psize;
    image->nSize=sizeof(CamImage);
    image->channelSeq[0]='R';
    image->channelSeq[1]='G';
    image->channelSeq[2]='B';
    image->channelSeq[3]=0;
    image->colorModel[0]='R';
    image->colorModel[1]='G';
    image->colorModel[2]='B';
    image->colorModel[3]=0;
    image->nChannels=3;
    image->alphaChannel=0;
    image->depth=8;
    image->dataOrder=CAM_DATA_ORDER_PIXEL;
    image->origin=CAM_ORIGIN_TL;
    image->align=CAM_ALIGN_4BYTES; // Aligned on 32bits boundaries
    image->width=width & ~CAM_HEADER_ONLY;
    image->height=height;
    image->roi=NULL;
    image->mask=NULL;
    image->imageId=NULL;
    image->misc=NULL;
    for (i=0;i<4;i++) {
	image->borderMode[i]=CAM_BORDER_REPLICATE;
	image->borderConst[i]=0;
    }    
    psize=8;
    image->widthStep=(((image->width*image->nChannels+image->align-1)*psize/8/image->align)*image->align);
    image->imageSize=image->widthStep*image->height*psize/8;
    if (width & CAM_HEADER_ONLY) {
        image->imageDataOrigin = NULL;
        image->imageData = NULL;
    } else {
        image->imageDataOrigin=(unsigned char*)malloc(image->imageSize+image->align);
        image->imageData=(unsigned char*)(((int)(image->imageDataOrigin-1)/image->align)*image->align+image->align);
        if (image->imageData == NULL) {
            camSetErrorStr("Memory allocation failed");
            return 0;
        }
    }
    return 1;
}

int camAllocateRGBAImage(CamImage *image, int width, int height)
{
    int i,psize;
    image->nSize=sizeof(CamImage);
    image->channelSeq[0]='R';
    image->channelSeq[1]='G';
    image->channelSeq[2]='B';
    image->channelSeq[3]='A';
    image->colorModel[0]='R';
    image->colorModel[1]='G';
    image->colorModel[2]='B';
    image->colorModel[3]='A';
    image->nChannels=4;
    image->alphaChannel=4;
    image->depth=8;
    image->dataOrder=CAM_DATA_ORDER_PIXEL;
    image->origin=CAM_ORIGIN_TL;
    image->align=CAM_ALIGN_4BYTES; // Aligned on 32 bits boundaries
    image->width=width & ~CAM_HEADER_ONLY;
    image->height=height;
    image->roi=NULL;
    image->mask=NULL;
    image->imageId=NULL;
    image->misc=NULL;
    for (i=0;i<4;i++) {
	image->borderMode[i]=CAM_BORDER_REPLICATE;
	image->borderConst[i]=0;
    }    
    psize=8;
    image->widthStep=(((image->width*image->nChannels+image->align-1)*psize/8/image->align)*image->align);
    image->imageSize=image->widthStep*image->height*psize/8;
    if (width & CAM_HEADER_ONLY) {
        image->imageDataOrigin = NULL;
        image->imageData = NULL;
    } else {
        image->imageDataOrigin=(unsigned char*)malloc(image->imageSize+image->align);
        image->imageData=(unsigned char*)(((int)(image->imageDataOrigin-1)/image->align)*image->align+image->align);
        if (image->imageData == NULL) {
            camSetErrorStr("Memory allocation failed");
            return 0;
        }
    }
    return 1;
}

int camAllocateBGRImage(CamImage *image, int width, int height)
{
    int i,psize;
    image->nSize=sizeof(CamImage);
    image->channelSeq[0]='B';
    image->channelSeq[1]='G';
    image->channelSeq[2]='R';
    image->channelSeq[3]=0;
    image->colorModel[0]='R';
    image->colorModel[1]='G';
    image->colorModel[2]='B';
    image->colorModel[3]=0;
    image->nChannels=3;
    image->alphaChannel=0;
    image->depth=8;
    image->dataOrder=CAM_DATA_ORDER_PIXEL;
    image->origin=CAM_ORIGIN_TL;
    image->align=CAM_ALIGN_4BYTES; // Aligned on 32 bits boundaries
    image->width=width & ~CAM_HEADER_ONLY;
    image->height=height;
    image->roi=NULL;
    image->mask=NULL;
    image->imageId=NULL;
    image->misc=NULL;
    for (i=0;i<4;i++) {
	image->borderMode[i]=CAM_BORDER_REPLICATE;
	image->borderConst[i]=0;
    }    
    psize=8;
    image->widthStep=(((image->width*image->nChannels+image->align-1)*psize/8/image->align)*image->align);
    image->imageSize=image->widthStep*image->height*psize/8;
    if (width & CAM_HEADER_ONLY) {
        image->imageDataOrigin = NULL;
        image->imageData = NULL;
    } else {
        image->imageDataOrigin=(unsigned char*)malloc(image->imageSize+image->align);
        image->imageData=(unsigned char*)(((int)(image->imageDataOrigin-1)/image->align)*image->align+image->align);
        if (image->imageData == NULL) {
            camSetErrorStr("Memory allocation failed");
            return 0;
        }
    }
    return 1;
}

int camAllocateBGRAImage(CamImage *image, int width, int height)
{
    int i,psize;
    image->nSize=sizeof(CamImage);
    image->channelSeq[0]='B';
    image->channelSeq[1]='G';
    image->channelSeq[2]='R';
    image->channelSeq[3]='A';
    image->colorModel[0]='R';
    image->colorModel[1]='G';
    image->colorModel[2]='B';
    image->colorModel[3]='A';
    image->nChannels=4;
    image->alphaChannel=4;
    image->depth=8;
    image->dataOrder=CAM_DATA_ORDER_PIXEL;
    image->origin=CAM_ORIGIN_TL;
    image->align=CAM_ALIGN_4BYTES; // Aligned on 32-bits boundaries
    image->width=width & ~CAM_HEADER_ONLY;
    image->height=height;
    image->roi=NULL;
    image->mask=NULL;
    image->imageId=NULL;
    image->misc=NULL;
    for (i=0;i<4;i++) {
	image->borderMode[i]=CAM_BORDER_REPLICATE;
	image->borderConst[i]=0;
    }    
    psize=8;
    image->widthStep=(((image->width*image->nChannels+image->align-1)*psize/8/image->align)*image->align);
    image->imageSize=image->widthStep*image->height*psize/8;
    if (width & CAM_HEADER_ONLY) {
        image->imageDataOrigin = NULL;
        image->imageData = NULL;
    } else {
        image->imageDataOrigin=(unsigned char*)malloc(image->imageSize+image->align);
        image->imageData=(unsigned char*)(((int)(image->imageDataOrigin-1)/image->align)*image->align+image->align);
        if (image->imageData == NULL) {
            camSetErrorStr("Memory allocation failed");
            return 0;
        }
    }
    return 1;
}

int camDeallocateImage(CamImage *image)
{
    if (image->imageData) {
        if (image->imageDataOrigin) {
            free(image->imageDataOrigin);
            image->imageDataOrigin=NULL;
        }
        image->imageData=NULL;
    } 
    return 1;
}

int camFreeImage(CamImage *image)
{
    return camDeallocateImage(image);
}

int camClone(CamImage *source, CamImage *dest)
{
    CAM_CHECK_ARGS2(camClone,source->imageData!=NULL,"source image is not initialized");
    *dest=*source;
    dest->imageDataOrigin=(unsigned char*)malloc(dest->imageSize+dest->align);
    dest->imageData=(unsigned char*)(((int)(dest->imageDataOrigin-1)/dest->align)*dest->align+dest->align);
    if (dest->imageData==NULL) {
        camError("camClone","Out of memory");
        return 0;
    }
    memcpy(dest->imageData,source->imageData,dest->imageSize);
    return 1;
}

int camSetRLEMask(CamImage *image, CamRLEImage *mask)
{
    image->mask=(void*)mask;
    return 1;
}

int camSetMask(CamImage *image, CamImage *mask)
{
    image->mask=(void*)mask;
    return 1;
}

int camRefCopy(CamImage *source, CamImage *dest)
{
    *dest=*source;
    dest->imageDataOrigin=NULL; // To avoid any deallocation
    return 1;
}

int camSetROI(CamROI *roi, int coi, int xOffset, int yOffset, int width, int height)
{
    roi->coi=coi;
    roi->height=height;
    roi->width=width;
    roi->xOffset=xOffset;
    roi->yOffset=yOffset;
    return 1;
}

int camSetMaxROI(CamROI *roi, CamImage *image)
{
    roi->coi=0;
    roi->xOffset=0;
    roi->yOffset=0;
    if (image) {
        roi->width=image->width;
        roi->height=image->height;
    } else {
        roi->width=0;
        roi->height=0;
    }
    return 1;
}

int camReduceROI(CamROI *roi, int pixels)
{
    roi->xOffset+=pixels;
    roi->yOffset+=pixels;
    roi->width-=pixels*2;
    roi->height-=pixels*2;
    if ((roi->width<1)||(roi->height<1)) return 0;
    return 1;
}

int camEnlargeROI(CamROI *roi, int pixels)
{
    roi->xOffset-=pixels;
    roi->yOffset-=pixels;
    roi->width+=pixels*2;
    roi->height+=pixels*2;
    if ((roi->width<1)||(roi->height<1)) return 0;
    return 1;
}

int camSetBorder(CamImage *image, int borderValue)
{
    image->borderConst[0]=borderValue;
    image->borderConst[1]=borderValue;
    image->borderConst[2]=borderValue;
    image->borderConst[3]=borderValue;
    return 1;
}

int camClipROI(CamImage *image)
{
    if (image->roi) {
	if (image->roi->xOffset<0) {
	    image->roi->width+=image->roi->xOffset;
	    image->roi->xOffset=0;
	}
	if (image->roi->yOffset<0) {
	    image->roi->height+=image->roi->yOffset;
	    image->roi->yOffset=0;
	}
	if (image->roi->xOffset+image->roi->width>image->width) {
	    if (image->roi->xOffset>=image->width) {
		image->roi->xOffset=image->width;
		image->roi->width=0;
	    } else {
		image->roi->width=image->width-image->roi->xOffset;
	    }
	}
	if (image->roi->yOffset+image->roi->height>image->height) {
	    if (image->roi->yOffset>=image->height) {
		image->roi->yOffset=image->height;
		image->roi->height=0;
	    } else {
		image->roi->height=image->height-image->roi->yOffset;
	    }
	}
        if (image->roi->width<0) image->roi->width=0;
        if (image->roi->height<0) image->roi->height=0;
    }
    return 1;
}

int camClip(CamROI *roi, CamImage *image)
{
    if (image->roi) {
        if (roi->xOffset<image->roi->xOffset) {
	    roi->width+=image->roi->xOffset-roi->xOffset;
	    roi->xOffset=image->roi->xOffset;
	}
	if (roi->yOffset<image->roi->yOffset) {
	    roi->height+=image->roi->yOffset-roi->yOffset;
	    roi->yOffset=image->roi->yOffset;
	}
	if (roi->xOffset+roi->width>image->roi->xOffset+image->roi->width) {
	    if (roi->xOffset>=image->roi->xOffset+image->roi->width) {
		roi->xOffset=image->roi->xOffset+image->roi->width;
		roi->width=0;
	    } else {
		roi->width=image->roi->xOffset+image->roi->width-roi->xOffset;
	    }
	}
	if (roi->yOffset+roi->height>image->roi->yOffset+image->roi->height) {
	    if (roi->yOffset>=image->roi->yOffset+image->roi->height) {
		roi->yOffset=image->roi->yOffset+image->roi->height;
		roi->height=0;
	    } else {
		roi->height=image->roi->yOffset+image->roi->height-roi->yOffset;
	    }
	}
        if (image->roi->width<0) image->roi->width=0;
        if (image->roi->height<0) image->roi->height=0;
    } else {
        if (roi->xOffset<0) {
	    roi->width+=roi->xOffset;
	    roi->xOffset=0;
	}
	if (roi->yOffset<0) {
	    roi->height+=roi->yOffset;
	    roi->yOffset=0;
	}
	if (roi->xOffset+roi->width>image->width) {
	    if (roi->xOffset>=image->width) {
		roi->xOffset=image->width;
		roi->width=0;
	    } else {
		roi->width=image->width-roi->xOffset;
	    }
	}
	if (roi->yOffset+roi->height>image->height) {
	    if (roi->yOffset>=image->height) {
		roi->yOffset=image->height;
		roi->height=0;
	    } else {
		roi->height=image->height-roi->yOffset;
	    }
	}
        if (roi->width<0) roi->width=0;
        if (roi->height<0) roi->height=0;
    }
    return 1;
}

int camROIIntersect(CamROI *roi1, CamROI *roi2, CamROI *res)
{
    *res=*roi1;
    if (roi2->xOffset>roi1->xOffset) {
        res->xOffset=roi2->xOffset;
        res->width-=roi2->xOffset-roi1->xOffset;
    }
    if (roi2->xOffset+roi2->width<roi1->xOffset+roi1->width) {
        res->width-=roi1->xOffset+roi1->width-roi2->xOffset-roi2->width;
    }
    if (roi2->yOffset>roi1->yOffset) {
        res->yOffset=roi2->yOffset;
        res->height-=roi2->yOffset-roi1->yOffset;
    }
    if (roi2->yOffset+roi2->height<roi1->yOffset+roi1->height) {
        res->height-=roi1->yOffset+roi1->height-roi2->yOffset-roi2->height;
    }
    return 1;
}

int camAlphaComposite(CamImage *source1, CamImage *source2, CamImage *dest)
{
    unsigned char *src1ptr,*src2ptr,*dstptr,*cpsrc1ptr,*cpsrc2ptr,*cpdstptr;
    int i,x,y,width,height,alpha,valpix;
    
    // ROI and masking are not taken into account by this function
    CAM_CHECK_ARGS2(camAlphaComposite,source1->imageData!=NULL,"source1 is not allocated");
    CAM_CHECK_ARGS2(camAlphaComposite,source2->imageData!=NULL,"source2 is not allocated");
    CAM_CHECK_ARGS(camAlphaComposite,source2->nChannels==4);
    CAM_CHECK_ARGS(camAlphaComposite,source1->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camAlphaComposite,source2->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camAlphaComposite,(source1->width==source2->width)&&(source1->height==source2->height));

    // Automatic dest allocation
    if (dest->imageData==NULL) {
        if (!camAllocateRGBImage(dest,source1->width,source2->height)) return 0;
    }
    CAM_CHECK_ARGS(camAlphaComposite,(dest->width==source1->width)&&(dest->height==source1->height));
    CAM_CHECK_ARGS(camAlphaComposite,(dest->nChannels==3)||(dest->nChannels==4));
    width=source1->width;
    height=source2->height;
    src1ptr=source1->imageData;
    src2ptr=source2->imageData;
    dstptr=dest->imageData;

    for (y=0;y<height;y++) {
        cpsrc1ptr=src1ptr;
        cpsrc2ptr=src2ptr;
        cpdstptr=dstptr;
        for (x=0;x<width;x++,src1ptr+=source1->nChannels,src2ptr+=4) {
            alpha=(int)src2ptr[3];
            if (alpha) {
                for (i=0;i<3;i++) {
                    valpix=((256-alpha)*src1ptr[i]+alpha*src2ptr[i])>>8;
                    dstptr[i]=valpix;
                }
            } else {
                for (i=0;i<3;i++) {
                    dstptr[i]=src1ptr[i];
                }
            }
            dstptr+=dest->nChannels;
        }
        if (dest->nChannels==4) {
            dstptr=cpdstptr;
            for (x=0;x<width;x++,dstptr+=4) dstptr[3]=0;
        }
        src1ptr=cpsrc1ptr+source1->widthStep;
        src2ptr=cpsrc2ptr+source2->widthStep;
        dstptr=cpdstptr+dest->widthStep;
    }

    return 1;
}

int camRGB(int r, int g, int b) {
    return ((r)|((g)<<8)|((b)<<16));
}

int camRGBA(int r, int g, int b, int a) {
    return ((r)|((g)<<8)|((b)<<16)|((a)<<24));
}

#ifdef _WIN32
#include <process.h>
char camImageViewer[256]="mspaint.exe";
#else
char camImageViewer[256]="eog"; // Eye of gnome
#endif

int camSetImageViewer(char *iv)
{
    return (strncpy(camImageViewer,iv,256))?1:0;
}

int camView(CamImage *image)
{
#ifdef _WIN32
    if (camSaveBMP(image,"~camellia.bmp")) {
        if (_spawnlp(_P_NOWAIT,camImageViewer,camImageViewer,"~camellia.bmp",NULL)==-1) return 0;
        return 1;
#else
    char str[256];
    sprintf(str,"%s ~camellia.bmp &",camImageViewer);
    if (camSaveBMP(image,"~camellia.bmp")) {
        return system(str);
#endif
    }
    return 0;
}

// Configuration file management functions
int camLoadConfig(const char *filename, CamConfig *config)
{
    FILE *handle;
    char string[1024],*ptr;
    config->nbEntries=0;
    if ((handle=fopen(filename,"rt"))==NULL) return 0;
    while (!feof(handle)) {
        if (fgets(string,1024,handle)==NULL) break;
        if (string[0]=='#') continue; // Comments in the file
        if (string[0]=='\n') continue; // Blank line        
        if (string[0]=='\r') continue; // Blank line        
        if ((ptr=strtok(string,"="))==NULL) break;
        strcpy(config->parameter[config->nbEntries],ptr);
        if ((ptr=strtok(NULL,"\n\r#"))==NULL) break;
        strcpy(config->value[config->nbEntries],ptr);
        config->nbEntries++;
    }
    fclose(handle);
    return config->nbEntries;
}

int camConfigInt(const CamConfig *config, const char *entry)
{
    int i;
    for (i=0;i<config->nbEntries;i++) {
        if (strcmp(config->parameter[i],entry)==0) {
            return atoi(config->value[i]);
        }
    }
    return 0xdead;
}

float camConfigFloat(const CamConfig *config, const char *entry)
{
    int i;
    for (i=0;i<config->nbEntries;i++) {
        if (strcmp(config->parameter[i],entry)==0) {
            return (float)atof(config->value[i]);
        }
    }
    return 6.55957f;
}

const char *camConfigString(const CamConfig *config, const char *entry)
{
    int i;
    for (i=0;i<config->nbEntries;i++) {
        if (strcmp(config->parameter[i],entry)==0) {
            return config->value[i];
        }
    }
    return "null";
}

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation
#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL unsigned char
#define CAM_PIXEL_DST CAM_PIXEL
#define CAM_FAST 8
#define camZoom2x camZoom2x8
#define camCopy camCopy8
#define camDecimateNN camDecimateNN8
#define camDownscaling2x2 camDownscaling2x28
#define camSet camSet8
#include "cam_utils_code.c"
#undef camZoom2x
#undef camCopy
#undef camDecimateNN
#undef camDownscaling2x2
#undef camSet
#undef CAM_FAST

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL unsigned short
#define CAM_PIXEL_DST CAM_PIXEL
#define CAM_FAST 16
#define camZoom2x camZoom2x16
#define camCopy camCopy16
#define camDecimateNN camDecimateNN16
#define camDownscaling2x2 camDownscaling2x216
#define camSet camSet16
#include "cam_utils_code.c"
#undef camZoom2x
#undef camCopy
#undef camDecimateNN
#undef camDownscaling2x2
#undef camSet
#undef CAM_FAST

#define CAM_COPY_ONLY

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL unsigned char
#define CAM_PIXEL_DST unsigned short
#define camCopy camCopy8to16
#include "cam_utils_code.c"
#undef camCopy

#undef CAM_PIXEL
#undef CAM_PIXEL_DST
#define CAM_PIXEL unsigned short
#define CAM_PIXEL_DST unsigned char
#define CAM_SATURATE
#define camCopy camCopy16to8
#include "cam_utils_code.c"
#undef camCopy
#undef CAM_SATURATE

int camZoom2x(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camZoom2x16(source,dest);
    } else {
	return camZoom2x8(source,dest);
    }
}

int camCopy(CamImage* source, CamImage* dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camCopy16to8(source,dest);
        } else return camCopy16(source,dest);
    } else {
        if ((dest->depth&CAM_DEPTH_MASK)==8) {
            return camCopy8(source,dest);
        } else return camCopy8to16(source,dest);
    }
}

int camDecimateNN(CamImage *source, CamImage *dest, int factor)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camDecimateNN16(source,dest,factor);
    } else {
	return camDecimateNN8(source,dest,factor);
    }
}

int camDownscaling2x2(CamImage *source, CamImage *dest)
{
    if ((source->depth&CAM_DEPTH_MASK)>8) {
	return camDownscaling2x216(source,dest);
    } else {
	return camDownscaling2x28(source,dest);
    }
}

int camSet(CamImage *image, int fillValue)
{
    if ((image->depth&CAM_DEPTH_MASK)==1) {
        CAM_CHECK_ARGS(camSet,image->roi==NULL);
        if (fillValue) {
            // for (i=0;i<image->imageSize;i++) image->imageData[i]=0xff;
            memset(image->imageData,0xff,image->imageSize);
        } else {
            // for (i=0;i<image->imageSize;i++) image->imageData[i]=0;
            memset(image->imageData,0,image->imageSize);
        }   
        return 1; 
    } else if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camSet16(image,fillValue);
    } else {
	return camSet8(image,fillValue);
    }
}
#else
#define CAM_PIXEL_DST CAM_PIXEL
#include "cam_utils_code.c"
#endif


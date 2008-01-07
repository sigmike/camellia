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

#ifdef CAM_GRAYSCALE

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
// camMonadicArithm1U : source is n bits, dest is one bit
int camMonadicArithm1U(CamImage *source, CamImage *dest, CamArithmParams *params)
{
    int x,y;
    int width,height;
    CAM_PIXEL *srcptr,*cpsrcptr;
    CAM_BIT_BLOCK *dstptr,*cpdstptr;
    CAM_BIT_BLOCK bit_block,eol_block;
    int sol_offset,bit_offset;

    int c1=params->c1;
    int c2=params->c2;
    int c3=params->c3;

    CamInternalROIPolicyStruct iROI;

    CAM_CHECK_ARGS(camMonadicArithm,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camMonadicArithm,(source->depth&CAM_DEPTH_MASK)>=8);

    // ROI (Region Of Interest) management
    CAM_CHECK(camMonadicArithm,camInternalROIPolicy(source, dest, &iROI, 0));
    srcptr=(CAM_PIXEL*)iROI.srcptr;
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;

    if (dest->roi) {
	sol_offset=dest->roi->xOffset%CAM_BIT_BLOCK_SIZE;
	dstptr=(CAM_BIT_BLOCK*)(dest->imageData+dest->roi->yOffset*dest->widthStep)+(dest->roi->xOffset/CAM_BIT_BLOCK_SIZE);
    } else {
	sol_offset=0;
	dstptr=(CAM_BIT_BLOCK*)dest->imageData;
    }

    switch(params->operation) {
    case CAM_ARITHM_ABS:
	camError("camMonadicArithm","CAM_ARITHM_ABS operation is not possible on binary images");
	break;
    case CAM_ARITHM_SELECT:
	if (c2) {
	    FOREACH_PIXEL {
		STORE_BIT((int)(*srcptr==c1));	    
	    } END_FOREACH_PIXEL
	} else {
	    FOREACH_PIXEL {
		STORE_BIT((int)(*srcptr!=c1));	    
	    } END_FOREACH_PIXEL
	}
	break;
    case CAM_ARITHM_THRESHOLD:
	if (c2) {
	    FOREACH_PIXEL {
		STORE_BIT((int)(*srcptr<c1));
	    } END_FOREACH_PIXEL
	} else {
	    FOREACH_PIXEL {
		STORE_BIT((int)(*srcptr>=c1));
	    } END_FOREACH_PIXEL
	}
	break;
    case CAM_ARITHM_DOUBLE_THRESHOLD:
	if (c3) {
	    FOREACH_PIXEL {
		STORE_BIT((int)((*srcptr>c1)&&(*srcptr<c2)));
	    } END_FOREACH_PIXEL
	} else {
	    FOREACH_PIXEL {
		STORE_BIT((int)((*srcptr<=c1)||(*srcptr>=c2)));
	    } END_FOREACH_PIXEL
	}
	break;
    }
    return 1;
}

#undef FOREACH_PIXEL
#undef END_FOREACH_PIXEL

#define FOREACH_PIXEL \
for (y=0;y<height;y++) {                            \
    cpsrcptr=srcptr; cpdstptr=dstptr; startx=0;     \
    do {                                            \
        if (!iROI.mask) endx=width; else {          \
            while ((run->value==0)&&(run->line==y)) \
            { startx+=run->length; run++; }         \
            if (run->line!=y) break;                \
            endx=startx+run->length;                \
            srcptr=cpsrcptr+startx*iROI.srcinc;     \
            dstptr=cpdstptr+startx*iROI.dstinc;     \
        }                                           \
        for (x=startx;x<endx;x++,srcptr+=iROI.srcinc,dstptr+=iROI.dstinc)   

#define END_FOREACH_PIXEL \
        if (iROI.mask) { startx+=run->length; run++; } \
    } while ((run)&&(run->line==y));                \
    srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);   \
    dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);     \
}
	
// Note that all monadic arithmetic operations can be implemented in hardware using LUTs
// (see cam_LUT.c)
int camMonadicArithm(CamImage *source, CamImage *dest, CamArithmParams *params)
{
    int x,y,valpix;
    int width,height;
    CAM_PIXEL *srcptr,*dstptr,*cpsrcptr,*cpdstptr;
    int c1=params->c1;
    int c2=params->c2;
    int c3=params->c3;
    int c4=params->c4;
    CamInternalROIPolicyStruct iROI;
    int acc=0;
    CamRun *run;
    int startx,endx;

    // Binary images processing 
    if (dest->depth==CAM_DEPTH_1U) {
	if (params->operation==CAM_ARITHM_INVERSE) {
	    return camInverse1U(source,dest);
	} else {
	    return camMonadicArithm1U(source,dest,params);
	}
    }
    
    // ROI (Region Of Interest) management
    CAM_CHECK(camMonadicArithm,camInternalROIPolicy(source, dest, &iROI, 1));
    CAM_CHECK_ARGS(camMonadicArithm,iROI.nChannels==1);
    CAM_CHECK_ARGS(camMonadicArithm,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camMonadicArithm,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camMonadicArithm,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camMonadicArithm,(dest->depth&CAM_DEPTH_MASK)>=8);
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    srcptr=(CAM_PIXEL*)iROI.srcptr;
    dstptr=(CAM_PIXEL*)iROI.dstptr;

    // Mask management
    if (iROI.mask) {
        run=iROI.mask->runs+1; // Skip the first dummy run
    } else {
        run=NULL;
    }

    if (params->operation==CAM_ARITHM_ABS) {
        CAM_CHECK_ARGS(camMonadicArithm,(source->depth&CAM_DEPTH_SIGN)); // Check that source image is signed
    } else {
        CAM_CHECK_ARGS2(camMonadicArithm,!(source->depth&CAM_DEPTH_SIGN),"signed source images are not supported");
	CAM_CHECK_ARGS2(camMonadicArithm,!(dest->depth&CAM_DEPTH_SIGN),"signed dest images are not supported");
    }

    switch(params->operation) {
    case CAM_ARITHM_ABS:
	FOREACH_PIXEL {
	    valpix=*(CAM_SIGNED_PIXEL*)srcptr;
	    if (valpix<0) valpix=-valpix;
	    *dstptr=(CAM_PIXEL)valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_INVERSE:
	FOREACH_PIXEL {
            valpix=(CAM_PIXEL)((1<<(sizeof(CAM_PIXEL)*8))-1)-*srcptr;
            *dstptr=(CAM_PIXEL)valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_SELECT:
	FOREACH_PIXEL {
            if (*srcptr==c1) {
                *dstptr=c2; acc+=c2;
            } else {
                *dstptr=c3; acc+=c3;
            }
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_THRESHOLD:
	FOREACH_PIXEL {
            if (*srcptr<c1) {
                *dstptr=c2; acc+=c2;
            } else {
                *dstptr=c3; acc+=c3;
            }
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_DOUBLE_THRESHOLD:
	FOREACH_PIXEL {
            if ((*srcptr>c1)&&(*srcptr<c2)) {
                *dstptr=c3; acc+=c3;
            } else {
                *dstptr=c4; acc+=c4;
            }
	} END_FOREACH_PIXEL
	break;
    }

    camInternalROIPolicyExit(&iROI);
    return acc;
}

#undef FOREACH_PIXEL
#undef END_FOREACH_PIXEL

#define FOREACH_PIXEL \
for (y=0;y<height;y++) {                            \
    cpsrc1ptr=src1ptr; cpsrc2ptr=src2ptr; cpdstptr=dstptr; startx=0;     \
    do {                                            \
        if (!mask) endx=width; else {               \
            while ((run->value==0)&&(run->line==y)) \
            { startx+=run->length; run++; }         \
            if (run->line!=y) break;                \
            endx=startx+run->length;                \
            src1ptr=cpsrc1ptr+startx*iROI1.srcinc;  \
            src2ptr=cpsrc2ptr+startx*iROI2.srcinc;  \
            dstptr=cpdstptr+startx*iROI1.dstinc;    \
        }                                           \
        for (x=startx;x<endx;x++,src1ptr+=iROI1.srcinc,src2ptr+=iROI2.srcinc,dstptr+=iROI1.dstinc)   

#define END_FOREACH_PIXEL \
        if (mask) { startx+=run->length; run++; }   \
    } while ((run)&&(run->line==y));                \
    src1ptr=(CAM_PIXEL*)(((char*)cpsrc1ptr)+source1->widthStep); \
    src2ptr=(CAM_PIXEL*)(((char*)cpsrc2ptr)+source2->widthStep); \
    dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep); \
}
	
int camDyadicArithm(CamImage *source1, CamImage *source2, CamImage *dest, CamArithmParams *params)
{
    int x,y,valpix;
    int width,height;
    CAM_PIXEL *src1ptr,*src2ptr,*dstptr,*cpsrc1ptr,*cpsrc2ptr,*cpdstptr;
    int c1=params->c1;
    int c2=params->c2;
    int c3=params->c3;
    CamInternalROIPolicyStruct iROI1,iROI2;
    int acc=0;
    int valmax;
    CamRLEImage *mask;
    CamRun *run;
    int startx,endx;

    if (source1->depth==CAM_DEPTH_1U) {
	return camDyadicArithm1U(source1,source2,dest,params);
    }

    // Automatic dest allocation
    if (dest->imageData==NULL) {
        CAM_CHECK(camDyadicArithm,camInternalROIPolicy(source1, source2, &iROI1, 0));
        CAM_CHECK(camDyadicArithm,camAllocateImage(dest,iROI1.srcroi.width,iROI1.srcroi.height,source1->depth));
    }
    
    // ROI (Region Of Interest) management
    CAM_CHECK(camDyadicArithm,camInternalROIPolicy(source1, dest, &iROI1, CAM_MASK_SUPPORT));
    CAM_CHECK_ARGS(camDyadicArithm,iROI1.nChannels==1);
    CAM_CHECK(camDyadicArithm,camInternalROIPolicy(source2, dest, &iROI2, CAM_MASK_SUPPORT));
    CAM_CHECK_ARGS(camDyadicArithm,iROI2.nChannels==1);
    CAM_CHECK_ARGS2(camDyadicArithm,((iROI1.srcroi.width==iROI2.srcroi.width)&&(iROI1.srcroi.height==iROI2.srcroi.height)),"can't handle these ROIs");
    CAM_CHECK_ARGS(camDyadicArithm,(source1->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camDyadicArithm,(source1->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camDyadicArithm,(source2->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camDyadicArithm,(source2->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camDyadicArithm,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camDyadicArithm,(dest->depth&CAM_DEPTH_MASK)>=8);

    // Saturation management
    valmax=(1<<(dest->depth&CAM_DEPTH_MASK))-1;

    // ROI (Region Of Interest) management
    width=iROI1.srcroi.width;
    height=iROI1.srcroi.height;
    src1ptr=(CAM_PIXEL*)iROI1.srcptr;
    src2ptr=(CAM_PIXEL*)iROI2.srcptr;
    dstptr=(CAM_PIXEL*)iROI1.dstptr;

    // Mask management
    if (iROI1.mask) {
        mask=iROI1.mask;
        CAM_CHECK_ARGS2(camDyadicArithm,(mask->nSize==sizeof(CamRLEImage)),"Only RLE images are currently supported for masking");
        CAM_CHECK_ARGS2(camDyadicArithm,mask->width==width,"Incorrect mask image width. Should match the ROI.");
        CAM_CHECK_ARGS2(camDyadicArithm,mask->height==height,"Incorrect mask image height. Should match the ROI.");
        run=mask->runs+1; // Skip the first dummy run
    } else if (iROI2.mask) {
        mask=iROI2.mask;
        CAM_CHECK_ARGS2(camDyadicArithm,(mask->nSize==sizeof(CamRLEImage)),"Only RLE images are currently supported for masking");
        CAM_CHECK_ARGS2(camDyadicArithm,mask->width==width,"Incorrect mask image width. Should match the ROI.");
        CAM_CHECK_ARGS2(camDyadicArithm,mask->height==height,"Incorrect mask image height. Should match the ROI.");
        run=mask->runs+1; // Skip the first dummy run        
    } else {
        mask=NULL;
        run=NULL;
    }

    switch(params->operation) {
    case CAM_ARITHM_ADD:
	if (dest->depth&CAM_DEPTH_SIGN) {
	    // Signed addition
	    valmax>>=1;
	    FOREACH_PIXEL {
		valpix=*(CAM_SIGNED_PIXEL*)src1ptr+(int)*(CAM_SIGNED_PIXEL*)src2ptr;
		// Saturation :
		if (valpix<-valmax-1) valpix=-valmax-1; 
		else if (valpix>valmax) valpix=valmax;
		*dstptr=(CAM_PIXEL)valpix;
                acc+=valpix;
	    } END_FOREACH_PIXEL
	} else {
	    FOREACH_PIXEL {
		valpix=*src1ptr+(int)*src2ptr;
		// Saturation :
		if (valpix>valmax) valpix=valmax;
		*dstptr=(CAM_PIXEL)valpix;
                acc+=valpix;
	    } END_FOREACH_PIXEL
	}
	break;
    case CAM_ARITHM_SUB:
	if (dest->depth&CAM_DEPTH_SIGN) {
	    // Signed substraction
	    valmax>>=1;
	    FOREACH_PIXEL {
		valpix=*(CAM_SIGNED_PIXEL*)src1ptr-(int)*(CAM_SIGNED_PIXEL*)src2ptr;
		// Saturation :
		if (valpix<-valmax-1) valpix=-valmax-1; 
		else if (valpix>valmax) valpix=valmax;
		*dstptr=(CAM_PIXEL)valpix;
                acc+=valpix;
	    } END_FOREACH_PIXEL
	} else {
	    FOREACH_PIXEL {
		valpix=*src1ptr-(int)*src2ptr;
		// Saturation :
		if (valpix<0) valpix=0;
		else if (valpix>valmax) valpix=valmax;
		*dstptr=(CAM_PIXEL)valpix;
                acc+=valpix;
	    } END_FOREACH_PIXEL
	}
	break;    
    case CAM_ARITHM_MUL:
	if (params->c1!=0) {
	    if (dest->depth&CAM_DEPTH_SIGN) {
		// Signed multiplication
		valmax>>=1;
		FOREACH_PIXEL {
		    valpix=*(CAM_SIGNED_PIXEL*)src1ptr*(int)*(CAM_SIGNED_PIXEL*)src2ptr;
		    valpix>>=params->c1;
		    // Saturation :
		    if (valpix<-valmax-1) valpix=-valmax-1; 
		    else if (valpix>valmax) valpix=valmax;
		    *dstptr=(CAM_PIXEL)valpix;
		    acc+=valpix;
		} END_FOREACH_PIXEL
	    } else {
		FOREACH_PIXEL {
		    valpix=*src1ptr*(int)*src2ptr;
		    valpix>>=params->c1;
		    // Saturation :
		    if (valpix<0) valpix=0;
		    else if (valpix>valmax) valpix=valmax;
		    *dstptr=(CAM_PIXEL)valpix;
		    acc+=valpix;
		} END_FOREACH_PIXEL
	    }
	} else {
	    if (dest->depth&CAM_DEPTH_SIGN) {
		// Signed multiplication
		valmax>>=1;
		FOREACH_PIXEL {
		    valpix=*(CAM_SIGNED_PIXEL*)src1ptr*(int)*(CAM_SIGNED_PIXEL*)src2ptr;
		    // Saturation :
		    if (valpix<-valmax-1) valpix=-valmax-1; 
		    else if (valpix>valmax) valpix=valmax;
		    *dstptr=(CAM_PIXEL)valpix;
		    acc+=valpix;
		} END_FOREACH_PIXEL
	    } else {
		FOREACH_PIXEL {
		    valpix=*src1ptr*(int)*src2ptr;
		    // Saturation :
		    if (valpix<0) valpix=0;
		    else if (valpix>valmax) valpix=valmax;
		    *dstptr=(CAM_PIXEL)valpix;
		    acc+=valpix;
		} END_FOREACH_PIXEL
	    }
	}
	break;    
    case CAM_ARITHM_ABSDIFF:
	FOREACH_PIXEL {
	    valpix=(int)*src1ptr-(int)*src2ptr;
	    if (valpix<0) valpix=-valpix;
	    *dstptr=valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_WEIGHTED_SUM:
	if (dest->depth&CAM_DEPTH_SIGN) {
	    // Signed multiplication
	    valmax>>=1;
	    FOREACH_PIXEL {
		valpix=(c1*(int)(*(CAM_SIGNED_PIXEL*)src1ptr)+c2*(int)(*(CAM_SIGNED_PIXEL*)src2ptr))>>c3;
		// Saturation :
		if (valpix<-valmax-1) valpix=-valmax-1; 
		else if (valpix>valmax) valpix=valmax;
		*dstptr=(CAM_PIXEL)valpix;
		acc+=valpix;
	    } END_FOREACH_PIXEL
	} else {
	    FOREACH_PIXEL {
		// Weighted sum gives good results only when depth is more than 8 bits
		valpix=(c1*(int)(*src1ptr)+c2*(int)(*src2ptr))>>c3;
		// Saturation :
		if (valpix<0) valpix=0;
		else if (valpix>valmax) valpix=valmax;
		*dstptr=valpix;
		acc+=valpix;
	    } END_FOREACH_PIXEL
	}
	break;
    case CAM_ARITHM_INF:
	FOREACH_PIXEL {
	    if (*src1ptr<*src2ptr) valpix=*src1ptr; else valpix=*src2ptr;
            *dstptr=valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_SUP:
	FOREACH_PIXEL {
	    if (*src1ptr>*src2ptr) valpix=*src1ptr; else valpix=*src2ptr;
            *dstptr=valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_COMP_INF:
	FOREACH_PIXEL {
	    if (*src1ptr<*src2ptr) valpix=c1; else valpix=c2;
            *dstptr=valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_COMP_EQUAL:
	FOREACH_PIXEL {
	    if (*src1ptr==*src2ptr) valpix=c1; else valpix=c2;
            *dstptr=valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_COMP_SUP:
	FOREACH_PIXEL {
	    if (*src1ptr>*src2ptr) valpix=c1; else valpix=c2;
            *dstptr=valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_AND:
	FOREACH_PIXEL {
	    valpix=*src1ptr&*src2ptr;
            *dstptr=valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    case CAM_ARITHM_OR:
	FOREACH_PIXEL {
	    valpix=*src1ptr|*src2ptr;
            *dstptr=valpix;
            acc+=valpix;
	} END_FOREACH_PIXEL
	break;
    default:
        camError("camDyadicArithm","operation not supported");
    }

    camInternalROIPolicyExit(&iROI1);
    camInternalROIPolicyExit(&iROI2);
    return acc;
}

#undef FOREACH_PIXEL
#undef END_FOREACH_PIXEL

int camThreshold(CamImage *source, CamImage *dest, int threshold)
{
    CamArithmParams params;
    params.operation=CAM_ARITHM_THRESHOLD;
    params.c1=threshold;
    params.c2=0;
    params.c3=255;
    return camMonadicArithm(source,dest,&params);
}

int camThresholdInv(CamImage *source, CamImage *dest, int threshold)
{
    CamArithmParams params;
    params.operation=CAM_ARITHM_THRESHOLD;
    params.c1=threshold;
    params.c2=255;
    params.c3=0;
    return camMonadicArithm(source,dest,&params);
}

int camAbs(CamImage *source, CamImage *dest)
{
    CamArithmParams params;
    params.operation=CAM_ARITHM_ABS;
    return camMonadicArithm(source,dest,&params);
}

#else // CAM_GRAYSCALE

#ifdef CAM_INVERSE1U
#define NB_SOURCES 1
int camInverse1U(CamImage *src, CamImage *dest)
#else
#undef NB_SOURCES
#define NB_SOURCES 2
int camDyadicArithm1U(CamImage *source1, CamImage *source2, CamImage *dest, CamArithmParams *params)
#endif
{
    int i,x,y;
    int width,height;
    CamImage *source[NB_SOURCES];
    CAM_BIT_BLOCK result;
    CAM_BIT_BLOCK *srcptr[NB_SOURCES],*cpsrcptr[NB_SOURCES];
    CAM_BIT_BLOCK src_bit_block[NB_SOURCES],src_loaded_block[NB_SOURCES];
    int src_bit_offset[NB_SOURCES];

    CAM_BIT_BLOCK *dstptr,*cpdstptr;
    CAM_BIT_BLOCK dst_bit_block;
    int dst_bit_offset;

    // Algorithm initialization
#ifdef CAM_INVERSE1U
    source[0]=src;

    // ROI (Region Of Interest) management
    if (src->roi) {
	width=src->roi->width;
	height=src->roi->height;
    } else {
	width=src->width;
	height=src->height;
    }
#else
    
    CamInternalROIPolicyStruct iROI1,iROI2;

    // ROI (Region Of Interest) management
    CAM_CHECK(camDyadicArithm,camInternalROIPolicy(source1, dest, &iROI1, 0));
    CAM_CHECK_ARGS(camDyadicArithm,iROI1.nChannels==1);
    CAM_CHECK(camDyadicArithm,camInternalROIPolicy(source2, dest, &iROI2, 0));
    CAM_CHECK_ARGS(camDyadicArithm,iROI2.nChannels==1);
    CAM_CHECK_ARGS2(camDyadicArithm,((iROI1.srcroi.width==iROI2.srcroi.width)&&(iROI1.srcroi.height==iROI2.srcroi.height)),"can't handle these ROIs");

    source[0]=source1;
    source[1]=source2;

    width=iROI1.srcroi.width;
    height=iROI1.srcroi.height;
#endif

    for (i=0;i<NB_SOURCES;i++) {
	CAM_CHECK_ARGS(camDyadicArithm,source[i]->depth==CAM_DEPTH_1U);
    }
    CAM_CHECK_ARGS(camDyadicArithm,dest->depth==CAM_DEPTH_1U);

    for (i=0;i<NB_SOURCES;i++) {
	if (source[i]->roi) {
	    src_bit_offset[i]=source[i]->roi->xOffset%CAM_BIT_BLOCK_SIZE;
	    srcptr[i]=(CAM_BIT_BLOCK*)(source[i]->imageData+source[i]->roi->yOffset*source[i]->widthStep)+source[i]->roi->xOffset/CAM_BIT_BLOCK_SIZE;
	} else {
	    src_bit_offset[i]=0;
	    srcptr[i]=(CAM_BIT_BLOCK*)(source[i]->imageData);
	}
    }

    if (dest->roi) {
        dst_bit_offset=dest->roi->xOffset%CAM_BIT_BLOCK_SIZE;
        dstptr=(CAM_BIT_BLOCK*)(dest->imageData+dest->roi->yOffset*dest->widthStep)+dest->roi->xOffset/CAM_BIT_BLOCK_SIZE;
    } else {
	dst_bit_offset=0;
	dstptr=(CAM_BIT_BLOCK*)(dest->imageData);
    }

    // For all the lines
    for (y=0;y<height;y++) {

	for (i=0;i<NB_SOURCES;i++) {
	    // Initialize pointers
	    cpsrcptr[i]=srcptr[i];
	    if (src_bit_offset[i]) { // Is this image aligned?
		// Loads some data from source images
		src_loaded_block[i]=*(srcptr[i]++);
		// Little endianess management
		src_loaded_block[i]=CAM_BIT_BLOCK_SWAP(src_loaded_block[i]);
	    }
	}
	// Destination management
	cpdstptr=dstptr;
	if (dst_bit_offset) { // Aligned destination ?
	    dst_bit_block=(*dstptr); // Load data from the destination picture
	    dst_bit_block=CAM_BIT_BLOCK_SWAP(dst_bit_block);
	    dst_bit_block&=~((
		((CAM_BIT_BLOCK)1)<<
		(CAM_BIT_BLOCK_SIZE-dst_bit_offset)
	    )-1);
	}

	for (x=0;x<(width>>CAM_BIT_BLOCK_SIZE_SHIFT);x++) {
	    // Reconstruct the source bit blocks
	    for (i=0;i<NB_SOURCES;i++) {
		if (src_bit_offset[i]) { // Is this image aligned
		    // No, so we need to load another part of image
		    src_bit_block[i]=src_loaded_block[i]<<src_bit_offset[i];
		    src_loaded_block[i]=*(srcptr[i]++);
		    src_loaded_block[i]=CAM_BIT_BLOCK_SWAP(src_loaded_block[i]);
		    src_bit_block[i]|=src_loaded_block[i]>>(CAM_BIT_BLOCK_SIZE-src_bit_offset[i]);
		    src_bit_block[i]=CAM_BIT_BLOCK_SWAP(src_bit_block[i]);
		} else {
		    // Yes, so it's more simple
		    src_bit_block[i]=*(srcptr[i]++);
		}
	    }
	    
	    // Compute the result of the operation
#ifdef CAM_INVERSE1U
	    result=~src_bit_block[0];
#else
	    if (params->operation==CAM_ARITHM_AND) {
		result=src_bit_block[0]&src_bit_block[1];
	    } else {
		result=src_bit_block[0]|src_bit_block[1];
	    }
#endif
	    
	    // Store the result in the destination
	    if (dst_bit_offset) { // Aligned destination ?
		// No ! Let's process it
		result=CAM_BIT_BLOCK_SWAP(result);
		dst_bit_block|=(result>>dst_bit_offset);
		*(dstptr++)=CAM_BIT_BLOCK_SWAP(dst_bit_block);
		dst_bit_block=result<<(CAM_BIT_BLOCK_SIZE-dst_bit_offset);
	    } else {
		*(dstptr++)=result;
	    }
	}

	// What is the remaining number of pixels to process?
	if (width&(CAM_BIT_BLOCK_SIZE-1)) {
	    // There are pixels remaining to be processed
	    // Reconstruct the source bit blocks
	    for (i=0;i<NB_SOURCES;i++) {
		if (src_bit_offset[i]) { // Is this image aligned
		    // No, so we need to load another part of image
		    src_bit_block[i]=src_loaded_block[i]<<src_bit_offset[i];
		    src_loaded_block[i]=*(srcptr[i]++);
		    src_loaded_block[i]=CAM_BIT_BLOCK_SWAP(src_loaded_block[i]);
		    src_bit_block[i]|=src_loaded_block[i]>>(CAM_BIT_BLOCK_SIZE-src_bit_offset[i]);
		    src_bit_block[i]=CAM_BIT_BLOCK_SWAP(src_bit_block[i]);
		} else {
		    // Yes, so it's more simple
		    src_bit_block[i]=*(srcptr[i]++);
		}
	    }
	    
	    // Compute the result of the operation
#ifdef CAM_INVERSE1U
	    result=~src_bit_block[0];
#else
	    if (params->operation==CAM_ARITHM_AND) {
		result=src_bit_block[0]&src_bit_block[1];
	    } else {
		result=src_bit_block[0]|src_bit_block[1];
	    }
#endif
	    
	    i=(width&(CAM_BIT_BLOCK_SIZE-1));
	    result=CAM_BIT_BLOCK_SWAP(result);
	    result&=~((
		    ((CAM_BIT_BLOCK)1)<<
		    (CAM_BIT_BLOCK_SIZE-i)
		)-1);
	    
	    // Remaining pixels :
	    if (dst_bit_offset+i>CAM_BIT_BLOCK_SIZE) {
		// We can write an additional full block of result
		dst_bit_block|=(result>>dst_bit_offset);
		*(dstptr++)=CAM_BIT_BLOCK_SWAP(dst_bit_block);
		dst_bit_block=result<<(CAM_BIT_BLOCK_SIZE-dst_bit_offset);		
		i=dst_bit_offset+i-CAM_BIT_BLOCK_SIZE;
	    } else {
		if (dst_bit_offset) {
		    dst_bit_block|=(result>>dst_bit_offset);
		    i+=dst_bit_offset;
		} else {
		    dst_bit_block=result;
		}
	    }
	    // And let's store the very last block (of size i)
	    result=*dstptr; // Load the remaining bits
	    result=CAM_BIT_BLOCK_SWAP(result);
	    dst_bit_block|=result&((
		((CAM_BIT_BLOCK)1)<<
		(CAM_BIT_BLOCK_SIZE-i)
		)-1);
	    *dstptr=CAM_BIT_BLOCK_SWAP(dst_bit_block); // Store the result
	} else {
	    // There is no pixel remaining to be processed
	    // Store the destination result
	    if (dst_bit_offset) {
	 	result=*dstptr; // Load the remaining bits
		result=CAM_BIT_BLOCK_SWAP(result);
		dst_bit_block|=result&((
		    ((CAM_BIT_BLOCK)1)<<
		    (CAM_BIT_BLOCK_SIZE-dst_bit_offset)
		)-1);
		*dstptr=CAM_BIT_BLOCK_SWAP(dst_bit_block); // Store the result
	    } // Else nothing is needed
	}

	// Go to next line
	for (i=0;i<NB_SOURCES;i++) {
	    srcptr[i]=(CAM_BIT_BLOCK*)(((char*)cpsrcptr[i])+source[i]->widthStep);
	}
	dstptr=(CAM_BIT_BLOCK*)(((char*)cpdstptr)+dest->widthStep);
    }
    return 1;
}
#endif // CAM_GRAYSCALE

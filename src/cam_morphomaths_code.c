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

#include "string.h"

/* Fundamental Morphological Mathematics Kernel
 * C code */

#ifndef CAM_MM_ONE_OP
#define CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
#endif

// Prototype for binary morphology kernel
int camMorphoMaths1U(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel);

// Grey level morphology kernel
int camMorphoMaths(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    int i,j,x,y,xp,yp,ntb,nlb;
    int width,height;
    int top,left;
    int value;
    int acc=0;

#ifndef CAM_MM_ONE_OP
    int doErosion=0,doDilation=0;
    int srcpix[3];
    int valmax;
#endif

#ifdef CAM_MM_DO_EROSION
#ifdef CAM_OPT_P4
    int valero[2];
#define eroded valero[0]
#else
    int eroded;
#endif // CAM_OPT_P4
    int erosion;
#endif // CAM_MM_DO_EROSION

#ifdef CAM_MM_DO_DILATION
#ifdef CAM_OPT_P4
    int valdil[2];
#define dilated valdil[0]
#else
    int dilated;
#endif // CAM_OPT_P4
    int dilation;
#endif // CAM_MM_DO_DILATION

    unsigned CAM_PIXEL *srcptr,*dstptr,*tmpptr,*cpsrcptr,*cpdstptr,*lptr;
    unsigned CAM_PIXEL *linesPtr[CAM_MM_NEIGHB];
    CAM_PIXEL linesBuffer[CAM_MM_NEIGHB][CAM_MAX_SCANLINE+CAM_MM_NEIGHB-1];

    DECLARE_MASK_MANAGEMENT;
    CamInternalROIPolicyStruct iROI;
    
    CAM_CHECK_ARGS2(CamMorphoMathsKernel,source->imageData!=NULL,"Source image is not allocated");
    // ROI (Region Of Interest) management
    CAM_CHECK(CamMorphoMathsKernel,camInternalROIPolicy(source, dest, &iROI, 1));
    CAM_CHECK_ARGS(CamMorphoMathsKernel,iROI.nChannels==1);
    CAM_CHECK_ARGS(CamMorphoMathsKernel,(source->depth==dest->depth));
    // Binary images management
    if (source->depth==CAM_DEPTH_1U) {
	return camMorphoMaths1U(source,dest,kernel);
    }

    CAM_CHECK_ARGS(CamMorphoMathsKernel,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(CamMorphoMathsKernel,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(CamMorphoMathsKernel,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(CamMorphoMathsKernel,(dest->depth&CAM_DEPTH_MASK)>=8);
    
#ifndef CAM_MM_ONE_OP    
    // Saturation management
    valmax=(1<<(dest->depth&CAM_DEPTH_MASK))-1;
#endif

    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    if (source->roi) {
        left=iROI.srcroi.xOffset;
        top=iROI.srcroi.yOffset;
        nlb=left; if (nlb>CAM_MM_NEIGHB/2) nlb=CAM_MM_NEIGHB/2;
	ntb=top; if (ntb>CAM_MM_NEIGHB/2) ntb=CAM_MM_NEIGHB/2;
        srcptr=(CAM_PIXEL*)(source->imageData+iROI.srcchoffset+(top-ntb)*source->widthStep)+(left-nlb);
    } else {
        srcptr=(CAM_PIXEL*)(source->imageData+iROI.srcchoffset);
        left=0;
        top=0;
    }
    dstptr=(CAM_PIXEL*)iROI.dstptr;
    CAM_CHECK_ARGS(CamMorphoMathsKernel,(width>CAM_MM_NEIGHB/2));
    CAM_CHECK_ARGS(CamMorphoMathsKernel,(height>CAM_MM_NEIGHB/2));    
	
    // Mask management
    INIT_MASK_MANAGEMENT;

    // Initialize algorithm
    for (i=0;i<CAM_MM_NEIGHB;i++) {
	linesPtr[i]=linesBuffer[i];
    }
    
    // Analyze the parameters
#ifndef CAM_MM_ONE_OP
    switch (kernel->operation) {
    case CAM_MM_SUBSTRACTION:
	if (kernel->source1==CAM_MM_DILATED) doDilation=1;
	else if (kernel->source1==CAM_MM_ERODED) doErosion=1;
	if (kernel->source2==CAM_MM_DILATED) doDilation=1;
	else if (kernel->source2==CAM_MM_ERODED) doErosion=1;
	break;
    case CAM_MM_MULTIPLEX:
	if (kernel->source1==CAM_MM_DILATED) doDilation=1;
	else if (kernel->source1==CAM_MM_ERODED) doErosion=1;
	break;
    case CAM_MM_THINNING:
    case CAM_MM_THICKENING:
	doDilation=1;
	doErosion=1;
	break;
    default:
	camError("CamMorphoMathsKernel","Bad operation parameter");
    };
#endif
    
    // Initialize neighbourhood
    
    // Fill the top lines
    for (y=0;y+top<CAM_MM_NEIGHB/2;y++) {
        // Out of frame : fill with border color
        if (source->borderMode[CAM_SIDE_TOP_INDEX]==CAM_BORDER_REPLICATE) {
            cpsrcptr=srcptr;
            for (x=0;x<CAM_MM_NEIGHB/2;x++) {
                linesPtr[y][x]=*srcptr;
            }
            for (;x<width+CAM_MM_NEIGHB/2;x++) {
                linesPtr[y][x]=*srcptr;
                srcptr+=iROI.srcinc;
            }
            for (;x<width+CAM_MM_NEIGHB-1;x++) {
                linesPtr[y][x]=*(srcptr-iROI.srcinc);
            }
            srcptr=cpsrcptr;
        } else {
            for (x=0;x<width+CAM_MM_NEIGHB-1;x++) {
                linesPtr[y][x]=source->borderConst[CAM_SIDE_TOP_INDEX];
            }
        }
    }
    
    // Fill the next lines with image pixels
    for (;y<CAM_MM_NEIGHB-1;y++) {
        cpsrcptr=srcptr;
        for (x=0;x<CAM_MM_NEIGHB/2;x++) {
            if (left+x-CAM_MM_NEIGHB/2<0) {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_LEFT_INDEX]==CAM_BORDER_REPLICATE) {
                    linesPtr[y][x]=*srcptr;
                } else {
                    linesPtr[y][x]=source->borderConst[CAM_SIDE_LEFT_INDEX];
                }
            } else {
                // Get border pixels from the source frame
                linesPtr[y][x]=*srcptr;
                srcptr+=iROI.srcinc;
            }
        }
        for (;x<width+CAM_MM_NEIGHB/2;x++) {
            linesPtr[y][x]=*srcptr;
            srcptr+=iROI.srcinc;
        }
        for (;x<width+CAM_MM_NEIGHB-1;x++) {
            if (left+x-CAM_MM_NEIGHB/2<source->width) {
                // Get border pixels from the source frame
                linesPtr[y][x]=*srcptr;
                srcptr+=iROI.srcinc;
            } else {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_RIGHT_INDEX]==CAM_BORDER_REPLICATE) {
                    linesPtr[y][x]=*(srcptr-iROI.srcinc);
                } else {
                    linesPtr[y][x]=source->borderConst[CAM_SIDE_RIGHT_INDEX];
                }
            }
        }
        srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
    }
    
    // Now process the whole image
    // This is the main loop
    for (y=0;y<height;y++) {
	cpsrcptr=srcptr;
        cpdstptr=dstptr;

        // Start a new line
        // Have we reached the bottom of the frame ?
        if (top+y+CAM_MM_NEIGHB/2>=source->height) {
            if (source->borderMode[CAM_SIDE_BOTTOM_INDEX]==CAM_BORDER_REPLICATE) {
                // Go up one line (in order to stay on the last line)
                cpsrcptr=(CAM_PIXEL*)(((char*)cpsrcptr)-source->widthStep);
                srcptr=cpsrcptr;
                for (x=0;x<CAM_MM_NEIGHB/2;x++) {
                    linesPtr[CAM_MM_NEIGHB-1][x]=*srcptr;
                }
                for (;x<width+CAM_MM_NEIGHB/2;x++) {
                    linesPtr[CAM_MM_NEIGHB-1][x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
                for (;x<width+CAM_MM_NEIGHB-1;x++) {
                    linesPtr[CAM_MM_NEIGHB-1][x]=*(srcptr-iROI.srcinc);
                }
            } else {
                for (x=0;x<width+CAM_MM_NEIGHB-1;x++) {
                    linesPtr[CAM_MM_NEIGHB-1][x]=source->borderConst[CAM_SIDE_BOTTOM_INDEX];
                }
            }
        } else {
            for (x=0;x<CAM_MM_NEIGHB/2;x++) {
                if (left+x-CAM_MM_NEIGHB/2<0) {
                    // Out of frame : fill with border color
                    if (source->borderMode[CAM_SIDE_LEFT_INDEX]==CAM_BORDER_REPLICATE) {
                        linesPtr[CAM_MM_NEIGHB-1][x]=*srcptr;
                    } else {
                        linesPtr[CAM_MM_NEIGHB-1][x]=source->borderConst[CAM_SIDE_LEFT_INDEX];
                    }
                } else {
                    // Get border pixels from the source frame
                    linesPtr[CAM_MM_NEIGHB-1][x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
            }

            // Fast transfer with memcpy
            if (iROI.srcinc==1) {
                memcpy(&linesPtr[CAM_MM_NEIGHB-1][x],srcptr,(source->width-left+CAM_MM_NEIGHB/2-x)<<(sizeof(CAM_PIXEL)-1));
                x=source->width-left+CAM_MM_NEIGHB/2;
            } else {
                for (;x<source->width-left+CAM_MM_NEIGHB/2;x++) {
                    // Get a pixel from the source frame
                    linesPtr[CAM_MM_NEIGHB-1][x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
            }
            for (;x<width+CAM_MM_NEIGHB-1;x++) {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_RIGHT_INDEX]==CAM_BORDER_REPLICATE) {
                    linesPtr[CAM_MM_NEIGHB-1][x]=*(srcptr+(source->width-1)*iROI.srcinc);
                } else {
                    linesPtr[CAM_MM_NEIGHB-1][x]=source->borderConst[CAM_SIDE_RIGHT_INDEX];
                }
            }
        }

	BEGIN_MASK_MANAGEMENT(dstptr=cpdstptr+startx*iROI.dstinc;);
            
            // Process all the pixels in the line
            for (x=startx+CAM_MM_NEIGHB-1;x<endx+CAM_MM_NEIGHB-1;x++) {
                
                // Now, let's compute the maximum and minimum value on the neighbourhood
#ifdef CAM_MM_DO_EROSION
                eroded=0xffff;
#endif
#ifdef CAM_MM_DO_DILATION
                dilated=0;
#endif
                i=x-CAM_MM_NEIGHB+1;
                for (yp=0,j=0;yp<CAM_MM_NEIGHB;yp++) {
                    lptr=linesPtr[yp];
                    for (xp=0;xp<CAM_MM_NEIGHB;xp++,j++) {
                        value=lptr[i+xp]; // linesPtr[yp][i+xp];
#ifndef CAM_MM_ONE_OP
                        if (doErosion)
#endif
#ifdef CAM_MM_DO_EROSION
                        {
                            erosion=kernel->erosionStructElt[yp][xp];
#ifdef CAM_OPT_P4
                            valero[1]=value;
                            valero[0]=valero[erosion&(valero[1]<valero[0])];
#else
                            eroded=(erosion&(value<eroded))?value:eroded;
#endif
                        }
#endif // CAM_MM_DO_EROSION
#ifndef CAM_MM_ONE_OP
                        if (doDilation)
#endif
#ifdef CAM_MM_DO_DILATION
                        {
                            dilation=kernel->dilationStructElt[yp][xp];
#ifdef CAM_OPT_P4
                            valdil[1]=value;
                            valdil[0]=valdil[dilation&(valdil[1]>valdil[0])];
#else
                            dilated=(dilation&(value>dilated))?value:dilated;
#endif
                        }
#endif // CAM_MM_DO_DILATION
                    }
                }
                
#ifdef CAM_MM_ONE_OP
#ifdef CAM_MM_DO_EROSION
                *dstptr=eroded;
                acc+=eroded;
#else
                *dstptr=dilated;
                acc+=dilated;
#endif
                dstptr+=iROI.dstinc;
#else
                // Get the original value of the pixel
                srcpix[CAM_MM_ORIGINAL]=linesPtr[CAM_MM_NEIGHB>>1][i+(CAM_MM_NEIGHB>>1)];
                
                // We've got the original, dilated and eroded value of pixels
                srcpix[CAM_MM_DILATED]=dilated;
                srcpix[CAM_MM_ERODED]=eroded;
                
                // Let's compute the final result
                switch (kernel->operation) {
                case CAM_MM_SUBSTRACTION:
                    value=srcpix[kernel->source1]-srcpix[kernel->source2];
                    // Saturation on the result
                    if (value<0) value=0;
                    else if (value>valmax) value=valmax;
                    break;
                case CAM_MM_MULTIPLEX:
                    value=srcpix[kernel->source1];
                    break;
                case CAM_MM_THINNING:
                    value=srcpix[CAM_MM_ORIGINAL];
                    if ((value<eroded)&&(value>=dilated)) {
                        value=eroded;
                    }
                    break;
                case CAM_MM_THICKENING:
                    value=srcpix[CAM_MM_ORIGINAL];
                    if ((value<=eroded)&&(value>dilated)) {
                        value=dilated;
                    }
                    break;
                }
                
                // Store the result in destination image
                *dstptr=value;
                dstptr+=iROI.dstinc;
                acc+=value;
#endif
            }

	END_MASK_MANAGEMENT;

	// Go to next line
	srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);
	
	// Reset neighborhood line pointers
	tmpptr=linesPtr[0];
	for (i=0;i<CAM_MM_NEIGHB-1;i++) {
	    linesPtr[i]=linesPtr[i+1];
	}
	linesPtr[CAM_MM_NEIGHB-1]=tmpptr;
    }

    camInternalROIPolicyExit(&iROI);
    return acc;
}

#undef eroded
#undef dilated

#ifndef GRAY_SCALE_ONLY

int camMorphoMaths1U(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel)
{
    int i,j,x,y,yp;
    int nlb,nrb,ntb; // # of left border pixels, right border pixels, top border pixels
    int xbr;
    int width,height;
    int top,left;
#ifndef CAM_MM_ONE_OP
    int value;
    int doErosion=0,doDilation=0;
    int srcpix[3];
    int idempotence=1;
#endif
    unsigned char *linesPtr[CAM_MM_NEIGHB],*tmpptr;
    unsigned char linesBuffer[CAM_MM_NEIGHB][CAM_MAX_SCANLINE/8+2*sizeof(CAM_BIT_BLOCK)];
#define NEIGHBORHOOD_SIZE ((CAM_MM_NEIGHB*16)/CAM_BIT_BLOCK_SIZE+1)
    CAM_BIT_BLOCK fillValue[2],neighborhood[NEIGHBORHOOD_SIZE];
#ifdef CAM_MM_DO_EROSION
    CAM_BIT_BLOCK eroded,maskErosion[NEIGHBORHOOD_SIZE];
#endif
#ifdef CAM_MM_DO_DILATION
    CAM_BIT_BLOCK dilated,maskDilation[NEIGHBORHOOD_SIZE];
#endif

    CAM_BIT_BLOCK *srcptr,*cpsrcptr;
    int src_sol_offset,src_bit_offset,src_bit_loaded;
    CAM_BIT_BLOCK *dstptr,*cpdstptr;
    CAM_BIT_BLOCK dst_bit_block,dst_eol_block;
    int dst_sol_offset,dst_bit_offset;

    // ROI (Region Of Interest) management
    if (source->roi) {
	width=source->roi->width;
	height=source->roi->height;
	left=source->roi->xOffset;
	top=source->roi->yOffset;
        nlb=left; if (nlb>CAM_MM_NEIGHB/2) nlb=CAM_MM_NEIGHB/2;
	ntb=top; if (ntb>CAM_MM_NEIGHB/2) ntb=CAM_MM_NEIGHB/2;
	nrb=source->width-(left+width); if (nrb>CAM_MM_NEIGHB/2) nrb=CAM_MM_NEIGHB/2;
	srcptr=(CAM_BIT_BLOCK*)(source->imageData+(top-ntb)*source->widthStep)+(left-nlb)/CAM_BIT_BLOCK_SIZE;
    } else {
	srcptr=(CAM_BIT_BLOCK*)source->imageData;
	width=source->width;
	height=source->height;
	left=0; top=0;
	nlb=0; nrb=0; ntb=0;
    }
    if (dest->roi) {
	dst_sol_offset=dest->roi->xOffset%CAM_BIT_BLOCK_SIZE;
	dstptr=(CAM_BIT_BLOCK*)(dest->imageData+dest->roi->yOffset*dest->widthStep)+(dest->roi->xOffset/CAM_BIT_BLOCK_SIZE);
    } else {
	dst_sol_offset=0;
	dstptr=(CAM_BIT_BLOCK*)dest->imageData;
    }
	
    // Initialize algorithm
    for (i=0;i<CAM_MM_NEIGHB;i++) {
	linesPtr[i]=linesBuffer[i];
    }

    // Analyze the parameters
#ifndef CAM_MM_ONE_OP
    switch (kernel->operation) {
    case CAM_MM_SUBSTRACTION:
	if (kernel->source1==CAM_MM_DILATED) doDilation=1;
	else if (kernel->source1==CAM_MM_ERODED) doErosion=1;
	if (kernel->source2==CAM_MM_DILATED) doDilation=1;
	else if (kernel->source2==CAM_MM_ERODED) doErosion=1;
	break;
    case CAM_MM_MULTIPLEX:
	if (kernel->source1==CAM_MM_DILATED) doDilation=1;
	else if (kernel->source1==CAM_MM_ERODED) doErosion=1;
	break;
    case CAM_MM_THINNING:
    case CAM_MM_THICKENING:
	doDilation=1;
	doErosion=1;
	break;
    default:
	camError("CamMorphoMathsKernel","Bad operation parameter in call to CamMorphoMathsKernel");
    };
#endif
    
    // Initialize neighbourhood
    fillValue[0]=0;
    fillValue[1]=~fillValue[0];
#ifdef CAM_MM_DO_EROSION
    // Fill the bit mask for erosion
    for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
	maskErosion[i]=-1; // Fill with ones
    }
    tmpptr=(unsigned char*)maskErosion;
    for (y=0;y<CAM_MM_NEIGHB;y++) {
	for (x=0;x<CAM_MM_NEIGHB;x++) {
	    if (kernel->erosionStructElt[y][x]==1) {
		tmpptr[0]&=0xff-(0x80>>x);
	    }
	}
	tmpptr+=2;
    }
    for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
	maskErosion[i]=CAM_BIT_BLOCK_SWAP(maskErosion[i]); // Bit block swap for little endian architecture
    }
#endif
#ifdef CAM_MM_DO_DILATION
    // Fill the bit mask for dilation
    for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
	maskDilation[i]=0; // Fill with zeros
    }
    tmpptr=(unsigned char*)maskDilation;
    for (y=0;y<CAM_MM_NEIGHB;y++) {
	for (x=0;x<CAM_MM_NEIGHB;x++) {
	    if (kernel->dilationStructElt[y][x]==1) {
		tmpptr[0]|=(0x80>>x);
	    }
	}
	tmpptr+=2;
    }
    for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
	maskDilation[i]=CAM_BIT_BLOCK_SWAP(maskDilation[i]); // Bit block swap for little endian architecture
    }
#endif

    // Fill the top lines
    for (y=0;y+top<CAM_MM_NEIGHB/2;y++) {
	// Out of frame : fill with border color
	for (x=0;x<(int)((width+CAM_MM_NEIGHB)>>CAM_BIT_BLOCK_SIZE_SHIFT)+2;x++) {
	    ((CAM_BIT_BLOCK*)linesPtr[y])[x]=fillValue[source->borderConst[CAM_SIDE_TOP_INDEX]];
	}
    }

    // Fill the next lines with image pixels
    for (;y<CAM_MM_NEIGHB-1;y++) {
	cpsrcptr=srcptr;
	// One bit block of border color 
	*((CAM_BIT_BLOCK*)linesPtr[y])=fillValue[source->borderConst[CAM_SIDE_LEFT_INDEX]];
	// Get the data from the picture, bit block by bit block
	for (x=1;x<=(int)((width+nlb+nrb)>>CAM_BIT_BLOCK_SIZE_SHIFT)+1;x++) {
	    ((CAM_BIT_BLOCK*)linesPtr[y])[x]=*(srcptr++);
	}
	// Some bits of right border color in any
	xbr=CAM_BIT_BLOCK_SIZE+((left-nlb)%CAM_BIT_BLOCK_SIZE)+width+nlb;
	if (source->borderConst[CAM_SIDE_RIGHT_INDEX]) {
	    for (i=0;i<CAM_MM_NEIGHB/2-nrb;i++) {
	        linesPtr[y][(xbr+i)>>3]|=((unsigned char)1<<(7-((xbr+i)&7)));
	    }
	} else {
	    for (i=0;i<CAM_MM_NEIGHB/2-nrb;i++) {
	        linesPtr[y][(xbr+i)>>3]&=~((unsigned char)1<<(7-((xbr+i)&7)));
	    }
	}	
        // Go to next line
	srcptr=(CAM_BIT_BLOCK*)(((char*)cpsrcptr)+dest->widthStep);
    }

    // Now process the whole image
    src_sol_offset=CAM_BIT_BLOCK_SIZE+left%CAM_BIT_BLOCK_SIZE-CAM_MM_NEIGHB/2;

    // This is the main loop
    for (y=0;y<height;y++) {
        src_bit_offset=src_sol_offset;
	
	// Init destination pointers
	dst_bit_block=*dstptr; // Get original 32 pixels from the destination image
	dst_bit_offset=dst_sol_offset;
	dst_bit_block=CAM_BIT_BLOCK_SWAP(dst_bit_block);
	dst_bit_block>>=(CAM_BIT_BLOCK_SIZE-dst_bit_offset);
	cpdstptr=dstptr;

	// Start a new line
	// Have we reached the bottom of the frame ?
	if (top+y+CAM_MM_NEIGHB/2<source->height) {
	    cpsrcptr=srcptr;
	    // No! Let's put the left border color bits
	    *((CAM_BIT_BLOCK*)linesPtr[CAM_MM_NEIGHB-1])=fillValue[source->borderConst[CAM_SIDE_LEFT_INDEX]];
	    // Load a bit block from source image memory
	    ((CAM_BIT_BLOCK*)linesPtr[CAM_MM_NEIGHB-1])[1]=*(srcptr++);
	    src_bit_loaded=2*CAM_BIT_BLOCK_SIZE;
	    // Check for the right border pixels
	    if (src_bit_loaded>xbr) {
		// We have reached a number of right border pixels
		// So, let's put them in the line buffer
		if (source->borderConst[CAM_SIDE_RIGHT_INDEX]) {
		    for (i=0;i<CAM_MM_NEIGHB/2-nrb;i++) {
			linesPtr[CAM_MM_NEIGHB-1][(xbr+i)>>3]|=((unsigned char)1<<(7-((xbr+i)&7)));
		    }
		} else {
		    for (i=0;i<CAM_MM_NEIGHB/2-nrb;i++) {
			linesPtr[CAM_MM_NEIGHB-1][(xbr+i)>>3]&=~((unsigned char)1<<(7-((xbr+i)&7)));
		    }
		}			
	    }
	} else {
	    // Out of frame : fill with border color
	    for (x=0;x<(int)((width+CAM_MM_NEIGHB)>>CAM_BIT_BLOCK_SIZE_SHIFT)+2;x++) {
		((CAM_BIT_BLOCK*)linesPtr[CAM_MM_NEIGHB-1])[x]=fillValue[source->borderConst[CAM_SIDE_BOTTOM_INDEX]];
	    }
	    src_bit_loaded=x<<CAM_BIT_BLOCK_SIZE_SHIFT;
	}

	// Let's now construct the neighborhood in some bit blocks
	for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
	    neighborhood[i]=0;
	}
	i=src_bit_offset>>3; // /8 -> byte boundary
	for (yp=0;yp<CAM_MM_NEIGHB;yp++) {
	    // 16-bits transfer
	    ((unsigned char*)neighborhood)[(yp<<1)+0]=linesPtr[yp][i+0];
	    ((unsigned char*)neighborhood)[(yp<<1)+1]=linesPtr[yp][i+1];
	}
        for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
    	    neighborhood[i]=CAM_BIT_BLOCK_SWAP(neighborhood[i]); // Bit block swap for little endian architecture
	}
        // Great. Now, let's move the neighborhood to align it to src_bit_offset
        // Could be optimized in assembler, not in C (shift through carry)
	for (j=0;j<(src_bit_offset&7);j++) {
	    for (i=0;i<NEIGHBORHOOD_SIZE-1;i++) {
		neighborhood[i]<<=1;
		neighborhood[i]|=(neighborhood[i+1]>>(CAM_BIT_BLOCK_SIZE-1));
	    }
	    neighborhood[NEIGHBORHOOD_SIZE-1]<<=1;
	}
    
	// Process all the pixels in the line
	for (x=0;x<width;x++,src_bit_offset++) {
	    // Do we need to retrieve some data from the source image ?
	    if (((src_bit_offset&0xfff8)+16)>src_bit_loaded) { // (x&0xfff8) == ((x/8)*8)
	    //if (src_bit_offset+CAM_MM_NEIGHB>src_bit_loaded) {
		// If we load some data, will we have reached some right border pixels?
		if ((int)(src_bit_loaded+CAM_BIT_BLOCK_SIZE)<xbr) {
		    // No, let's load some data from image memory
		    ((CAM_BIT_BLOCK*)linesPtr[CAM_MM_NEIGHB-1])[src_bit_loaded>>CAM_BIT_BLOCK_SIZE_SHIFT]=*(srcptr++);
		} else {
		    // Well, the situation is more complicated
		    // Can we retrieve some data from source image ?
		    // xbr+nrb represents the last pixel to be reached in the source image
		    if (src_bit_loaded<xbr+nrb) {
			((CAM_BIT_BLOCK*)linesPtr[CAM_MM_NEIGHB-1])[src_bit_loaded>>CAM_BIT_BLOCK_SIZE_SHIFT]=*(srcptr++);
		    }
		    // We might reach a number of right border pixels
		    // So, let's put them in the line buffer
		    if (source->borderConst[CAM_SIDE_RIGHT_INDEX]) {
			for (i=0;i<CAM_MM_NEIGHB/2-nrb;i++) {
			    linesPtr[CAM_MM_NEIGHB-1][(xbr+i)>>3]|=((unsigned char)1<<(7-((xbr+i)&7)));
			}
		    } else {
			for (i=0;i<CAM_MM_NEIGHB/2-nrb;i++) {
			    linesPtr[CAM_MM_NEIGHB-1][(xbr+i)>>3]&=~((unsigned char)1<<(7-((xbr+i)&7)));
			}
		    }	
		}
		src_bit_loaded+=CAM_BIT_BLOCK_SIZE;
	    }
	    	    
	    // Well, let's now reconstruct the neighborhood in some bit blocks
	    if ((src_bit_offset&7)==0) { // If we are at a byte boundary
		for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
		    neighborhood[i]=0;
		}
		i=src_bit_offset>>3; // /8 -> byte boundary
		for (yp=0;yp<CAM_MM_NEIGHB;yp++) {
		    // 16-bits transfer
		    ((unsigned char*)neighborhood)[(yp<<1)+0]=linesPtr[yp][i+0];
		    ((unsigned char*)neighborhood)[(yp<<1)+1]=linesPtr[yp][i+1];
		}
		for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
		    neighborhood[i]=CAM_BIT_BLOCK_SWAP(neighborhood[i]); // Bit block swap for little endian architecture
		}
	    }

	    // OK. We've got the neighborhood.
	    // Let's compute the erosion
#ifdef CAM_MM_DO_EROSION
#ifndef CAM_MM_ONE_OP
	    if (doErosion)
#endif
	    {
		eroded=-1; // Fill with ones
		for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
		    eroded&=neighborhood[i]|maskErosion[i];
		}
	    }
	    eroded=(~eroded)?0:1;
#endif
	    // Let's compute the dilation
#ifdef CAM_MM_DO_DILATION
#ifndef CAM_MM_ONE_OP
	    if (doDilation)
#endif
	    {
		dilated=0;
		for (i=0;i<NEIGHBORHOOD_SIZE;i++) {
		    dilated|=neighborhood[i]&maskDilation[i];
		}
	    }
	    dilated=(dilated)?1:0;
#endif
	    // Great. Now, let's move the neighborhood
	    // Could be optimized in assembler, not in C (shift through carry)
	    for (i=0;i<NEIGHBORHOOD_SIZE-1;i++) {
		neighborhood[i]<<=1;
		neighborhood[i]|=(neighborhood[i+1]>>(CAM_BIT_BLOCK_SIZE-1));
	    }
	    neighborhood[NEIGHBORHOOD_SIZE-1]<<=1;
	    	    
#ifdef CAM_MM_ONE_OP
#ifdef CAM_MM_DO_EROSION
	    // Store the result in destination image
	    dst_bit_block<<=1;
	    dst_bit_block|=eroded;
	    dst_bit_offset++;
	    if (dst_bit_offset==CAM_BIT_BLOCK_SIZE) {
		dst_bit_block=CAM_BIT_BLOCK_SWAP(dst_bit_block);
		*dstptr=dst_bit_block;
		dst_bit_offset=0;
		dstptr++;
	    }
#else
	    // Store the result in destination image
	    dst_bit_block<<=1;
	    dst_bit_block|=dilated;
	    dst_bit_offset++;
	    if (dst_bit_offset==CAM_BIT_BLOCK_SIZE) {
		dst_bit_block=CAM_BIT_BLOCK_SWAP(dst_bit_block);
		*dstptr=dst_bit_block;
		dst_bit_offset=0;
		dstptr++;
	    }
#endif
#else
	    // Get the original value of the pixel
	    i=(src_bit_offset+CAM_MM_NEIGHB/2);
	    srcpix[CAM_MM_ORIGINAL]=(((linesPtr[CAM_MM_NEIGHB/2][i>>3])&(0x80>>(i&7)))>>(7-(i&7)));

	    // We've got the original, dilated and eroded value of pixels
	    srcpix[CAM_MM_DILATED]=(int)dilated; // To optimize
	    srcpix[CAM_MM_ERODED]=(int)eroded;   // To optimize
	    
	    // Let's compute the final result
	    switch (kernel->operation) {
	    case CAM_MM_SUBSTRACTION:
		value=srcpix[kernel->source1]-srcpix[kernel->source2];
		// Saturation on the result
		if (value<0) value=0;
		else if (value>1) value=1;
		break;
	    case CAM_MM_MULTIPLEX:
		value=srcpix[kernel->source1];
		break;
	    case CAM_MM_THINNING:
		// TO DO : Check these formulas
		value=srcpix[CAM_MM_ORIGINAL];
		if ((value<srcpix[CAM_MM_ERODED])&&(value>=srcpix[CAM_MM_DILATED])) {
		    value=srcpix[CAM_MM_ERODED];
		}
		break;
	    case CAM_MM_THICKENING:
		value=srcpix[CAM_MM_ORIGINAL];
		if ((value<=srcpix[CAM_MM_ERODED])&&(value>srcpix[CAM_MM_DILATED])) {
		    value=srcpix[CAM_MM_DILATED];
		}
		break;
	    }
	    
	    // Store the result in destination image
	    dst_bit_block<<=1;
	    dst_bit_block|=value;
	    dst_bit_offset++;
	    if (dst_bit_offset==CAM_BIT_BLOCK_SIZE) {
		dst_bit_block=CAM_BIT_BLOCK_SWAP(dst_bit_block);
		*dstptr=dst_bit_block;
		dst_bit_offset=0;
		dstptr++;
	    }
	    if ((idempotence)&&(value!=srcpix[CAM_MM_ORIGINAL])) idempotence=0;
#endif
	}
	
	// Go to next line
	srcptr=(CAM_BIT_BLOCK*)(((char*)cpsrcptr)+dest->widthStep);
	if (dst_bit_offset) {
	    // Store the remaining pixels in the destination image
	    dst_eol_block=*dstptr; // Load 32 pixels from original dest image
	    dst_eol_block=CAM_BIT_BLOCK_SWAP(dst_eol_block);
	    dst_eol_block&=((
		((CAM_BIT_BLOCK)1)<<
		(CAM_BIT_BLOCK_SIZE-dst_bit_offset)
		)-1);
	    dst_eol_block|=(dst_bit_block<<(CAM_BIT_BLOCK_SIZE-dst_bit_offset));
	    dst_eol_block=CAM_BIT_BLOCK_SWAP(dst_eol_block);
	    *dstptr=dst_eol_block; // Store 32 pixels at once
	}
	dstptr=(CAM_BIT_BLOCK*)(((char*)cpdstptr)+dest->widthStep);

	// Reset neighborhood line pointers
	tmpptr=linesPtr[0];
	for (i=0;i<CAM_MM_NEIGHB-1;i++) {
	    linesPtr[i]=linesPtr[i+1];
	}
	linesPtr[CAM_MM_NEIGHB-1]=tmpptr;
    }

#ifdef CAM_MM_ONE_OP
    return 1;
#else
    return !idempotence;
#endif
}

#endif // GRAY_SCALE_ONLY

#ifndef CAM_MM_ONE_OP
#undef CAM_MM_DO_EROSION
#undef CAM_MM_DO_DILATION
#endif

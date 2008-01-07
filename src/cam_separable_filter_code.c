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

/* Linear Separable Filter Kernel
 * C code */

#if (CAM_FIXED_FILTER == CAM_SOBEL_H)||(CAM_FIXED_FILTER == CAM_GAUSSIAN_3x3)
#define PROCESS_X(sign,y) linesPtr[y][x]=(int)tmpLine##sign[x]+(((int)tmpLine##sign[x+1])<<1)+(int)tmpLine##sign[x+2]
#elif (CAM_FIXED_FILTER == CAM_SOBEL_V)||(CAM_FIXED_FILTER == CAM_SCHARR_V)
#define PROCESS_X(sign,y) linesPtr[y][x]=(int)tmpLine##sign[x+2]-(int)tmpLine##sign[x]
#elif CAM_FIXED_FILTER == CAM_GAUSSIAN_5x5
#define PROCESS_X(sign,y) linesPtr[y][x]=\
    (int)tmpLine##sign[x]+\
    (((int)tmpLine##sign[x+1])<<2)+\
    (((int)tmpLine##sign[x+2])<<2)+(((int)tmpLine##sign[x+2])<<1)+\
    (((int)tmpLine##sign[x+3])<<2)+\
    (int)tmpLine##sign[x+4]
#elif CAM_FIXED_FILTER == CAM_GAUSSIAN_7x7
#define PROCESS_X(sign,y) linesPtr[y][x]=\
    (int)tmpLine##sign[x]+\
    (((int)tmpLine##sign[x+1])<<2)+(((int)tmpLine##sign[x+1])<<1)+\
    (((int)tmpLine##sign[x+2])<<4)-((int)tmpLine##sign[x+2])+\
    (((int)tmpLine##sign[x+3])<<4)+(((int)tmpLine##sign[x+3])<<2)+\
    (((int)tmpLine##sign[x+4])<<4)-((int)tmpLine##sign[x+4])+\
    (((int)tmpLine##sign[x+5])<<2)+(((int)tmpLine##sign[x+5])<<1)+\
    (int)tmpLine##sign[x+6]
#elif CAM_FIXED_FILTER == CAM_SCHARR_H
#define PROCESS_X(sign,y) linesPtr[y][x]=((int)tmpLine##sign[x]<<1)+(int)tmpLine##sign[x]+(((int)tmpLine##sign[x+1])<<3)+(((int)tmpLine##sign[x+1])<<1)+((int)tmpLine##sign[x+2]<<1)+(int)tmpLine##sign[x+2]    
#else
#define PROCESS_X(sign,y) \
    linesPtr[y][x]=kernel->x[0]*tmpLine##sign[x]; \
    for (i=1;i<CAM_LF_NEIGHB_X;i++) linesPtr[y][x]+=kernel->x[i]*tmpLine##sign[x+i];
#endif

#if defined(CAM_FIXED_FILTER)
int camSepFilter(CamImage *source, CamImage *dest)
#else
int camSepFilter(CamImage *source, CamImage *dest, CamSepFilterKernel *kernel)
#endif
{
    int i,j,x,y;
    int width,height;
    int left,top;
    int result;
    unsigned CAM_PIXEL *srcptr,*cpsrcptr;
    unsigned CAM_PIXEL_DST *dstptr,*cpdstptr;
    int *tmpptr, *linesPtr[CAM_LF_NEIGHB_Y];
    int linesBuffer[CAM_LF_NEIGHB_Y][CAM_MAX_SCANLINE];
    unsigned CAM_PIXEL tmpLineU[((CAM_MAX_SCANLINE+CAM_LF_NEIGHB_X-1)&~15)+16];
    signed CAM_PIXEL *tmpLineS = (signed CAM_PIXEL*) tmpLineU;
    int valmax;

    CamRun *run;
    int startx,endx;
    CamInternalROIPolicyStruct iROI;
#ifndef CAM_FIXED_FILTER
    int acc=0;
#endif
    
    // ROI (Region Of Interest) management
    CAM_CHECK(camSepFilter,camInternalROIPolicy(source, dest, &iROI, 1));
    CAM_CHECK_ARGS(camSepFilter,iROI.nChannels==1);
    CAM_CHECK_ARGS(camSepFilter,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camSepFilter,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camSepFilter,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL_DST)*8));
    CAM_CHECK_ARGS(camSepFilter,(dest->depth&CAM_DEPTH_MASK)>=8);

    // Saturation management
    valmax=(1<<(dest->depth&CAM_DEPTH_MASK))-1;

    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    if (source->roi) {
        left=iROI.srcroi.xOffset;
        top=iROI.srcroi.yOffset;
        i=left; if (i>CAM_LF_NEIGHB_X/2) i=CAM_LF_NEIGHB_X/2;
        j=top; if (j>CAM_LF_NEIGHB_Y/2) j=CAM_LF_NEIGHB_Y/2;
        srcptr=(unsigned CAM_PIXEL*)(source->imageData+iROI.srcchoffset+(top-j)*source->widthStep)+(left-i);
    } else {
        srcptr=(unsigned CAM_PIXEL*)(source->imageData+iROI.srcchoffset);
        left=0;
        top=0;
    }
    dstptr=(unsigned CAM_PIXEL_DST*)iROI.dstptr;
    CAM_CHECK_ARGS(camSepFilter,(width>CAM_LF_NEIGHB_X/2));
    CAM_CHECK_ARGS(camSepFilter,(height>CAM_LF_NEIGHB_Y/2));    
    
    // Mask management
    if (iROI.mask) {
        run=iROI.mask->runs+1; // Skip the first dummy run
    } else {
        run=NULL;
    }

    // Initialize algorithm
    for (i=0;i<CAM_LF_NEIGHB_Y;i++) {
        linesPtr[i]=linesBuffer[i];
    }
    if (dest->depth&CAM_DEPTH_SIGN) {
        valmax>>=1;
    }
    
    // Initialize neighbourhood
    
    // Fill the top lines
    for (y=0;y+top<CAM_LF_NEIGHB_Y/2;y++) {
        // Out of frame : fill with border color
        if (source->borderMode[CAM_SIDE_TOP_INDEX]==CAM_BORDER_REPLICATE) {
            cpsrcptr=srcptr;
            for (x=0;x<CAM_LF_NEIGHB_X/2;x++) {
                tmpLineU[x]=*srcptr;
            }
            for (;x<width+CAM_LF_NEIGHB_X/2;x++) {
                tmpLineU[x]=*srcptr;
                srcptr+=iROI.srcinc;
            }
            for (;x<width+CAM_LF_NEIGHB_X-1;x++) {
                tmpLineU[x]=*(srcptr-iROI.srcinc);
            }
            srcptr=cpsrcptr;
        } else {
            for (x=0;x<width+CAM_LF_NEIGHB_X-1;x++) {
                tmpLineU[x]=source->borderConst[CAM_SIDE_TOP_INDEX];
            }
        }
        
        // Process the line
        if (source->depth & CAM_DEPTH_SIGN) {
	    for (x=0;x<width;x++) {
		PROCESS_X(S,y);
	    }
	} else {
	    for (x=0;x<width;x++) {
		PROCESS_X(U,y);
	    }
	}

    }
    
    // Fill the next lines with image pixels
    for (;y<CAM_LF_NEIGHB_Y-1;y++) {
        cpsrcptr=srcptr;
        for (x=0;x<CAM_LF_NEIGHB_X/2;x++) {
            if (left+x-CAM_LF_NEIGHB_X/2<0) {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_LEFT_INDEX]==CAM_BORDER_REPLICATE) {
                    tmpLineU[x]=*srcptr;
                } else {
                    tmpLineU[x]=source->borderConst[CAM_SIDE_LEFT_INDEX];
                }
            } else {
                // Get border pixels from the source frame
                tmpLineU[x]=*srcptr;
                srcptr+=iROI.srcinc;
            }
        }
        for (;x<width+CAM_LF_NEIGHB_X/2;x++) {
            tmpLineU[x]=*srcptr;
            srcptr+=iROI.srcinc;
        }
        for (;x<width+CAM_LF_NEIGHB_X-1;x++) {
            if (left+x-CAM_LF_NEIGHB_X/2<source->width) {
                // Get border pixels from the source frame
                tmpLineU[x]=*srcptr;
                srcptr+=iROI.srcinc;
            } else {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_RIGHT_INDEX]==CAM_BORDER_REPLICATE) {
                    tmpLineU[x]=*(srcptr-iROI.srcinc);
                } else {
                    tmpLineU[x]=source->borderConst[CAM_SIDE_RIGHT_INDEX];
                }
            }
        }
        srcptr=(unsigned CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
        
        // Process the line
        if (source->depth & CAM_DEPTH_SIGN) {
	    for (x=0;x<width;x++) {
		PROCESS_X(S,y);
	    }
	} else {
	    for (x=0;x<width;x++) {
		PROCESS_X(U,y);
	    }
	}

    }
    
    // Now process the whole image
    // This is the main loop
    for (y=0;y<height;y++) {
        cpsrcptr=srcptr;
        cpdstptr=dstptr;
        
        // Start a new line
        // Have we reached the bottom of the frame ?
        if (top+y+CAM_LF_NEIGHB_Y/2>=source->height) {
            if (source->borderMode[CAM_SIDE_BOTTOM_INDEX]==CAM_BORDER_REPLICATE) {
                // Go up one line (in order to stay on the last line)
                cpsrcptr=(unsigned CAM_PIXEL*)(((char*)cpsrcptr)-source->widthStep);
                srcptr=cpsrcptr;
                for (x=0;x<CAM_LF_NEIGHB_X/2;x++) {
                    tmpLineU[x]=*srcptr;
                }
                for (;x<width+CAM_LF_NEIGHB_X/2;x++) {
                    tmpLineU[x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
                for (;x<width+CAM_LF_NEIGHB_X-1;x++) {
                    tmpLineU[x]=*(srcptr-iROI.srcinc);
                }
            } else {
                for (x=0;x<width+CAM_LF_NEIGHB_X-1;x++) {
                    tmpLineU[x]=source->borderConst[CAM_SIDE_BOTTOM_INDEX];
                }
            }
        } else {
            for (x=0;x<CAM_LF_NEIGHB_X/2;x++) {
                if (left+x-CAM_LF_NEIGHB_X/2<0) {
                    // Out of frame : fill with border color
                    if (source->borderMode[CAM_SIDE_LEFT_INDEX]==CAM_BORDER_REPLICATE) {
                        tmpLineU[x]=*srcptr;
                    } else {
                        tmpLineU[x]=source->borderConst[CAM_SIDE_LEFT_INDEX];
                    }
                } else {
                    // Get border pixels from the source frame
                    tmpLineU[x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
            }

            // Fast transfer with memcpy
            if (iROI.srcinc==1) {
                memcpy(&tmpLineU[x],srcptr,(source->width-left+CAM_LF_NEIGHB_X/2-x)<<(sizeof(CAM_PIXEL)-1));
                x=source->width-left+CAM_LF_NEIGHB_X/2;
            } else {
                for (;x<source->width-left+CAM_LF_NEIGHB_X/2;x++) {
                    // Get a pixel from the source frame
                    tmpLineU[x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
            }
            for (;x<width+CAM_LF_NEIGHB_X-1;x++) {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_RIGHT_INDEX]==CAM_BORDER_REPLICATE) {
                    tmpLineU[x]=*(srcptr+(source->width-1)*iROI.srcinc);
                } else {
                    tmpLineU[x]=source->borderConst[CAM_SIDE_RIGHT_INDEX];
                }
            }
        }
        
        // Process the line
        if (source->depth & CAM_DEPTH_SIGN) {
	    for (x=0;x<width;x++) {
		PROCESS_X(S,CAM_LF_NEIGHB_Y-1);
	    }
	} else {
	    for (x=0;x<width;x++) {
		PROCESS_X(U,CAM_LF_NEIGHB_Y-1);
	    }
	}

        startx=0;
        do {
            // Mask management
            if (!iROI.mask) {
                endx=width;
            } else {
                while ((run->value==0)&&(run->line==y)) {
                    startx+=run->length;
                    run++;
                }
                if (run->line!=y) break; // Go directly to next line
                endx=startx+run->length;
                dstptr=cpdstptr+startx*iROI.dstinc;
            }
            
            // Process all the pixels in the line
            for (x=startx;x<endx;x++) {   
                
                // Now, let's compute the result of the linear filter
#if CAM_FIXED_FILTER == CAM_SOBEL_H
                result=linesPtr[2][x]-linesPtr[0][x];
#elif CAM_FIXED_FILTER == CAM_SCHARR_H
                result=linesPtr[2][x]-linesPtr[0][x];
		result>>=2;
#elif CAM_FIXED_FILTER == CAM_SOBEL_V
                result=linesPtr[0][x]+(linesPtr[1][x]<<1)+linesPtr[2][x];
#elif CAM_FIXED_FILTER == CAM_GAUSSIAN_3x3
                result=linesPtr[0][x]+(linesPtr[1][x]<<1)+linesPtr[2][x];
                result>>=4;
#elif CAM_FIXED_FILTER == CAM_GAUSSIAN_5x5
                result=\
                    linesPtr[0][x]+\
                    ((linesPtr[1][x])<<2)+\
                    ((linesPtr[2][x])<<2)+((linesPtr[2][x])<<1)+\
                    ((linesPtr[3][x])<<2)+\
                    linesPtr[4][x];
                result>>=8;
#elif CAM_FIXED_FILTER == CAM_GAUSSIAN_7x7
                result=\
                    linesPtr[0][x]+\
                    ((linesPtr[1][x])<<2)+((linesPtr[1][x])<<1)+\
                    ((linesPtr[2][x])<<4)-(linesPtr[2][x])+\
                    ((linesPtr[3][x])<<4)+((linesPtr[3][x])<<2)+\
                    ((linesPtr[4][x])<<4)-(linesPtr[4][x])+\
                    ((linesPtr[5][x])<<2)+((linesPtr[5][x])<<1)+\
                    linesPtr[6][x];
                result>>=12;
#elif CAM_FIXED_FILTER == CAM_SCHARR_V
                result=(linesPtr[0][x]<<1)+linesPtr[0][x]+(linesPtr[1][x]<<1)+(linesPtr[1][x]<<3)+(linesPtr[2][x]<<1)+linesPtr[2][x];
		result>>=2;
#else
                result=kernel->y[0]*linesPtr[0][x];
                for (i=1;i<CAM_LF_NEIGHB_Y;i++) { 
                    result+=kernel->y[i]*linesPtr[i][x];
                }

                if (kernel->coeff1!=1) result*=kernel->coeff1;
                if (kernel->coeff2!=0) result>>=kernel->coeff2;
#endif
                
#ifdef CAM_LF_ABS
                if (result<0) result=-result;
                // Saturation :
                if (result>valmax) result=valmax;
#else
                if (dest->depth&CAM_DEPTH_SIGN) {
                    // Saturation :
                    if (result<-valmax) result=-valmax; 
                    else if (result>valmax) result=valmax;
                } else {
                    // Saturation :
                    if (result<0) result=0;
                    else if (result>valmax) result=valmax;
                }
#endif
                
                // Store the result in destination image
                *dstptr=(unsigned CAM_PIXEL_DST)result;
                dstptr+=iROI.dstinc;
#ifndef CAM_FIXED_FILTER
                acc+=result; // Accumulate
#endif
            }

            if (iROI.mask) {
                startx+=run->length;
                run++;
            }

        } while ((run)&&(run->line==y));
        
        // Go to next line
        srcptr=(unsigned CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
        dstptr=(unsigned CAM_PIXEL_DST*)(((char*)cpdstptr)+dest->widthStep);
        
        // Reset neighborhood line pointers
        tmpptr=linesPtr[0];
        for (i=0;i<CAM_LF_NEIGHB_Y-1;i++) {
            linesPtr[i]=linesPtr[i+1];
        }
        linesPtr[CAM_LF_NEIGHB_Y-1]=tmpptr;
    }
    
    camInternalROIPolicyExit(&iROI);
#ifndef CAM_FIXED_FILTER
    return acc;
#else
    return 1;
#endif
}

#undef PROCESS_X


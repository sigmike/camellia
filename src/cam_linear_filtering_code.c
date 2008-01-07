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

/* Linear Filtering Kernel
 * C code */

int camLinearFilter(CamImage *source, CamImage *dest, camLinearFilterKernelPtr params)
{
#define CAM_ALIGN 0
    int i,j,x,y;
    int width,height;
    int left,top;
    int result;
    CAM_PIXEL *srcptr,*tmpptr,*cpsrcptr;
    CAM_PIXEL_DST *dstptr,*cpdstptr;
    unsigned CAM_PIXEL *linesPtr[CAM_LF_NEIGHB_Y];
    CAM_PIXEL linesBuffer[CAM_LF_NEIGHB_Y][((CAM_MAX_SCANLINE+CAM_LF_NEIGHB_X-1+CAM_ALIGN)&~15)+16];
    int valmax;
#ifndef CAM_SOBEL
    int xp,yp;
    int value;
    int nbk;
    int xpk[CAM_LF_NEIGHB_X*CAM_LF_NEIGHB_Y];
    int ypk[CAM_LF_NEIGHB_X*CAM_LF_NEIGHB_Y];
    int valk[CAM_LF_NEIGHB_X*CAM_LF_NEIGHB_Y];
#else
    int results[3];
#endif
    DECLARE_MASK_MANAGEMENT;

    CamInternalROIPolicyStruct iROI;
    int acc=0;
    
    // ROI (Region Of Interest) management
    CAM_CHECK(camLinearFilter,camInternalROIPolicy(source, dest, &iROI, 1));
    CAM_CHECK_ARGS(camLinearFilter,iROI.nChannels==1);
    CAM_CHECK_ARGS(camLinearFilter,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camLinearFilter,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camLinearFilter,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL_DST)*8));
    CAM_CHECK_ARGS(camLinearFilter,(dest->depth&CAM_DEPTH_MASK)>=8);

    // Saturation management
    valmax=(1<<(dest->depth&CAM_DEPTH_MASK))-1;

    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    if (source->roi) {
        left=iROI.srcroi.xOffset;
        top=iROI.srcroi.yOffset;
        i=left; if (i>CAM_LF_NEIGHB_X/2) i=CAM_LF_NEIGHB_X/2;
        j=top; if (j>CAM_LF_NEIGHB_Y/2) j=CAM_LF_NEIGHB_Y/2;
        srcptr=(CAM_PIXEL*)(source->imageData+iROI.srcchoffset+(top-j)*source->widthStep)+(left-i);
    } else {
        srcptr=(CAM_PIXEL*)(source->imageData+iROI.srcchoffset);
        left=0;
        top=0;
    }
    dstptr=(CAM_PIXEL_DST*)iROI.dstptr;
    CAM_CHECK_ARGS(camLinearFilter,(width>CAM_LF_NEIGHB_X/2));
    CAM_CHECK_ARGS(camLinearFilter,(height>CAM_LF_NEIGHB_Y/2));    
    
    // Mask management
    INIT_MASK_MANAGEMENT;

    // Initialize algorithm
    for (i=0;i<CAM_LF_NEIGHB_Y;i++) {
        linesPtr[i]=linesBuffer[i];
    }
    if (dest->depth&CAM_DEPTH_SIGN) {
        valmax>>=1;
    }
    
    // Initialize compressed kernel
#ifndef CAM_SOBEL
    nbk=0;
    for (yp=0;yp<CAM_LF_NEIGHB_Y;yp++) {
        for (xp=0;xp<CAM_LF_NEIGHB_X;xp++) {
            value=params->kernel[yp][xp];
            if (value) {
                valk[nbk]=value;
                xpk[nbk]=xp;
                ypk[nbk]=yp;
                nbk++;
            }
        }
    }
#endif
    
    // Initialize neighbourhood
    
    // Fill the top lines
    for (y=0;y+top<CAM_LF_NEIGHB_Y/2;y++) {
        // Out of frame : fill with border color
        if (source->borderMode[CAM_SIDE_TOP_INDEX]==CAM_BORDER_REPLICATE) {
            cpsrcptr=srcptr;
            for (x=0;x<CAM_LF_NEIGHB_X/2;x++) {
                linesPtr[y][x+CAM_ALIGN]=*srcptr;
            }
            for (;x<width+CAM_LF_NEIGHB_X/2;x++) {
                linesPtr[y][x+CAM_ALIGN]=*srcptr;
                srcptr+=iROI.srcinc;
            }
            for (;x<width+CAM_LF_NEIGHB_X-1;x++) {
                linesPtr[y][x+CAM_ALIGN]=*(srcptr-iROI.srcinc);
            }
            srcptr=cpsrcptr;
        } else {
            for (x=0;x<width+CAM_LF_NEIGHB_X-1;x++) {
                linesPtr[y][x+CAM_ALIGN]=source->borderConst[CAM_SIDE_TOP_INDEX];
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
                    linesPtr[y][x+CAM_ALIGN]=*srcptr;
                } else {
                    linesPtr[y][x+CAM_ALIGN]=source->borderConst[CAM_SIDE_LEFT_INDEX];
                }
            } else {
                // Get border pixels from the source frame
                linesPtr[y][x+CAM_ALIGN]=*srcptr;
                srcptr+=iROI.srcinc;
            }
        }
        for (;x<width+CAM_LF_NEIGHB_X/2;x++) {
            linesPtr[y][x+CAM_ALIGN]=*srcptr;
            srcptr+=iROI.srcinc;
        }
        for (;x<width+CAM_LF_NEIGHB_X-1;x++) {
            if (left+x-CAM_LF_NEIGHB_X/2<source->width) {
                // Get border pixels from the source frame
                linesPtr[y][x+CAM_ALIGN]=*srcptr;
                srcptr+=iROI.srcinc;
            } else {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_RIGHT_INDEX]==CAM_BORDER_REPLICATE) {
                    linesPtr[y][x+CAM_ALIGN]=*(srcptr-iROI.srcinc);
                } else {
                    linesPtr[y][x+CAM_ALIGN]=source->borderConst[CAM_SIDE_RIGHT_INDEX];
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
        if (top+y+CAM_LF_NEIGHB_Y/2>=source->height) {
            if (source->borderMode[CAM_SIDE_BOTTOM_INDEX]==CAM_BORDER_REPLICATE) {
                // Go up one line (in order to stay on the last line)
                cpsrcptr=(CAM_PIXEL*)(((char*)cpsrcptr)-source->widthStep);
                srcptr=cpsrcptr;
                for (x=0;x<CAM_LF_NEIGHB_X/2;x++) {
                    linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=*srcptr;
                }
                for (;x<width+CAM_LF_NEIGHB_X/2;x++) {
                    linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
                for (;x<width+CAM_LF_NEIGHB_X-1;x++) {
                    linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=*(srcptr-iROI.srcinc);
                }
            } else {
                for (x=0;x<width+CAM_LF_NEIGHB_X-1;x++) {
                    linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=source->borderConst[CAM_SIDE_BOTTOM_INDEX];
                }
            }
        } else {
            for (x=0;x<CAM_LF_NEIGHB_X/2;x++) {
                if (left+x-CAM_LF_NEIGHB_X/2<0) {
                    // Out of frame : fill with border color
                    if (source->borderMode[CAM_SIDE_LEFT_INDEX]==CAM_BORDER_REPLICATE) {
                        linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=*srcptr;
                    } else {
                        linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=source->borderConst[CAM_SIDE_LEFT_INDEX];
                    }
                } else {
                    // Get border pixels from the source frame
                    linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
            }

            // Fast transfer with memcpy
            if (iROI.srcinc==1) {
                memcpy(&linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN],srcptr,((source->width-left+CAM_LF_NEIGHB_X/2-x)<<(sizeof(CAM_PIXEL)-1)));
                x=source->width-left+CAM_LF_NEIGHB_X/2;
            } else {
                for (;x<source->width-left+CAM_LF_NEIGHB_X/2;x++) {
                    // Get a pixel from the source frame
                    linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
            }
            for (;x<width+CAM_LF_NEIGHB_X-1;x++) {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_RIGHT_INDEX]==CAM_BORDER_REPLICATE) {
                    linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=*(srcptr+(source->width-1)*iROI.srcinc);
                } else {
                    linesPtr[CAM_LF_NEIGHB_Y-1][x+CAM_ALIGN]=source->borderConst[CAM_SIDE_RIGHT_INDEX];
                }
            }
        }
       
	if (source->depth & CAM_DEPTH_SIGN) {	
	    BEGIN_MASK_MANAGEMENT(dstptr=cpdstptr+startx*iROI.dstinc;)

#ifdef CAM_SOBEL
		if (vertical_edges) {
		    results[0]=(int)((signed CAM_PIXEL **)linesPtr)[0][startx+CAM_ALIGN]+(((int)((signed CAM_PIXEL **)linesPtr)[1][startx+CAM_ALIGN])<<1)+(int)((signed CAM_PIXEL **)linesPtr)[2][startx+CAM_ALIGN];
		    results[1]=(int)((signed CAM_PIXEL **)linesPtr)[0][startx+1+CAM_ALIGN]+(((int)((signed CAM_PIXEL **)linesPtr)[1][startx+1+CAM_ALIGN])<<1)+(int)((signed CAM_PIXEL **)linesPtr)[2][startx+1+CAM_ALIGN];
		} else {
		    results[0]=-(int)((signed CAM_PIXEL **)linesPtr)[0][startx+CAM_ALIGN]+(int)((signed CAM_PIXEL **)linesPtr)[2][startx+CAM_ALIGN];
		    results[1]=-(int)((signed CAM_PIXEL **)linesPtr)[0][startx+1+CAM_ALIGN]+(int)((signed CAM_PIXEL **)linesPtr)[2][startx+1+CAM_ALIGN];
		}
#endif

	    // Process all the pixels in the line
	    for (x=startx+CAM_LF_NEIGHB_X-1;x<endx+CAM_LF_NEIGHB_X-1;x++) {   

#ifdef CAM_SOBEL
		if (vertical_edges) {
		    results[2]=(int)((signed CAM_PIXEL **)linesPtr)[0][x+CAM_ALIGN]+(((int)((signed CAM_PIXEL **)linesPtr)[1][x+CAM_ALIGN])<<1)+(int)((signed CAM_PIXEL **)linesPtr)[2][x+CAM_ALIGN];
		    result=results[2]-results[0];
		} else {
		    results[2]=-(int)((signed CAM_PIXEL **)linesPtr)[0][x+CAM_ALIGN]+(int)((signed CAM_PIXEL **)linesPtr)[2][x+CAM_ALIGN];
		    result=results[0]+(results[1]<<1)+results[2];
		}
		results[0]=results[1];
		results[1]=results[2];
#else
		// Now, let's compute the result of the linear filter
		result=0;
		i=x-CAM_LF_NEIGHB_X+1+CAM_ALIGN;
		for (j=0;j<nbk;j++) {
		    result+=((signed CAM_PIXEL **)linesPtr)[ypk[j]][i+xpk[j]]*valk[j];
		}
		if (params->coeff1!=1) result*=params->coeff1;
		if (params->coeff2!=0) result>>=params->coeff2;
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
		*dstptr=(CAM_PIXEL_DST)result;
		dstptr+=iROI.dstinc;
		acc+=result; // Accumulate
	    }

	    END_MASK_MANAGEMENT;
	} else {
	    BEGIN_MASK_MANAGEMENT(dstptr=cpdstptr+startx*iROI.dstinc;)

#ifdef CAM_SOBEL
		if (vertical_edges) {
		    results[0]=(int)linesPtr[0][startx+CAM_ALIGN]+(((int)linesPtr[1][startx+CAM_ALIGN])<<1)+(int)linesPtr[2][startx+CAM_ALIGN];
		    results[1]=(int)linesPtr[0][startx+1+CAM_ALIGN]+(((int)linesPtr[1][startx+1+CAM_ALIGN])<<1)+(int)linesPtr[2][startx+1+CAM_ALIGN];
		} else {
		    results[0]=-(int)linesPtr[0][startx+CAM_ALIGN]+(int)linesPtr[2][startx+CAM_ALIGN];
		    results[1]=-(int)linesPtr[0][startx+1+CAM_ALIGN]+(int)linesPtr[2][startx+1+CAM_ALIGN];
		}
#endif

	    // Process all the pixels in the line
	    for (x=startx+CAM_LF_NEIGHB_X-1;x<endx+CAM_LF_NEIGHB_X-1;x++) {   

#ifdef CAM_SOBEL
		if (vertical_edges) {
		    results[2]=(int)linesPtr[0][x+CAM_ALIGN]+(((int)linesPtr[1][x+CAM_ALIGN])<<1)+(int)linesPtr[2][x+CAM_ALIGN];
		    result=results[2]-results[0];
		} else {
		    results[2]=-(int)linesPtr[0][x+CAM_ALIGN]+(int)linesPtr[2][x+CAM_ALIGN];
		    result=results[0]+(results[1]<<1)+results[2];
		}
		results[0]=results[1];
		results[1]=results[2];
#else
		// Now, let's compute the result of the linear filter
		result=0;
		i=x-CAM_LF_NEIGHB_X+1+CAM_ALIGN;
		/* Original version : full scan of the kernel
		   for (yp=0;yp<CAM_LF_NEIGHB_Y;yp++) {
			for (xp=0;xp<CAM_LF_NEIGHB_X;xp++) {
			    value=linesPtr[yp][i+xp];
			    result+=value*params->kernel[yp][xp];
			}
		   } */
		for (j=0;j<nbk;j++) {
		    result+=linesPtr[ypk[j]][i+xpk[j]]*valk[j];
		}
		if (params->coeff1!=1) result*=params->coeff1;
		if (params->coeff2!=0) result>>=params->coeff2;
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
		*dstptr=(CAM_PIXEL_DST)result;
		dstptr+=iROI.dstinc;
		acc+=result; // Accumulate
	    }

	    END_MASK_MANAGEMENT;
	}

        // Go to next line
        srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
        dstptr=(CAM_PIXEL_DST*)(((char*)cpdstptr)+dest->widthStep);
        
        // Reset neighborhood line pointers
        tmpptr=linesPtr[0];
        for (i=0;i<CAM_LF_NEIGHB_Y-1;i++) {
            linesPtr[i]=linesPtr[i+1];
        }
        linesPtr[CAM_LF_NEIGHB_Y-1]=tmpptr;
    }
    
    camInternalROIPolicyExit(&iROI);
    return acc;
}
#undef CAM_ALIGN

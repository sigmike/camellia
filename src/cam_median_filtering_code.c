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

/* Median Filtering Kernel
 * C code */

int camMedianFilter(CamImage *source, CamImage *dest)
{
    int i,j,x,y,xp,yp;
    int width,height;
    int value,result;
    int left,top;
    CAM_PIXEL *srcptr,*dstptr,*tmpptr,*cpsrcptr,*cpdstptr;
    CAM_PIXEL *linesPtr[CAM_MF_NEIGHB];
    CAM_PIXEL linesBuffer[CAM_MF_NEIGHB][CAM_MAX_SCANLINE+CAM_MF_NEIGHB-1];
    
    // Data for sorting pixels in the neighbourhood
    int values[CAM_MF_NEIGHB*CAM_MF_NEIGHB];
    int next[CAM_MF_NEIGHB*CAM_MF_NEIGHB];
    int first,last,nb,prev;
    
    CamInternalROIPolicyStruct iROI;
    
    // ROI (Region Of Interest) management
    CAM_CHECK(camMedianFilter,camInternalROIPolicy(source, dest, &iROI, 1));
    CAM_CHECK_ARGS(camMedianFilter,iROI.nChannels==1);

    CAM_CHECK_ARGS(camMedianFilter,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camMedianFilter,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camMedianFilter,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camMedianFilter,(dest->depth&CAM_DEPTH_MASK)>=8);
    
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    if (source->roi) {
        left=iROI.srcroi.xOffset;
        top=iROI.srcroi.yOffset;
        i=left; if (i>CAM_MF_NEIGHB/2) i=CAM_MF_NEIGHB/2;
        j=top; if (j>CAM_MF_NEIGHB/2) j=CAM_MF_NEIGHB/2;
        srcptr=(CAM_PIXEL*)(source->imageData+iROI.srcchoffset+(top-j)*source->widthStep)+(left-i);
    } else {
        srcptr=(CAM_PIXEL*)(source->imageData+iROI.srcchoffset);
        left=0;
        top=0;
    }
    dstptr=(CAM_PIXEL*)iROI.dstptr;
    CAM_CHECK_ARGS(camMedianFilter,(width>CAM_MF_NEIGHB/2));
    CAM_CHECK_ARGS(camMedianFilter,(height>CAM_MF_NEIGHB/2));    
	
    // Initialize algorithm
    for (i=0;i<CAM_MF_NEIGHB;i++) {
	linesPtr[i]=linesBuffer[i];
    }

    // Initialize neighbourhood
    
    // Fill the top lines
    for (y=0;y+top<CAM_MF_NEIGHB/2;y++) {
        // Out of frame : fill with border color
        if (source->borderMode[CAM_SIDE_TOP_INDEX]==CAM_BORDER_REPLICATE) {
            cpsrcptr=srcptr;
            for (x=0;x<CAM_MF_NEIGHB/2;x++) {
                linesPtr[y][x]=*srcptr;
            }
            for (;x<width+CAM_MF_NEIGHB/2;x++) {
                linesPtr[y][x]=*srcptr;
                srcptr+=iROI.srcinc;
            }
            for (;x<width+CAM_MF_NEIGHB-1;x++) {
                linesPtr[y][x]=*(srcptr-iROI.srcinc);
            }
            srcptr=cpsrcptr;
        } else {
            for (x=0;x<width+CAM_MF_NEIGHB-1;x++) {
                linesPtr[y][x]=source->borderConst[CAM_SIDE_TOP_INDEX];
            }
        }
    }
    
    // Fill the next lines with image pixels
    for (;y<CAM_MF_NEIGHB-1;y++) {
        cpsrcptr=srcptr;
        for (x=0;x<CAM_MF_NEIGHB/2;x++) {
            if (left+x-CAM_MF_NEIGHB/2<0) {
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
        for (;x<width+CAM_MF_NEIGHB/2;x++) {
            linesPtr[y][x]=*srcptr;
            srcptr+=iROI.srcinc;
        }
        for (;x<width+CAM_MF_NEIGHB-1;x++) {
            if (left+x-CAM_MF_NEIGHB/2<source->width) {
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
        if (top+y+CAM_MF_NEIGHB/2>=source->height) {
            if (source->borderMode[CAM_SIDE_BOTTOM_INDEX]==CAM_BORDER_REPLICATE) {
                // Go up one line (in order to stay on the last line)
                cpsrcptr=(CAM_PIXEL*)(((char*)cpsrcptr)-source->widthStep);
                srcptr=cpsrcptr;
                for (x=0;x<CAM_MF_NEIGHB/2;x++) {
                    linesPtr[CAM_MF_NEIGHB-1][x]=*srcptr;
                }
                for (;x<width+CAM_MF_NEIGHB/2;x++) {
                    linesPtr[CAM_MF_NEIGHB-1][x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
                for (;x<width+CAM_MF_NEIGHB-1;x++) {
                    linesPtr[CAM_MF_NEIGHB-1][x]=*(srcptr-iROI.srcinc);
                }
            } else {
                for (x=0;x<width+CAM_MF_NEIGHB-1;x++) {
                    linesPtr[CAM_MF_NEIGHB-1][x]=source->borderConst[CAM_SIDE_BOTTOM_INDEX];
                }
            }
        } else {
            for (x=0;x<CAM_MF_NEIGHB/2;x++) {
                if (left+x-CAM_MF_NEIGHB/2<0) {
                    // Out of frame : fill with border color
                    if (source->borderMode[CAM_SIDE_LEFT_INDEX]==CAM_BORDER_REPLICATE) {
                        linesPtr[CAM_MF_NEIGHB-1][x]=*srcptr;
                    } else {
                        linesPtr[CAM_MF_NEIGHB-1][x]=source->borderConst[CAM_SIDE_LEFT_INDEX];
                    }
                } else {
                    // Get border pixels from the source frame
                    linesPtr[CAM_MF_NEIGHB-1][x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
            }

            // Fast transfer with memcpy
            if (iROI.srcinc==1) {
                memcpy(&linesPtr[CAM_MF_NEIGHB-1][x],srcptr,(source->width-left+CAM_MF_NEIGHB/2-x)<<(sizeof(CAM_PIXEL)-1));
                x=source->width-left+CAM_MF_NEIGHB/2;
            } else {
                for (;x<source->width-left+CAM_MF_NEIGHB/2;x++) {
                    // Get a pixel from the source frame
                    linesPtr[CAM_MF_NEIGHB-1][x]=*srcptr;
                    srcptr+=iROI.srcinc;
                }
            }
            for (;x<width+CAM_MF_NEIGHB-1;x++) {
                // Out of frame : fill with border color
                if (source->borderMode[CAM_SIDE_RIGHT_INDEX]==CAM_BORDER_REPLICATE) {
                    linesPtr[CAM_MF_NEIGHB-1][x]=*(srcptr+(source->width-1)*iROI.srcinc);
                } else {
                    linesPtr[CAM_MF_NEIGHB-1][x]=source->borderConst[CAM_SIDE_RIGHT_INDEX];
                }
            }
        }

	// Process all the pixels in the line
	for (x=CAM_MF_NEIGHB-1;x<width+CAM_MF_NEIGHB-1;x++) {

	    // Now, let's sort the pixels in the neighbourhood
	    first=0;
	    last=0;
	    nb=1;
	    i=x-CAM_MF_NEIGHB+1;
	    // Get the first pixel
	    value=linesPtr[0][i];
	    // And put it in the sorted list
	    values[0]=value;
	    next[0]=-1;

#define INSERT_VALUE(value)				\
	    values[nb]=value;				\
	    if (value>values[first]) {			\
		prev=first;				\
		j=next[first];				\
		while ((j!=-1)&&(value>values[j])) {	\
		    prev=j;				\
		    j=next[j];				\
		}					\
		if (j==-1) {				\
		    next[last]=nb;			\
		    last=nb;				\
		    next[nb]=-1;			\
		} else {				\
		    next[nb]=j;				\
		    next[prev]=nb;			\
		}					\
	    } else {					\
		next[nb]=first;				\
		first=nb;				\
	    }						\
	    nb++;			

    	    for (xp=1;xp<CAM_MF_NEIGHB;xp++) {
		value=linesPtr[0][i+xp];
		INSERT_VALUE(value);
	    }

	    for (yp=1;yp<CAM_MF_NEIGHB;yp++) {
		for (xp=0;xp<CAM_MF_NEIGHB;xp++) {
		    value=linesPtr[yp][i+xp];
		    INSERT_VALUE(value);
		}
	    }

	    // Retrieve the median value
	    j=first;
	    for (i=0;i<CAM_MF_NEIGHB*CAM_MF_NEIGHB/2;i++) {
		j=next[j];
	    }
	    result=values[j];

	    // Store the result in destination image
	    *dstptr=result;
            dstptr+=iROI.dstinc;
	}
	
	// Go to next line
	srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);
	
	// Reset neighborhood line pointers
	tmpptr=linesPtr[0];
	for (i=0;i<CAM_MF_NEIGHB-1;i++) {
	    linesPtr[i]=linesPtr[i+1];
	}
	linesPtr[CAM_MF_NEIGHB-1]=tmpptr;
    }

    camInternalROIPolicyExit(&iROI);
    return 1;
}

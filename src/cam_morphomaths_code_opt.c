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

/* Fundamental Morphological Mathematics Kernel
 * C code */

// Square3 optimized morphology kernel
#ifdef CAM_MM_OPT_SQUARE3
#ifdef CAM_MM_FINDLOCALMAXIMA
int camFindLocalMaximaSquare3(CamImage *source, CamKeypoints *points, int threshold)
#define CAM_MM_DO_DILATION
#else
#ifdef CAM_MM_GRADIENT
#define CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
int camMorphoGradientSquare3(CamImage *source, CamImage *dest)
#else
#ifdef CAM_MM_DO_EROSION
int camErodeSquare3(CamImage *source, CamImage *dest)
#else
int camDilateSquare3(CamImage *source, CamImage *dest)
#endif // CAM_MM_DO_EROSION
#endif // CAM_MM_GRADIENT
#endif // CAM_MM_FINDLOCALMAXIMA
#endif // CAM_MM_OPT_SQUARE3

// Circle5 optimized morphology kernel
#ifdef CAM_MM_OPT_CIRCLE5
#ifdef CAM_MM_FINDLOCALMAXIMA
int camFindLocalMaximaCircle5(CamImage *source, CamKeypoints *points, int threshold)
#define CAM_MM_DO_DILATION
#else
#ifdef CAM_MM_GRADIENT
#define CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
int camMorphoGradientCircle5(CamImage *source, CamImage *dest)
#else
#ifdef CAM_MM_DO_EROSION
int camErodeCircle5(CamImage *source, CamImage *dest)
#else
int camDilateCircle5(CamImage *source, CamImage *dest)
#endif // CAM_MM_DO_EROSION
#endif // CAM_MM_GRADIENT
#endif // CAM_MM_FINDLOCALMAXIMA
#endif // CAM_MM_OPT_CIRCLE5

// Circle7 optimized morphology kernel
#ifdef CAM_MM_OPT_CIRCLE7
#ifdef CAM_MM_FINDLOCALMAXIMA
int camFindLocalMaximaCircle7(CamImage *source, CamKeypoints *points, int threshold)
#define CAM_MM_DO_DILATION
#else
#ifdef CAM_MM_GRADIENT
#define CAM_MM_DO_EROSION
#define CAM_MM_DO_DILATION
int camMorphoGradientCircle7(CamImage *source, CamImage *dest)
#else
#ifdef CAM_MM_DO_EROSION
int camErodeCircle7(CamImage *source, CamImage *dest)
#else
int camDilateCircle7(CamImage *source, CamImage *dest)
#endif // CAM_MM_DO_EROSION
#endif // CAM_MM_GRADIENT
#endif // CAM_MM_FINDLOCALMAXIMA
#endif // CAM_MM_OPT_CIRCLE7
{
    int i,x,y,ntb,nlb;
    int width,height;
    int top,left;
    int firsttime;
    int acc=0;

    CamInternalROIPolicyStruct iROI;
    
#ifdef CAM_MM_DO_EROSION
    int tabero[9];
    int valero[2];
#endif // CAM_MM_DO_EROSION

#ifdef CAM_MM_DO_DILATION
    int tabdil[9];
    int valdil[2];
#endif // CAM_MM_DO_DILATION

    CAM_PIXEL *srcptr,*dstptr,*tmpptr,*cpsrcptr,*cpdstptr;
    unsigned CAM_PIXEL *linesPtr[CAM_MM_NEIGHB];
    CAM_PIXEL linesBuffer[CAM_MM_NEIGHB][CAM_MAX_SCANLINE+CAM_MM_NEIGHB-1];

    DECLARE_MASK_MANAGEMENT;

#ifdef CAM_MM_FINDLOCALMAXIMA
    int valcenter, xp, yp, zp, found;
#ifdef CAM_MM_OPT_CIRCLE7
    const int lines2test = 5;
    const int start2test[] = {2, 1, 0, 0, 0};
    const int end2test[] = {4, 5, 4, 2, 1};
#else
#ifdef CAM_MM_OPT_CIRCLE5
    const int lines2test = 4;
    const int start2test[] = {1, 0, 0, 0};
    const int end2test[] = {3, 3, 1, 0};
#else
    const int lines2test = 2;
    const int start2test[] = {0, 0};
    const int end2test[] = {2, 0};
#endif // CAM_MM_OPT_CIRCLE5
#endif // CAM_MM_OPT_CIRCLE7

    // ROI (Region Of Interest) management
    CAM_CHECK(camFindLocalMaxima, camInternalROIPolicy(source, NULL, &iROI, 1));
    CAM_CHECK_ARGS(camFindLocalMaxima, iROI.nChannels==1);
    CAM_CHECK_ARGS2(camFindLocalMaxima, points != NULL, "points destination parameter should not be set to NULL");
#else
    CamMorphoMathsKernel params;
    CAM_CHECK(camMorphoMathsKernel,camInternalROIPolicy(source, dest, &iROI, 1));
    CAM_CHECK_ARGS(camMorphoMathsKernel,(source->depth==dest->depth));
    CAM_CHECK_ARGS(camMorphoMathsKernel,iROI.nChannels==1);

    if (source->depth == CAM_DEPTH_1U) {
	// Binary processing
#ifdef CAM_MM_DO_EROSION
#ifdef CAM_MM_OPT_SQUARE3
	for (y=0;y<3;y++) {
	    for (x=0;x<3;x++) {
		params.erosionStructElt[y][x]=1;
	    }
	}
	return camErode3x3(source,dest,&params);
#endif
#ifdef CAM_MM_OPT_CIRCLE5
	for (y=0;y<5;y++) {
	    for (x=0;x<5;x++) {
		params.erosionStructElt[y][x]=1;
	    }
	}
	params.erosionStructElt[0][0]=0;
	params.erosionStructElt[0][4]=0;
	params.erosionStructElt[4][0]=0;
	params.erosionStructElt[4][4]=0;
	return camErode5x5(source,dest,&params);
#endif
#ifdef CAM_MM_OPT_CIRCLE7
	for (y=0;y<7;y++) {
	    for (x=0;x<7;x++) {
		params.erosionStructElt[y][x]=camCircle7StructElt[y][x];
	    }
	}
	return camErode7x7(source,dest,&params);
#endif
#else
#ifdef CAM_MM_OPT_SQUARE3
	for (y=0;y<3;y++) {
	    for (x=0;x<3;x++) {
		params.dilationStructElt[y][x]=1;
	    }
	}
	return camDilate3x3(source,dest,&params);
#endif
#ifdef CAM_MM_OPT_CIRCLE5
	for (y=0;y<5;y++) {
	    for (x=0;x<5;x++) {
		params.dilationStructElt[y][x]=1;
	    }
	}
	params.dilationStructElt[0][0]=0;
	params.dilationStructElt[0][4]=0;
	params.dilationStructElt[4][0]=0;
	params.dilationStructElt[4][4]=0;
	return camDilate5x5(source,dest,&params);
#endif
#ifdef CAM_MM_OPT_CIRCLE7
	for (y=0;y<7;y++) {
	    for (x=0;x<7;x++) {
		params.dilationStructElt[y][x]=camCircle7StructElt[y][x];
	    }
	}
	return camDilate7x7(source,dest,&params);
#endif
#endif // CAM_MM_DO_EROSION
    }
#endif // CAM_MM_FINDLOCALMAXIMA

    CAM_CHECK_ARGS(camMorphoMathsKernel,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camMorphoMathsKernel,(source->depth&CAM_DEPTH_MASK)>=8);
#ifndef CAM_MM_FINDLOCALMAXIMA
    CAM_CHECK_ARGS(camMorphoMathsKernel,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camMorphoMathsKernel,(dest->depth&CAM_DEPTH_MASK)>=8);
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
    CAM_CHECK_ARGS(camMorphoMathsKernel,(width>CAM_MM_NEIGHB/2));
    CAM_CHECK_ARGS(camMorphoMathsKernel,(height>CAM_MM_NEIGHB/2));    
	
    // Mask management
    INIT_MASK_MANAGEMENT;

    // Initialize algorithm
    for (i=0;i<CAM_MM_NEIGHB;i++) {
	linesPtr[i]=linesBuffer[i];
    }
    
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
	firsttime=1;

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

	if (source->depth & CAM_DEPTH_SIGN) {
#define linesPtr ((signed CAM_PIXEL**) linesPtr)
	    BEGIN_MASK_MANAGEMENT(dstptr=cpdstptr+startx*iROI.dstinc;)

		// Process all the pixels in the line
		for (x=startx+CAM_MM_NEIGHB-1;x<endx+CAM_MM_NEIGHB-1;x++) {

#ifdef CAM_MM_DO_EROSION
		    {
#define tabX tabero
#define valX valero
#define XOP < 
#ifdef CAM_MM_OPT_CIRCLE7
#include "cam_morphomaths_code_circle7.c"
#else
#ifdef CAM_MM_OPT_CIRCLE5
#include "cam_morphomaths_code_circle5.c"
#else
#include "cam_morphomaths_code_square3.c"
#endif // CAM_MM_OPT_CIRCLE5
#endif // CAM_MM_OPT_CIRCLE7
#undef tabX
#undef valX
#undef XOP
		    }
#endif // CAM_MM_DO_EROSION

#ifdef CAM_MM_DO_DILATION
		    {
#define tabX tabdil
#define valX valdil
#define XOP > 
#ifdef CAM_MM_OPT_CIRCLE7
#include "cam_morphomaths_code_circle7.c"
#else
#ifdef CAM_MM_OPT_CIRCLE5
#include "cam_morphomaths_code_circle5.c"
#else
#include "cam_morphomaths_code_square3.c"
#endif // CAM_MM_OPT_CIRCLE5
#endif // CAM_MM_OPT_CIRCLE7
#undef tabX
#undef valX
#undef XOP
		    }
#endif // CAM_MM_DO_DILATION
		    firsttime=0;

#ifdef CAM_MM_FINDLOCALMAXIMA
		    valcenter = linesPtr[CAM_MM_NEIGHB / 2][x - CAM_MM_NEIGHB + 1 + CAM_MM_NEIGHB / 2];	
		    if (valdil[0] == valcenter && valcenter >= threshold && valcenter != 0) {
			// Check that it is the only point with this maximum (in the upper part)
			for (yp = 0, zp = 0, found = 0; yp < lines2test; yp++) {
			    for (xp = start2test[yp]; xp <= end2test[yp]; xp++) {
				if (linesPtr[yp][x - CAM_MM_NEIGHB + 1 + xp] == valcenter) {
				    found = 1;
				    break;
				}
			    }
			}
			if (!found) {
			    // Keep this point : this is a local maximum
			    if (points->nbPoints < points->allocated) {
				points->keypoint[points->nbPoints]->x = x - CAM_MM_NEIGHB + 1;
				points->keypoint[points->nbPoints]->y = y;
				points->keypoint[points->nbPoints]->scale = 0;
				points->keypoint[points->nbPoints]->angle = 0;
				points->keypoint[points->nbPoints]->value = valcenter;
				points->nbPoints++;
				acc += valdil[0];
			    } else {
				camInternalROIPolicyExit(&iROI);
				return 0;
			    }
			}
		    }
#else	
#ifdef CAM_MM_GRADIENT
		    acc+=(*dstptr=valdil[0]-valero[0]);
#else
#ifdef CAM_MM_DO_EROSION
		    acc+=(*dstptr=valero[0]);
#else
		    acc+=(*dstptr=valdil[0]);
#endif
#endif // CAM_MM_GRADIENT
		    dstptr+=iROI.dstinc;
#endif // CAM_MM_FINDLOCALMAXIMA
		}

	    END_MASK_MANAGEMENT;       
#undef linesPtr
	} else {
	    BEGIN_MASK_MANAGEMENT(dstptr=cpdstptr+startx*iROI.dstinc;)

		// Process all the pixels in the line
		for (x=startx+CAM_MM_NEIGHB-1;x<endx+CAM_MM_NEIGHB-1;x++) {

#ifdef CAM_MM_DO_EROSION
		    {
#define tabX tabero
#define valX valero
#define XOP < 
#ifdef CAM_MM_OPT_CIRCLE7
#include "cam_morphomaths_code_circle7.c"
#else
#ifdef CAM_MM_OPT_CIRCLE5
#include "cam_morphomaths_code_circle5.c"
#else
#include "cam_morphomaths_code_square3.c"
#endif // CAM_MM_OPT_CIRCLE5
#endif // CAM_MM_OPT_CIRCLE7
#undef tabX
#undef valX
#undef XOP
		    }
#endif // CAM_MM_DO_EROSION

#ifdef CAM_MM_DO_DILATION
		    {
#define tabX tabdil
#define valX valdil
#define XOP > 
#ifdef CAM_MM_OPT_CIRCLE7
#include "cam_morphomaths_code_circle7.c"
#else
#ifdef CAM_MM_OPT_CIRCLE5
#include "cam_morphomaths_code_circle5.c"
#else
#include "cam_morphomaths_code_square3.c"
#endif // CAM_MM_OPT_CIRCLE5
#endif // CAM_MM_OPT_CIRCLE7
#undef tabX
#undef valX
#undef XOP
		    }
#endif // CAM_MM_DO_DILATION
		    firsttime=0;

#ifdef CAM_MM_FINDLOCALMAXIMA
		    valcenter = linesPtr[CAM_MM_NEIGHB / 2][x - CAM_MM_NEIGHB + 1 + CAM_MM_NEIGHB / 2];	
		    if (valdil[0] == valcenter && valcenter >= threshold && valcenter != 0) {
			// Check that it is the only point with this maximum (in the upper part)
			for (yp = 0, zp = 0, found = 0; yp < lines2test; yp++) {
			    for (xp = start2test[yp]; xp <= end2test[yp]; xp++) {
				if (linesPtr[yp][x - CAM_MM_NEIGHB + 1 + xp] == valcenter) {
				    found = 1;
				    break;
				}
			    }
			}
			if (!found)
			{
			    // Keep this point : this is a local maximum
			    if (points->nbPoints < points->allocated) {
				points->keypoint[points->nbPoints]->x = x - CAM_MM_NEIGHB + 1;
				points->keypoint[points->nbPoints]->y = y;
				points->keypoint[points->nbPoints]->scale = 0;
				points->keypoint[points->nbPoints]->angle = 0;
				points->keypoint[points->nbPoints]->value = valcenter;
				points->nbPoints++;
				acc += valdil[0];
			    } else {
				camInternalROIPolicyExit(&iROI);
				return 0;
			    }
			}
		    }
#else	
#ifdef CAM_MM_GRADIENT
		    acc+=(*dstptr=valdil[0]-valero[0]);
#else
#ifdef CAM_MM_DO_EROSION
		    acc+=(*dstptr=valero[0]);
#else
		    acc+=(*dstptr=valdil[0]);
#endif
#endif // CAM_MM_GRADIENT
		    dstptr+=iROI.dstinc;
#endif // CAM_MM_FINDLOCALMAXIMA
		}

	    END_MASK_MANAGEMENT;       
	}

	// Go to next line
	srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
#ifndef CAM_MM_FINDLOCALMAXIMA
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);
#endif

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

#ifdef CAM_MM_GRADIENT
#undef CAM_MM_DO_DILATION
#undef CAM_MM_DO_EROSION
#endif
#ifdef CAM_MM_FINDLOCALMAXIMA
#undef CAM_MM_DO_EROSION
#endif

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

/* RLE morpho/erosion code
 * C code */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "camellia.h"
#include "camellia_internals.h"

#define parent blob

int camRLEErodeCross(CamRLEImage *source, CamRLEImage *dest)
{
    CamRun *l1, *l2, *l3;       // Three pointers for the upper line, current line, lower line runs
    CamRun *out=dest->runs;	// A point to the next run in the dest image
    int outRuns=0;		// The number of runs in dest
    CamRun runInConstruction;	// The run that we are currently making for the RLE eroded image
    CamRun currentRun;		// The run we are currently examining
    int x1=0,x2=0,x3=0;	        // The current position for the runs on upper line, current line, lower line
    CAM_RLE_INT_TYPE previousColor=0xffff; // The value of the previous run
    int y=0;			// The current line being eroded
    CAM_RLE_INT_TYPE value;
    int finished;
    int erodeRightPixel;
    CAM_RLE_INT_TYPE colorRightPixel=0;
    int length;
    int z1;
    int z3;
    CamRun dummyRun[2];
                 
    if (source->nbRuns==0) {
	dest->nbRuns=0;
	return 0;
    }
      
    dest->height=source->height;
    dest->width=source->width;

    // Put a run at the beginning to have a reference starting point
    out->value=-1;
    out->length=0;
    out->parent=-1;
    out->line=-1;
    outRuns=1;
    out++;

    dummyRun[0].length=source->width;
    dummyRun[0].value=0xffff; 
    dummyRun[1].length=source->width;
    dummyRun[1].value=0xffff;
    // Prepare for erosion
    l1=&dummyRun[0];
    l2=source->runs;
    // Lower scan begins on second line, so skip over first
    l3=l2;
    while (x3<source->width) {
        x3 += (l3++)->length;
    }
    x3=0;
    
    // Set up the currentRun
    currentRun=*(l2++);
    // And the current run in construction
    runInConstruction.length=0;
    runInConstruction.value=0;
    
    // Well, we are now ready to erode the image
    do {
        // Process the currentRun
        if (currentRun.value!=0) {
            // First of all, do we have to erode the pixel on the left?
            if (previousColor<currentRun.value) {
                // We must erode this pixel
                // Is this also a right pixel or a pixel on the right border?
                if (currentRun.length==1) {
                    // Yes, let's see if we must choose the left value (previousColor) or the right value
                    if (x2+1==source->width) {
                        value=previousColor;
                    } else if (l2->value<previousColor) {
                        value=l2->value;
                    } else {
                        value=previousColor;
                    }
                } else {
                    value=previousColor;
                }
                
                // Let's have a look above
                if (l1->value<value) {
                    value=l1->value;
                }
                // Let's have a look below
                if (l3->value<value) {
                    value=l3->value;
                }
                
                // Let's append this single pixel to the current run in construction
                if (runInConstruction.length!=0) {
                    // There is a run in construction, so let's see if it's the value of our erosion
                    if (runInConstruction.value!=value) {
                        // Well, we must transfer the current run in construction to the image
                        out->value=runInConstruction.value;
                        out->length=runInConstruction.length;
                        out->line=y;
                        out->parent=outRuns++;
                        out++;
                        if (outRuns+1>=dest->allocated-dest->width) {
                            dest->nbRuns=0;
                            return 0;
                        }
                        // And set this new run in construction (a single pixel)
                        runInConstruction.value=value;
                        runInConstruction.length=1;
                    } else {
                        // We can append the single eroded pixel to the run in construction
                        runInConstruction.length++;
                    }
                } else {
                    // We have no run in construction
                    // Let's build one (a single pixel)
                    runInConstruction.value=value;
                    runInConstruction.length=1;
                }
                // Let's move one pixel to the right
                x2++;
                // Consume one pixel
                currentRun.length--;
                // Let's compute the new (l1,x1) to follow the current run
                while (x1+l1->length-1<x2) {
                    x1 += (l1++)->length; 
                }
                // Let's compute the new (l3,x3) to follow the current run
                while (x3+l3->length-1<x2) {
                    x3 += (l3++)->length;
                }			
            }
            // Let's update the previous value
            previousColor=currentRun.value;
            if (currentRun.length!=0) {
                finished=0;
                erodeRightPixel=0;
                // Then, we erode or reproduce the remaining pixels of the run, until all of them have been consumed
                do {
                    z1=l1->length+x1-x2;
                    z3=l3->length+x3-x2;
                    length=(z1<z3)?z1:z3;
                    value=(l1->value<l3->value)?l1->value:l3->value;
                    value=(value<currentRun.value)?value:currentRun.value;
                    
                    if (length>=currentRun.length) {
                        finished=1; // We will have consumed all the pixels of the run
                        // The pixel on the right of the run is concerned. 
                        // Let's see if we must erode it
                        if ((x2+currentRun.length!=source->width)&&(l2->value<value)) {
                            // We must erode it
                            if (currentRun.length==1) {
                                length=1;
                                value=l2->value;
                            } else {
                                length=currentRun.length-1;
                                erodeRightPixel=1;
                                colorRightPixel=l2->value;
                            }
                        } else {
                            // OK. No problem with the right pixel
                            length=currentRun.length; // We must erode or keep all the pixels, but not more!
                        }
                    }
                    
                    // Let's append these pixels to the current run in construction
                    if (runInConstruction.length!=0) {
                        // There is a run in construction, so let's see if it's a hole
                        if (runInConstruction.value!=value) {
                            // Well, we must transfer the current run in construction to the image
                            out->value=runInConstruction.value;
                            out->length=runInConstruction.length;
                            out->line=y;
                            out->parent=outRuns++;
                            out++;
                            if (outRuns+1>=dest->allocated-dest->width) {
                                dest->nbRuns=0;
                                return 0;
                            }
                            // And set this new run in construction
                            runInConstruction.value=value;
                            runInConstruction.length=length;
                        } else {
                            // We can append the eroded pixels to the run in construction
                            runInConstruction.length+=length;
                        }
                    } else {
                        // We have no run in construction
                        // Let's build one
                        runInConstruction.value=value;
                        runInConstruction.length=length;
                    }
                    // Let's move to the right
                    x2+=length;
                    // And consume the pixels
                    currentRun.length-=length;
                    // Let's compute the new (l1,x1) to follow the current run
                    while (x1+l1->length-1<x2) {
                        x1 += (l1++)->length; 
                    }
                    // Let's compute the new (l3,x3) to follow the current run
                    while (x3+l3->length-1<x2) {
                        x3 += (l3++)->length;
                    }								
                } while (!finished);
                // Let's see if we must erode the right pixel
                if (erodeRightPixel) {
                    // Let's append these pixels to the current run in construction
                    if (runInConstruction.length!=0) {
                        // There is a run in construction, so let's see if it's a hole
                        if (runInConstruction.value!=colorRightPixel) {
                            // Well, we must transfer the current run in construction to the image
                            out->value=runInConstruction.value;
                            out->length=runInConstruction.length;
                            out->line=y;
                            out->parent=outRuns++;
                            out++;
                            if (outRuns+1>=dest->allocated-dest->width) {
                                dest->nbRuns=0;
                                return 0;
                            }
                            // And set this new run in construction
                            runInConstruction.value=colorRightPixel;
                            runInConstruction.length=1;
                        } else {
                            // We can append the eroded right pixel to the run in construction
                            runInConstruction.length++;
                        }
                    } else {
                        // We have no run in construction
                        // Let's build one
                        runInConstruction.value=colorRightPixel;
                        runInConstruction.length=1;
                    }
                    // Let's move to the right
                    x2++;
                }
            }
        } else {
            // This run is a hole
            // Let's append it to the run in construction
            if (runInConstruction.length!=0) {
                // There is a run in construction, so let's see if it's a hole
                if (runInConstruction.value!=0) {
                    // Well, we must transfer the current run in construction to the image
                    out->value=runInConstruction.value;
                    out->length=runInConstruction.length;
                    out->line=y;
                    out->parent=outRuns++;
                    out++;
                    if (outRuns+1>=dest->allocated-dest->width) {
                        dest->nbRuns=0;
                        return 0;
                    }
                    // And set this new run in construction
                    runInConstruction.value=0;
                    runInConstruction.length=currentRun.length;
                } else {
                    // We can append the currentRun to the runInConstruction
                    runInConstruction.length+=currentRun.length;
                }
            } else {
                // We have no run in construction
                // Let's build one
                runInConstruction.value=0;
                runInConstruction.length=currentRun.length;
            }
            // Let's update the previous value
            previousColor=0;
            // Let's move
            x2+=currentRun.length;
            currentRun.length=0;
        }
        // We have consumed the current run
        // Have we reached the source->width of the picture?
        if (x2==source->width) {
            // Yes, we must transfer the current run in construction to the result,
            out->value=runInConstruction.value;
            out->length=runInConstruction.length;
            out->line=y;
            out->parent=outRuns++;
            out++;
            if (outRuns+1>=dest->allocated-dest->width) {
                dest->nbRuns=0;
                return 0;
            }
            // Go to next line
            if (y==0) {
                y=1;
                l1=source->runs+1;
                x1=0;
                // (l3,x3): Go to the next line
                while (x3<source->width) {
                    x3 += (l3++)->length;
                }
                x3=0;
            } else {
                y++;
                if (y>=source->height-1) {
                    if (y==source->height) break; // We have finished
                    // It's the last line
                    x3=0;
                    l3=&dummyRun[0];
                } else {
                    // (l3,x3): Go to the next line
                    while (x3<source->width) {
                        x3 += (l3++)->length;
                    }
                    x3=0;
                }
                // (l1,x1): Go to the next line
                while (x1<source->width) {
                    x1 += (l1++)->length;
                }
                x1=0;
            }
            // Start a new empty run in construction
            runInConstruction.length=0;
            // We're at the beginning of the next line
            x2=0;
            // Let's set the previous value, since we are beginning a new line
            previousColor=0xffff;
            // And take a new run
            currentRun=*(l2++);
        } else {
            // Let's take a new current run
            currentRun=*(l2++);
            // Let's compute the new (l1,x1) to follow the current run
            while (x1+l1->length-1<x2) {
                x1 += (l1++)->length;
            }
            // Let's compute the new (l3,x3) to follow the current run
            while (x3+l3->length-1<x2) {
                x3 += (l3++)->length;
            }		
        }
    } while (1);
    dest->nbRuns=outRuns;

    // Add one last run, just to ease blob analysis/reconstruction
    out->value=-1;
    out->length=0;
    out->parent=-1;
    out->line=-1;
    if (outRuns+1>=dest->allocated-dest->width) {
	dest->nbRuns=0;
	return 0;
    }

    return 1;
}

int camRLEErode3x3(CamRLEImage *source, CamRLEImage *dest)
{
    CamRun *l1, *l2, *l3;       // Three pointers for the upper line, current line, lower line
    CamRun *out=dest->runs;	// A point to the next run in the dest image
    int outRuns=0;		// The number of runs in dest
    CamRun runInConstruction;	// The run that we are currently making for the RLE eroded image
    CamRun currentRun;		// The run we are currently examining
    int x1=0,x2=0,x3=0;	        // The current position for the runs on upper line, current line, lower line
    CAM_RLE_INT_TYPE previousColor=0xffff; // The value of the previous run
    int y=0;			// The current line being eroded
    int upperLeftInL1=1,lowerLeftInL3=1;
    CAM_RLE_INT_TYPE value;
    int finished;
    int erodeRightPixel;
    CAM_RLE_INT_TYPE colorRightPixel=0;
    int length;
    int z1;
    int z3;
    CamRun dummyRun[2];
    
    if (source->nbRuns==0) {
	dest->nbRuns=0;
	return 0;
    }

                   
    dest->height=source->height;
    dest->width=source->width;

    // Put a run at the begin to have a reference starting point
    out->value=-1;
    out->length=0;
    out->parent=-1;
    out->line=-1;
    outRuns=1;
    out++;

    dummyRun[0].length=source->width;
    dummyRun[0].value=0xffff; 
    dummyRun[1].length=source->width;
    dummyRun[1].value=0xffff;
    // Prepare for erosion
    l1=&dummyRun[0];
    l2=source->runs+1;
    // Lower scan begins on second line, so skip over first
    l3=l2;
    while (x3<source->width) {
        x3 += (l3++)->length;
    }
    x3=0;
    
    // Set up the currentRun
    currentRun=*(l2++);
    // And the current run in construction
    runInConstruction.length=0;
    runInConstruction.value=0;
    
    // Well, we are now ready to erode the image
    do {
        // Process the currentRun
        if (currentRun.value!=0) {
            // First of all, do we have to erode the pixel on the left?
            if (previousColor<currentRun.value) {
                // We must erode this pixel
                // Is this also a right pixel or a pixel on the right border?
                if (currentRun.length==1) {
                    // Yes, let's see if we must choose the left value (previousColor) or the right value
                    if (x2+1==source->width) { // We're on the right of the picture
                        value=previousColor;
                    } else if (l2->value<previousColor) {
                        value=l2->value;
                    } else {
                        value=previousColor;
                    }
                } else {
                    value=previousColor;
                }
                
                // Let's have a look above
                if (l1->value<value) {
                    value=l1->value;
                }
                if ((!upperLeftInL1)&&((l1-1)->value<value)) {
                    value=(l1-1)->value;
                }
                if (x1+l1->length-1==x2) {
                    // We must compare with the upper right pixel value
                    if ((x2+1!=source->width)&&((l1+1)->value<value)) {
                        value=(l1+1)->value;
                    }
                }
                // Let's have a look below
                if (l3->value<value) {
                    value=l3->value;
                }
                if ((!lowerLeftInL3)&&((l3-1)->value<value)) {
                    value=(l3-1)->value;
                }
                if (x3+l3->length-1==x2) {
                    // We must compare with the lower right pixel value
                    if ((x2+1!=source->width)&&((l3+1)->value<value)) {
                        value=(l3+1)->value;
                    }
                }
                
                // Let's append this single pixel to the current run in construction
                if (runInConstruction.length!=0) {
                    // There is a run in construction, so let's see if it is the value of our erosion
                    if (runInConstruction.value!=value) {
                        // Well, we must transfer the current run in construction to the image
                        out->value=runInConstruction.value;
                        out->length=runInConstruction.length;
                        out->line=y;
                        out->parent=outRuns++;
                        out++;
                        if (outRuns+1>=dest->allocated-dest->width) {
                            dest->nbRuns=0;
                            return 0;
                        }
                        // And set this new run in construction (a single pixel)
                        runInConstruction.value=value;
                        runInConstruction.length=1;
                    } else {
                        // We can append the single eroded pixel to the run in construction
                        runInConstruction.length++;
                    }
                } else {
                    // We have no run in construction
                    // Let's build one (a single pixel)
                    runInConstruction.value=value;
                    runInConstruction.length=1;
                }
                // Let's move one pixel to the right
                x2++;
                // Consume one pixel
                currentRun.length--;
                // Let's compute the new (l1,x1) to follow the current run
                while (x1+l1->length-1<x2) {
                    x1 += (l1++)->length; 
                }
                upperLeftInL1=(x1!=x2);
                // Let's compute the new (l3,x3) to follow the current run
                while (x3+l3->length-1<x2) {
                    x3 += (l3++)->length;
                }			
                lowerLeftInL3=(x3!=x2);
            }
            // Let's update the previous value
            previousColor=currentRun.value;
            if (currentRun.length!=0) {
                finished=0;
                erodeRightPixel=0;
                // Then, we erode or reproduce the remaining pixels of the run, until all of them have been consumed
                do {
                    z1=l1->length+x1-x2;
                    z3=l3->length+x3-x2;
                    length=((z1<z3)?z1:z3)-1;
                    value=(l1->value<l3->value)?l1->value:l3->value;
                    value=(value<currentRun.value)?value:currentRun.value;
                    
                    if (length==0) {
                        // It means that I'm not sure about the upper right or lower right pixels
                        if (x2+1!=source->width) { 
                            if (z1==1) { // I must look at the upper right pixel value
                                if ((l1+1)->value<value) {
                                    value=(l1+1)->value;
                                }
                            }
                            if (z3==1) { // I must look at the upper right pixel value
                                if ((l3+1)->value<value) {
                                    value=(l3+1)->value;
                                }
                            }
                        }
                        length=1;
                    }
                    
                    // Let's finally consider the upper left and lower left pixels
                    if ((!upperLeftInL1)&&((l1-1)->value<value)) {
                        value=(l1-1)->value;
                        length=1;
                    }
                    if ((!lowerLeftInL3)&&((l3-1)->value<value)) {
                        value=(l3-1)->value;
                        length=1;
                    }
                    
                    if (length>=currentRun.length) {
                        finished=1; // We will have consumed all the pixels of the run
                        // The pixel on the right of the run is concerned. 
                        // Let's see if we must erode it
                        if ((x2+currentRun.length!=source->width)&&(l2->value<value)) {
                            // We must erode it
                            if (currentRun.length==1) {
                                length=1;
                                value=l2->value;
                            } else {
                                length=currentRun.length-1;
                                erodeRightPixel=1;
                                colorRightPixel=l2->value;
                            }
                        } else {
                            // OK. No problem with the right pixel
                            length=currentRun.length; // We must erode or keep all the pixels, but not more!
                        }
                    }
                    
                    // Let's append these pixels to the current run in construction
                    if (runInConstruction.length!=0) {
                        // There is a run in construction, so let's see if it's a hole
                        if (runInConstruction.value!=value) {
                            // Well, we must transfer the current run in construction to the image
                            out->value=runInConstruction.value;
                            out->length=runInConstruction.length;
                            out->line=y;
                            out->parent=outRuns++;
                            out++;
                            if (outRuns+1>=dest->allocated-dest->width) {
                                dest->nbRuns=0;
                                return 0;
                            }
                            
                            // And set this new run in construction
                            runInConstruction.value=value;
                            runInConstruction.length=length;
                        } else {
                            // We can append the eroded pixels to the run in construction
                            runInConstruction.length+=length;
                        }
                    } else {
                        // We have no run in construction
                        // Let's build one
                        runInConstruction.value=value;
                        runInConstruction.length=length;
                    }
                    // Let's move to the right
                    x2+=length;
                    // And consume the pixels
                    currentRun.length-=length;
                    // Let's compute the new (l1,x1) to follow the current run
                    while (x1+l1->length-1<x2) {
                        x1 += (l1++)->length; 
                    }
                    upperLeftInL1=(x1!=x2);
                    // Let's compute the new (l3,x3) to follow the current run
                    while (x3+l3->length-1<x2) {
                        x3 += (l3++)->length;
                    }								
                    lowerLeftInL3=(x3!=x2);
                } while (!finished);
                // Let's see if we must erode the right pixel
                if (erodeRightPixel) {
                    // Let's append these pixels to the current run in construction
                    if (runInConstruction.length!=0) {
                        // There is a run in construction, so let's see if it's a hole
                        if (runInConstruction.value!=colorRightPixel) {
                            // Well, we must transfer the current run in construction to the image
                            out->value=runInConstruction.value;
                            out->length=runInConstruction.length;
                            out->line=y;
                            out->parent=outRuns++;
                            out++;
                            if (outRuns+1>=dest->allocated-dest->width) {
                                dest->nbRuns=0;
                                return 0;
                            }

                            // And set this new run in construction
                            runInConstruction.value=colorRightPixel;
                            runInConstruction.length=1;
                        } else {
                            // We can append the eroded right pixel to the run in construction
                            runInConstruction.length++;
                        }
                    } else {
                        // We have no run in construction
                        // Let's build one
                        runInConstruction.value=colorRightPixel;
                        runInConstruction.length=1;
                    }
                    // Let's move to the right
                    x2++;
                }
            }
        } else {
            // This run is a hole
            // Let's append it to the run in construction
            if (runInConstruction.length!=0) {
                // There is a run in construction, so let's see if it's a hole
                if (runInConstruction.value!=0) {
                    // Well, we must transfer the current run in construction to the image
                    out->value=runInConstruction.value;
                    out->length=runInConstruction.length;
                    out->line=y;
                    out->parent=outRuns++;
                    out++;
                    if (outRuns+1>=dest->allocated-dest->width) {
                        dest->nbRuns=0;
                        return 0;
                    }

                    // And set this new run in construction
                    runInConstruction.value=0;
                    runInConstruction.length=currentRun.length;
                } else {
                    // We can append the currentRun to the runInConstruction
                    runInConstruction.length+=currentRun.length;
                }
            } else {
                // We have no run in construction
                // Let's build one
                runInConstruction.value=0;
                runInConstruction.length=currentRun.length;
            }
            // Let's update the previous value
            previousColor=0;
            // Let's move
            x2+=currentRun.length;
        }
        // We have consumed the current run
        // Have we reached the source->width of the picture?
        if (x2==source->width) {
            // Yes, we must transfer the current run in construction to the result,
            out->value=runInConstruction.value;
            out->length=runInConstruction.length;
            out->line=y;
            out->parent=outRuns++;
            out++;
            if (outRuns+1>=dest->allocated-dest->width) {
                dest->nbRuns=0;
                return 0;
            }

            // Go to next line
            if (y==0) {
                y=1;
                l1=source->runs+1;
                x1=0;
                // (l3,x3): Go to the next line
                while (x3<source->width) {
                    x3 += (l3++)->length;
                }
                x3=0;
            } else {
                y++;
                if (y>=source->height-1) {
                    if (y==source->height) break; // We have finished
                    // It's the last line
                    x3=0;
                    l3=&dummyRun[0];
                } else {
                    // (l3,x3): Go to the next line
                    while (x3<source->width) {
                        x3 += (l3++)->length;
                    }
                    x3=0;
                }
                // (l1,x1): Go to the next line
                while (x1<source->width) {
                    x1 += (l1++)->length;
                }
                x1=0;
            }
            // Start a new empty run in construction
            runInConstruction.length=0;
            // We're at the beginning of the next line
            x2=0;
            // Let's set the previous value, since we are beginning a new line
            previousColor=0xffff;
            // And take a new run
            currentRun=*(l2++);
            // And tell that the upper and lower left pixels are in l1
            upperLeftInL1=1;
            lowerLeftInL3=1;
        } else {
            // Let's take a new current run
            currentRun=*(l2++);
            // Let's compute the new (l1,x1) to follow the current run
            while (x1+l1->length-1<x2) {
                x1 += (l1++)->length;
            }
            upperLeftInL1=(x1!=x2);
            // Let's compute the new (l3,x3) to follow the current run
            while (x3+l3->length-1<x2) {
                x3 += (l3++)->length;
            }		
            lowerLeftInL3=(x3!=x2);
        }
    } while (1);
    dest->nbRuns=outRuns;

    // Add one last run, just to ease blob analysis/reconstruction
    out->value=-1;
    out->length=0;
    out->parent=-1;
    out->line=-1;
    if (outRuns+1>=dest->allocated-dest->width) {
	dest->nbRuns=0;
	return 0;
    }

    return 1;
}

int camRLEErode3x2(CamRLEImage *source, CamRLEImage *dest)
{
    CamRun *l1, *l2;		// Two pointers for the upper line and the current line
    CamRun *out=dest->runs;	// A point to the next run in the dest image
    int outRuns;		// The number of runs in dest
    CamRun runInConstruction;   // The run that we are currently making for the RLE eroded image
    CamRun currentRun;		// The run we are currently examining
    int x1=0,x2=0;		// The current position for the runs on upper line and current line
    CAM_RLE_INT_TYPE previousColor=0xffff; // The value of the previous run
    int y=0;			// The current line being eroded
    int upperLeftInL1=1;
    CAM_RLE_INT_TYPE value;
    int finished;
    int erodeRightPixel;
    CAM_RLE_INT_TYPE colorRightPixel=0;
    int length;
    int z1;
    CamRun dummyRun[2];
    
    if (source->nbRuns==0) {
	dest->nbRuns=0;
	return 0;
    }

    dest->height=source->height;
    dest->width=source->width;

    // Put a run at the beginning to have a reference starting point
    out->value=-1;
    out->length=0;
    out->parent=-1;
    out->line=-1;
    outRuns=1;
    out++;

    dummyRun[0].length=source->width;
    dummyRun[0].value=0xffff;
    dummyRun[1].length=source->width;
    dummyRun[1].value=0xffff;
    // Prepare for erosion
    l1=&dummyRun[0];
    l2=source->runs+1;
    
    // Set up the currentRun
    currentRun=*(l2++);
    // And the current run in construction
    runInConstruction.length=0;
    runInConstruction.value=0;
    
    // Well, we are now ready to erode the image
    do {
        // Process the currentRun
        if (currentRun.value!=0) {
            // First of all, do we have to erode the pixel on the left?
            if (previousColor<currentRun.value) {
                // We must erode this pixel
                // Is this also a right pixel or a pixel on the right border?
                if (currentRun.length==1) {
                    // Yes, let's see if we must choose the left value (previousColor) or the right value
                    if (x2+1==source->width) { // We're on the right side of the picture
                        value=previousColor;
                    } else if (l2->value<previousColor) {
                        value=l2->value;
                    } else {
                        value=previousColor;
                    }
                } else {
                    value=previousColor;
                }
                
                // Let's have a look above
                if (l1->value<value) {
                    value=l1->value;
                }
                if ((!upperLeftInL1)&&((l1-1)->value<value)) {
                    value=(l1-1)->value;
                }
                if (x1+l1->length-1==x2) {
                    // We must compare with the upper right pixel value
                    if ((x2+1!=source->width)&&((l1+1)->value<value)) {
                        value=(l1+1)->value;
                    }
                }
                
                // Let's append this single pixel to the current run in construction
                if (runInConstruction.length!=0) {
                    // There is a run in construction, so let's see if it is the value of our erosion
                    if (runInConstruction.value!=value) {
                        // Well, we must transfer the current run in construction to the image
                        out->value=runInConstruction.value;
                        out->length=runInConstruction.length;
                        out->line=y;
                        out->parent=outRuns++;
                        out++;
                        if (outRuns+1>=dest->allocated-dest->width) {
                            dest->nbRuns=0;
                            return 0;
                        }

                        // And set this new run in construction (a single pixel)
                        runInConstruction.value=value;
                        runInConstruction.length=1;
                    } else {
                        // We can append the single eroded pixel to the run in construction
                        runInConstruction.length++;
                    }
                } else {
                    // We have no run in construction
                    // Let's build one (a single pixel)
                    runInConstruction.value=value;
                    runInConstruction.length=1;
                }
                // Let's move one pixel to the right
                x2++;
                // Consume one pixel
                currentRun.length--;
                // Let's compute the new (l1,x1) to follow the current run
                while (x1+l1->length-1<x2) {
                    x1 += (l1++)->length; 
                }
                upperLeftInL1=(x1!=x2);
            }
            // Let's update the previous value
            previousColor=currentRun.value;
            if (currentRun.length!=0) {
                finished=0;
                erodeRightPixel=0;
                // Then, we erode or reproduce the remaining pixels of the run, until all of them have been consumed
                do {
                    z1=l1->length+x1-x2;
                    length=z1-1;
                    value=l1->value;
                    value=(value<currentRun.value)?value:currentRun.value;
                    
                    if (length==0) {
                        // It means that I'm not sure about the upper right or lower right pixels
                        if (x2+1!=source->width) { 
                            if (z1==1) { // I must look at the upper right pixel value
                                if ((l1+1)->value<value) {
                                    value=(l1+1)->value;
                                }
                            }
                        }
                        length=1;
                    }
                    
                    // Let's finally consider the upper left and lower left pixels
                    if ((!upperLeftInL1)&&((l1-1)->value<value)) {
                        value=(l1-1)->value;
                        length=1;
                    }
                    if (length>=currentRun.length) {
                        finished=1; // We will have consumed all the pixels of the run
                        // The pixel on the right of the run is concerned. 
                        // Let's see if we must erode it
                        if ((x2+currentRun.length!=source->width)&&(l2->value<value)) {
                            // We must erode it
                            if (currentRun.length==1) {
                                length=1;
                                value=l2->value;
                            } else {
                                length=currentRun.length-1;
                                erodeRightPixel=1;
                                colorRightPixel=l2->value;
                            }
                        } else {
                            // OK. No problem with the right pixel
                            length=currentRun.length; // We must erode or keep all the pixels, but not more!
                        }
                    }
                    
                    // Let's append these pixels to the current run in construction
                    if (runInConstruction.length!=0) {
                        // There is a run in construction, so let's see if it's a hole
                        if (runInConstruction.value!=value) {
                            // Well, we must transfer the current run in construction to the image
                            out->value=runInConstruction.value;
                            out->length=runInConstruction.length;
                            out->line=y;
                            out->parent=outRuns++;
                            out++;
                            if (outRuns+1>=dest->allocated-dest->width) {
                                dest->nbRuns=0;
                                return 0;
                            }
                            
                            // And set this new run in construction
                            runInConstruction.value=value;
                            runInConstruction.length=length;
                        } else {
                            // We can append the eroded pixels to the run in construction
                            runInConstruction.length+=length;
                        }
                    } else {
                        // We have no run in construction
                        // Let's build one
                        runInConstruction.value=value;
                        runInConstruction.length=length;
                    }
                    // Let's move to the right
                    x2+=length;
                    // And consume the pixels
                    currentRun.length-=length;
                    // Let's compute the new (l1,x1) to follow the current run
                    while (x1+l1->length-1<x2) {
                        x1 += (l1++)->length; 
                    }
                    upperLeftInL1=(x1!=x2);
                } while (!finished);
                // Let's see if we must erode the right pixel
                if (erodeRightPixel) {
                    // Let's append these pixels to the current run in construction
                    if (runInConstruction.length!=0) {
                        // There is a run in construction, so let's see if it's a hole
                        if (runInConstruction.value!=colorRightPixel) {
                            // Well, we must transfer the current run in construction to the image
                            out->value=runInConstruction.value;
                            out->length=runInConstruction.length;
                            out->line=y;
                            out->parent=outRuns++;
                            out++;
                            if (outRuns+1>=dest->allocated-dest->width) {
                                dest->nbRuns=0;
                                return 0;
                            }

                            // And set this new run in construction
                            runInConstruction.value=colorRightPixel;
                            runInConstruction.length=1;
                        } else {
                            // We can append the eroded right pixel to the run in construction
                            runInConstruction.length++;
                        }
                    } else {
                        // We have no run in construction
                        // Let's build one
                        runInConstruction.value=colorRightPixel;
                        runInConstruction.length=1;
                    }
                    // Let's move to the right
                    x2++;
                }
            }
        } else {
            // This run is a hole
            // Let's append it to the run in construction
            if (runInConstruction.length!=0) {
                // There is a run in construction, so let's see if it's a hole
                if (runInConstruction.value!=0) {
                    // Well, we must transfer the current run in construction to the image
                    out->value=runInConstruction.value;
                    out->length=runInConstruction.length;
                    out->line=y;
                    out->parent=outRuns++;
                    out++;
                    if (outRuns+1>=dest->allocated-dest->width) {
                        dest->nbRuns=0;
                        return 0;
                    }

                    // And set this new run in construction
                    runInConstruction.value=0;
                    runInConstruction.length=currentRun.length;
                } else {
                    // We can append the currentRun to the runInConstruction
                    runInConstruction.length+=currentRun.length;
                }
            } else {
                // We have no run in construction
                // Let's build one
                runInConstruction.value=0;
                runInConstruction.length=currentRun.length;
            }
            // Let's update the previous value
            previousColor=0;
            // Let's move
            x2+=currentRun.length;
        }
        // We have consumed the current run
        // Have we reached the source->width of the picture?
        if (x2==source->width) {
            // Yes, we must transfer the current run in construction to the result,
            out->value=runInConstruction.value;
            out->length=runInConstruction.length;
            out->line=y;
            out->parent=outRuns++;
            out++;
            if (outRuns+1>=dest->allocated-dest->width) {
                dest->nbRuns=0;
                return 0;
            }

            // Go to next line
            if (y==0) {
                y=1;
                l1=source->runs+1;
                x1=0;
            } else {
                y++;
                if (y==source->height) break; // We have finished
                // (l1,x1): Go to the next line
                while (x1<source->width) {
                    x1 += (l1++)->length;
                }
                x1=0;
            }
            // Start a new empty run in construction
            runInConstruction.length=0;
            // We're at the beginning of the next line
            x2=0;
            // Let's set the previous value, since we are beginning a new line
            previousColor=0xffff;
            // And take a new run
            currentRun=*(l2++);
            // And tell that the upper and lower left pixels are in l1
            upperLeftInL1=1;
        } else {
            // Let's take a new current run
            currentRun=*(l2++);
            // Let's compute the new (l1,x1) to follow the current run
            while (x1+l1->length-1<x2) {
                x1 += (l1++)->length;
            }
            upperLeftInL1=(x1!=x2);
        }
    } while (1);
    dest->nbRuns=outRuns;

    // Add one last run, just to ease blob analysis/reconstruction
    out->value=-1;
    out->length=0;
    out->parent=-1;
    out->line=-1;
    if (outRuns+1>=dest->allocated-dest->width) {
	dest->nbRuns=0;
	return 0;
    }
    
    return 1;
}


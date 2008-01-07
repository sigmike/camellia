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

/* Labelling Kernel
 * C code */
#include "camellia.h"

/* v1.1 : Added error checking regarding to CAM_LABEL_MAX_BLOBS
 * v1.2 : 19th of November 2002 
 *	- Bug correction in the processing loop for the top scanline
 *	- Corrected CAM_LABEL_OPTIMIZE1 label optimization
 * v1.3 : 7th of June 2003
 *	- Bug correction in CAM_LABEL_OPTIMIZE1 label optimization
 */

#define CAM_LABEL_OPTIMIZE1

int camLabelling(CamImage *source, CamImage *dest, CamLabellingResults *results)
{
    int x,y,i,n,p;
    int width,height;
    CAM_PIXEL valpix,prevpix,*srcptr,*srctmpptr;
    CAM_LABEL_PIXEL *dstptr,*dsttmpptr;
    int nbBlobs,nbLabels,labval,label1,label2;
    int srcptrwidthstep,dstptrwidthstep;
    int *equivTable=results->equiv;
    int optimize1valid;

    // Check destination is of the right depth
    if ((dest->depth&CAM_DEPTH_MASK)!=8*sizeof(CAM_LABEL_PIXEL)) {
	return 0;
    }
    // Initialize the algorithm
    srcptrwidthstep=source->widthStep;
    dstptrwidthstep=dest->widthStep;

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
	dstptr=(CAM_LABEL_PIXEL*)(dest->imageData+dest->roi->yOffset*dest->widthStep+dest->roi->xOffset*sizeof(CAM_LABEL_PIXEL));
    } else {
	dstptr=(CAM_LABEL_PIXEL*)dest->imageData;
    }

    // Process the first line
    srctmpptr=srcptr;
    dsttmpptr=dstptr;
    // First upper left pixel management (it's the root of the first blob)
    prevpix=*(srcptr++); *(dstptr++)=0; equivTable[0]=0; nbLabels=1; nbBlobs=1;
    // Then process the other pixels on the line
    for (x=1;x<width;x++) { // Bug correction
	valpix=*(srcptr++);
	if (valpix!=prevpix) {
	    // This is a new blob
	    equivTable[nbLabels]=nbLabels;
	    *(dstptr++)=nbLabels++;
	    if (nbLabels>=CAM_LABEL_MAX_BLOBS) return 0;
	    nbBlobs++;
	} else {
	    *dstptr=*CAM_LABEL_PIXEL_ACCESS(dstptr,0,-1);
	    dstptr++;
	}
	prevpix=valpix;
    }
    srcptr=(CAM_PIXEL*)(((char*)srctmpptr)+source->widthStep);
    dstptr=(CAM_LABEL_PIXEL*)(((char*)dsttmpptr)+dest->widthStep);
    
    // And then process the other lines
    for (y=1;y<height;y++) {
        srctmpptr=srcptr;
	dsttmpptr=dstptr;

	// Process the left pixel
	valpix=*srcptr;
	// If the upper pixel value is the same, then copy its label
	if (valpix==*CAM_PIXEL_ACCESS(srcptr,-1,0)) {
	    *dstptr=*CAM_LABEL_PIXEL_ACCESS(dstptr,-1,0);
	    dstptr++;
	    optimize1valid=0; // Label optimization is not valid : we have taken an existing blob
	} else {
	    // This is a new blob
	    equivTable[nbLabels]=nbLabels;
	    *(dstptr++)=nbLabels++;
	    if (nbLabels>=CAM_LABEL_MAX_BLOBS) return 0;
	    nbBlobs++;
	    optimize1valid=1; // Label optimization is valid : this is a new blob
	}
	srcptr++; // Go to the next pixel
	prevpix=valpix;

	// Process the other pixels
	for (x=1;x<width;x++) {
	    valpix=*srcptr;
	    if (valpix==*CAM_PIXEL_ACCESS(srcptr,-1,0)) {
		// If the upper pixel value is the same, then copy its label
		// Check with the left pixel
		if (valpix==prevpix) {
		    // This pixel is connected to the upper AND left pixel
		    // Check the equivalence of labels
		    label1=*CAM_LABEL_PIXEL_ACCESS(dstptr,0,-1);
		    label2=*CAM_LABEL_PIXEL_ACCESS(dstptr,-1,0);
		    if (equivTable[label1]==equivTable[label2]) {
			// If it is the same, then OK
			labval=equivTable[label1];
		    } else {
#ifdef CAM_LABEL_OPTIMIZE1
			// Equivalence table optimization
			// Search backward to eliminate this blob
			if (optimize1valid) {
			    // If this is the latest label
			    // Then remove this label. It was not useful.
			    i=1;
			    labval=equivTable[label2];
			    do {
				*(dstptr-(i++))=labval;
			    } while ((i<=x)&&(*(dstptr-i)==nbLabels-1));
			    nbBlobs--;
			    nbLabels--;
			} else 
#endif
			{
			    // Find terminal roots of each path
			    n=equivTable[label1];
			    while (n!=equivTable[n]) n=equivTable[n];
			    p=equivTable[label2];
			    while (p!=equivTable[p]) p=equivTable[p];
			    
			    // Must use smaller of two to preserve DAGness!
			    if (n<p) {
				equivTable[p]=n;
			    } else {
				equivTable[n]=p;
			    }

			    labval=n;
			    // We have removed one blob (now merged)
			    nbBlobs--;
			}
			optimize1valid=0; // And label optimization is invalidated
		    }
		} else {
		    // Take the label from above
		    labval=*CAM_LABEL_PIXEL_ACCESS(dstptr,-1,0);
		    optimize1valid=0; // And label optimization is invalidated
		}
		// Store the label for this pixel
		*(dstptr++)=labval;
	    } else {
		// The upper value is different
		// Is the left value the same?
		if (valpix==prevpix) {
		    // Yes, then propagate the label of the previous pixel
		    *dstptr=equivTable[*CAM_LABEL_PIXEL_ACCESS(dstptr,0,-1)];
		    dstptr++;
		} else {
		    // No! Then this is a new blob
		    equivTable[nbLabels]=nbLabels;
		    *(dstptr++)=nbLabels++;
		    if (nbLabels>=CAM_LABEL_MAX_BLOBS) return 0;
		    nbBlobs++;
		    optimize1valid=1; // Label optimization is valid : this is a new blob
		}
	    }

	    srcptr++; // Go to the next pixel
	    prevpix=valpix;
	}

        srcptr=(CAM_PIXEL*)(((char*)srctmpptr)+source->widthStep);
	dstptr=(CAM_LABEL_PIXEL*)(((char*)dsttmpptr)+dest->widthStep);
    }

    results->nbLabels=nbLabels;
    return 1;
}

// This algorithm is useless. Presented here for better understanding purpose only.
// Indeed, Blob analysis first scan (camBlobAnalysis1stScan) integrates this second scan.
// In-place processing
int camLabelling2ndScan(CamImage *image, CamLabellingResults *results)
{
    int x,y,p,valpix;
    int width,height;
    CAM_LABEL_PIXEL *imptr,*tmpptr;
    int *equivTable=results->equiv;

    // ROI (Region Of Interest) management
    if (image->roi) {
	imptr=(CAM_LABEL_PIXEL*)(image->imageData+image->roi->yOffset*image->widthStep+image->roi->xOffset*sizeof(CAM_LABEL_PIXEL));
	width=image->roi->width;
	height=image->roi->height;
    } else {
	imptr=(CAM_LABEL_PIXEL*)image->imageData;
	width=image->width;
	height=image->height;
    }

    for (y=0;y<height;y++) {
	tmpptr=imptr;
	for (x=0;x<width;x++,imptr++) {
	    valpix=*imptr;
	    // Parent path compression
	    p=equivTable[valpix];
	    if (p!=valpix) {
		while (p!=equivTable[p]) p=equivTable[p];
		*imptr=p;
	    }    
	}
	imptr=(CAM_LABEL_PIXEL*)(((char*)tmpptr)+image->widthStep);
    }
    return 1;    
}

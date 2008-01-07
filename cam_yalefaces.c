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
#include <math.h>
#include <time.h>
#include "camellia.h"

int main()
{
    CamImage imodel[15], image;
    CamKeypoints points[15], points2;
    CamKeypoints *models[15];
    int i, j, bestMatch, rate = 0, nbGoodMatches = 0, nbFeatures = 0;
    CamKeypointsMatches matches;
    char filename[256];
    char *feature[] = {"centerlight", "glasses", "happy", "leftlight", "noglasses", "rightlight", "sad", "sleepy", "surprised", "wink"};
    const int cx[15] = {181, 195, 164, 193, 189, 118, 162, 161, 155, 171, 183, 208, 191, 133, 168};
    const int cy[15] = {138, 158, 165, 150, 137, 145, 164, 172, 146, 143, 148, 147, 164, 152, 165};
    const int width = 120, height = 140;
    CamAffineTransform t;
    int error, c, x1, y1, x2, y2;
    CamPoint xy[7], uv[7];
    CamImage dimage;
    CamROI roi;
    CamFPKdTreeNode *kdTree;

    const int threshold = 100; 

    for (i = 0; i < 15; i++) {
	sprintf(filename, "resources/yalefaces/subject%02d.normal.pgm", i + 1);
	printf("Feature point detection on %s ...\n", filename);
	imodel[i].imageData = NULL;
	camLoadPGM(&imodel[i], filename);
	image.imageData = NULL; image.depth = CAM_DEPTH_8U;
	camCopy(&imodel[i], &image); // Automatic allocation
	camAllocateKeypoints(&points[i], 10000);
	points[i].id = i;
	points[i].cx = cx[i];
	points[i].cy = cy[i];
	camFastHessianDetector(&imodel[i], &points[i], threshold, CAM_UPRIGHT);
	camDrawKeypoints(&points[i], &image, 128);
	sprintf(filename, "output/yalefaces%02d.pgm", i + 1);
	camSavePGM(&image, filename);
    	camDeallocateImage(&image);
    }	

    camAllocateImage(&dimage, imodel[0].width, imodel[0].height * 2, imodel[0].depth);
    camSetROI(&roi, 0, 0, imodel[0].height, imodel[0].width, imodel[0].height);
    for (i = 0; i < 15; i++) models[i] = &points[i];
    kdTree = camKeypointsCompileKdTree(models, 15);
    camAllocateKeypointsMatches(&matches, 1000);

    for (i = 0; i < 15; i++) {
	for (j = 0; j < 10; j++) {
	    sprintf(filename, "resources/yalefaces/subject%02d.%s.pgm", i + 1, feature[j]);
	    printf("Feature point detection on %s ...\n", filename);
	    image.imageData = NULL;
	    camLoadPGM(&image, filename);
	    camAllocateKeypoints(&points2, 10000);
	    camFastHessianDetector(&image, &points2, threshold, CAM_UPRIGHT);
	    printf("# features = %d\n", points2.nbPoints);
	    nbFeatures += points2.nbPoints;
	    //bestMatch = camKeypointsMatching(&points2, models, 15, &matches);
	    bestMatch = camKeypointsMatchingKdTree(&points2, kdTree, &matches, 100);
	    printf("Best match is : %d (%d)\n", bestMatch + 1, matches.nbMatches);
	    
	    if (bestMatch == i) { 
		rate++;
		nbGoodMatches += matches.nbMatches;

		// Create result image
		dimage.roi = NULL;
		camCopy(&imodel[i], &dimage);
		dimage.roi = &roi;
		camCopy(&image, &dimage);
		dimage.roi = NULL;
		
		// Draw lines between model and target
		for (c = 0; c < matches.nbMatches; c++) {
		    camDrawKeypoint(matches.pairs[c].p1, &dimage, 128);
		    x1 = matches.pairs[c].p1->x;
		    y1 = matches.pairs[c].p1->y;
		    x2 = matches.pairs[c].p2->x;
		    y2 = matches.pairs[c].p2->y;
		    y2 += image.height;
		    camDrawLine(&dimage, x1, y1, x2, y2, 128);
		}
		// Find affine parameters
		camFindAffineTransform(&matches, &t, &error);
		// Draw feature points (lower part)
		for (c = 0; c < matches.nbMatches; c++) {
		    matches.pairs[c].p2->y += image.height;
		    camDrawKeypoint(matches.pairs[c].p2, &dimage, 128);
		}
		// Draw box on model and target
		xy[0].x = xy[3].x = cx[i] - width / 2;
		xy[0].y = xy[1].y = cy[i] - height / 2;
		xy[1].x = xy[2].x = cx[i] + width / 2;
		xy[2].y = xy[3].y = cy[i] + height / 2;
		xy[4].x = xy[5].x = cx[i];
		xy[4].y = xy[6].y = cy[i];
		xy[5].y = cy[i] - height / 4;
		xy[6].x = cx[i] + width / 4;
		for (c = 0; c < 7; c++) {
		    camApplyAffineTransform(&xy[c], &uv[c], &t);
		    uv[c].y += image.height;
		}
		camDrawLine(&dimage, xy[0].x, xy[0].y, xy[1].x, xy[1].y, 0);
		camDrawLine(&dimage, xy[2].x, xy[2].y, xy[1].x, xy[1].y, 0);
		camDrawLine(&dimage, xy[2].x, xy[2].y, xy[3].x, xy[3].y, 0);
		camDrawLine(&dimage, xy[0].x, xy[0].y, xy[3].x, xy[3].y, 0);
		camDrawLine(&dimage, xy[4].x, xy[4].y, xy[5].x, xy[5].y, 0);
		camDrawLine(&dimage, xy[6].x, xy[6].y, xy[4].x, xy[4].y, 0);
		camDrawLine(&dimage, uv[0].x, uv[0].y, uv[1].x, uv[1].y, 0);
		camDrawLine(&dimage, uv[2].x, uv[2].y, uv[1].x, uv[1].y, 0);
		camDrawLine(&dimage, uv[2].x, uv[2].y, uv[3].x, uv[3].y, 0);
		camDrawLine(&dimage, uv[0].x, uv[0].y, uv[3].x, uv[3].y, 0);
		camDrawLine(&dimage, uv[4].x, uv[4].y, uv[5].x, uv[5].y, 0);
		camDrawLine(&dimage, uv[6].x, uv[6].y, uv[4].x, uv[4].y, 0);
		
		sprintf(filename, "output/yalefaces%02d.%s.pgm", i + 1, feature[j]);
		camSavePGM(&dimage, filename);
	    }
	    camDeallocateImage(&image);
	    camFreeKeypoints(&points2);
	}
    }
    
    for (i = 0; i < 15; i++) {
	camDeallocateImage(&imodel[i]);
	camFreeKeypoints(&points[i]);
    }
    camDeallocateImage(&dimage);
    camFreeKeypointsMatches(&matches);
    free(kdTree);

    printf("Recognition rate is %lg%%\n", rate * 100.0 / (15 * 10));
    printf("Good matches = %d\n", nbGoodMatches);
    printf("Nb features = %d (%% matching = %lg%%)\n", nbFeatures, nbGoodMatches * 100.0 / nbFeatures);
}



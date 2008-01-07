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
    CamImage imodel[3], image, color_image;
    CamKeypoints points[3], points2;
    int i, j;
    CamKeypointsMatches matches;
    char filename[256];
    char *model_images[] = {"edog5", "dvd", "mrpotato4"};
#define DISPLAYED_MODEL 2
#define NB_SCENES 5
    char *test_images[NB_SCENES] = {"scene1", "scene3", "scene4", "scene5", "scene7"};

    const int cx[3] = {450, 390, 550};
    const int cy[3] = {340, 510, 450};
    const int width[3] = {720, 690, 500};
    const int height[3] = {640, 910, 500};
    CamAffineTransform t;
    int error, c, x1, y1, x2, y2, score, nbFeatures;
    CamPoint xy[7], uv[7];
    CamImage dimage;
    CamROI roi;

    const int threshold = 50; 

    for (i = 0; i < 3; i++) {
	sprintf(filename, "resources/photos/%s.bmp", model_images[i]);
	printf("Feature point detection on %s ...\n", model_images[i]);
	imodel[i].imageData = NULL;
	camLoadBMP(&imodel[i], filename);
	/*
	camAllocateImage(&image, imodel[i].width, imodel[i].height, CAM_DEPTH_8U);
	camRGB2Y(&imodel[i], &image);
	*/
	camAllocateYUVImage(&image, imodel[i].width, imodel[i].height);
	camRGB2YUV(&imodel[i], &image);
	camAllocateKeypoints(&points[i], 100000);
	points[i].id = i;
	points[i].cx = cx[i];
	points[i].cy = cy[i];
	camFastHessianDetector(&image, &points[i], threshold, CAM_UPRIGHT);
	/*
	camDrawKeypoints(&points[i], &image, 128);
	sprintf(filename, "output/%s.pgm", model_images[i]);
	camSavePGM(&image, filename);
    	*/
	camDeallocateImage(&image);
    }	

    camAllocateRGBImage(&dimage, imodel[0].width, imodel[0].height * 2);
    camSetROI(&roi, 0, 0, imodel[0].height, imodel[0].width, imodel[0].height);
    
    camAllocateKeypointsMatches(&matches, 2048);
    nbFeatures = 0;
    score = 0;

    for (i = 0; i < NB_SCENES; i++) {
	sprintf(filename, "resources/photos/%s.bmp", test_images[i]);
	printf("Feature point detection on %s ...\n", test_images[i]);
	color_image.imageData = NULL;
	camLoadBMP(&color_image, filename);
	/*
	camAllocateImage(&image, color_image.width, color_image.height, CAM_DEPTH_8U);
	camRGB2Y(&color_image, &image);
	*/
	camAllocateYUVImage(&image, color_image.width, color_image.height);
	camRGB2YUV(&color_image, &image);
	camAllocateKeypoints(&points2, 100000);
	camFastHessianDetector(&image, &points2, threshold, CAM_UPRIGHT);
	printf("# features = %d\n", points2.nbPoints);
	nbFeatures += points2.nbPoints;

	// Create result image
	dimage.roi = NULL;
	camCopy(&imodel[DISPLAYED_MODEL], &dimage);
	dimage.roi = &roi;
	camCopy(&color_image, &dimage);
	dimage.roi = NULL;

	for (j = 0; j < 3; j++) {
	    camKeypointsMatching2(&points[j], &points2, &matches);
	    // Find affine parameters
	    if (camFindAffineTransform2(&matches, &t, &error)) {
		score += matches.nbMatches - matches.nbOutliers;
		if (j == DISPLAYED_MODEL) {
		    // Draw lines between model and target
		    for (c = 0; c < matches.nbMatches; c++) {
			if (matches.pairs[c].mark != -1) {
			    camDrawKeypoint(matches.pairs[c].p1, &dimage, CAM_RGB(255, 0, 0));
			    x1 = matches.pairs[c].p1->x;
			    y1 = matches.pairs[c].p1->y;
			    x2 = matches.pairs[c].p2->x;
			    y2 = matches.pairs[c].p2->y;
			    y2 += image.height;
			    camDrawLine(&dimage, x1, y1, x2, y2, CAM_RGB(0, 255, 0));
			}
		    }
		    for (c = 0; c < matches.nbMatches; c++) {
			matches.pairs[c].p2->y += image.height;
			camDrawKeypoint(matches.pairs[c].p2, &dimage, 128);
		    }
		}

		// Draw box on model and target
		xy[0].x = xy[3].x = cx[j] - width[j] / 2;
		xy[0].y = xy[1].y = cy[j] - height[j] / 2;
		xy[1].x = xy[2].x = cx[j] + width[j] / 2;
		xy[2].y = xy[3].y = cy[j] + height[j] / 2;
		xy[4].x = xy[5].x = cx[j];
		xy[4].y = xy[6].y = cy[j];
		xy[5].y = cy[j] - height[j] / 4;
		xy[6].x = cx[j] + width[j] / 4;
		for (c = 0; c < 7; c++) {
		    camApplyAffineTransform(&xy[c], &uv[c], &t);
		    uv[c].y += image.height;
		}
		if (j == DISPLAYED_MODEL) {
		    camDrawLine(&dimage, xy[0].x, xy[0].y, xy[1].x, xy[1].y, 0);
		    camDrawLine(&dimage, xy[2].x, xy[2].y, xy[1].x, xy[1].y, 0);
		    camDrawLine(&dimage, xy[2].x, xy[2].y, xy[3].x, xy[3].y, 0);
		    camDrawLine(&dimage, xy[0].x, xy[0].y, xy[3].x, xy[3].y, 0);
		    camDrawLine(&dimage, xy[4].x, xy[4].y, xy[5].x, xy[5].y, 0);
		    camDrawLine(&dimage, xy[6].x, xy[6].y, xy[4].x, xy[4].y, 0);
		}
		camDrawLine(&dimage, uv[0].x, uv[0].y, uv[1].x, uv[1].y, 0);
		camDrawLine(&dimage, uv[2].x, uv[2].y, uv[1].x, uv[1].y, 0);
		camDrawLine(&dimage, uv[2].x, uv[2].y, uv[3].x, uv[3].y, 0);
		camDrawLine(&dimage, uv[0].x, uv[0].y, uv[3].x, uv[3].y, 0);
		camDrawLine(&dimage, uv[4].x, uv[4].y, uv[5].x, uv[5].y, 0);
		camDrawLine(&dimage, uv[6].x, uv[6].y, uv[4].x, uv[4].y, 0);
	    }
	}
	    
	sprintf(filename, "output/%s.bmp", test_images[i]);
	camSaveBMP(&dimage, filename);
	
	camDeallocateImage(&image);
	camDeallocateImage(&color_image);
	camFreeKeypoints(&points2);
    }
    
    for (i = 0; i < 3; i++) {
	camDeallocateImage(&imodel[i]);
	camFreeKeypoints(&points[i]);
    }
    camDeallocateImage(&dimage);
    camFreeKeypointsMatches(&matches);

    printf("# features in scenes = %d\n", nbFeatures);
    printf("# matching features = %d (%lg%%)\n", score, score*100.0/nbFeatures);

}



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
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832795 
#endif
#include <time.h>
#include "camellia.h"

// Benchmarking functions
#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
void camInitBenchmark() {timeBeginPeriod(1);}
int camGetTimeMs() {return timeGetTime();}
#else
#include <sys/time.h>
#include <unistd.h>
void camInitBenchmark() {}
int camGetTimeMs()
{
    int t;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    t=tv.tv_sec*1000+tv.tv_usec/1000;
    return t;
}
#endif

const int SOBEL_3x3_V[3][3]={
    {-1,0,1},
    {-2,0,2},
    {-1,0,1}
}; // Sobel Vertical filter

// Linear and median filtering examples
void example_filters()
{
    CamImage source,dest;
    CamLinearFilterKernel kernel;
    int i,j;

    // Load picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    
    // Sobel filter on the image (signed)
    camAllocateImage(&dest,source.width,source.height,CAM_DEPTH_8S);
    for (i=0;i<3;i++) {
	for (j=0;j<3;j++) {
	    kernel.kernel[i][j]=SOBEL_3x3_V[i][j];
	}
    }
    kernel.coeff1=1;
    kernel.coeff2=0;
    camSetBorder(&source,128);
    camLinearFilter3x3(&source,&dest,&kernel);
    camSavePGM(&dest,"output/chess_sobel_signed.pgm");

    camSobelV(&source,&dest);
    camSavePGM(&dest,"output/chess_sobel2_signed.pgm");

    // Sobel filter on the image (unsigned)
    dest.depth=CAM_DEPTH_8U;
    camLinearFilter3x3(&source,&dest,&kernel);
    camSavePGM(&dest,"output/chess_sobel_unsigned.pgm");

    camSobelV(&source,&dest);
    camSavePGM(&dest,"output/chess_sobel2_unsigned.pgm");

    // Sobel filter on the image (unsigned)
    dest.depth=CAM_DEPTH_8U;
    camLinearFilterAbs3x3(&source,&dest,&kernel);
    camSavePGM(&dest,"output/chess_sobel_abs.pgm");

    camSobelVAbs(&source,&dest);
    camSavePGM(&dest,"output/chess_sobel2_abs.pgm");

    // Median filtering
    camMedianFilter5x5(&source,&dest);
    camSavePGM(&dest,"output/chess_median.pgm");
    
    camDeallocateImage(&source);
    camDeallocateImage(&dest);
}

// RLE labelling example
void example_labeling()
{
    CamImage source;
    CamRLEImage encoded;
    CamBlobs results;
    int i;

    // Load picture small_ulp.pgm (example provided by ULP, which made labelling v1.0 fail)
    camLoadPGM(&source,"resources/small_ulp.pgm");
    
    // Label the image
    camRLEAllocate(&encoded,10000);
    camRLEEncode(&source,&encoded);
    printf("Number of runs : %d\n",encoded.nbRuns);
    camRLELabeling(&encoded,&results);

    // Print the results 
    for (i=0;i<results.nbBlobs;i++) {
	printf("Blob #%2d : (%3d,%3d,%3d,%3d) Surface=%d\n",
	    i,results.blobInfo[i].left,results.blobInfo[i].top,
	    results.blobInfo[i].width,results.blobInfo[i].height,
	    results.blobInfo[i].surface);
    }

    camRLEDeallocate(&encoded);
    camDeallocateImage(&source);
}

// Warping example
void example_warping()
{
    CamImage source,resampled,warped,rotated;
    CamWarpingParams params;
    int t1,t2;
    int c;

    // Load picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    camAllocateImage(&resampled,source.width/2,source.height/2,source.depth);
    camAllocateImage(&warped,source.width,source.height,source.depth);
    camAllocateImage(&rotated,source.width,source.width,source.depth);

    t1=camGetTimeMs();
    for (c=0;c<1000;c++)
    {
	params.interpolation=0;
	
	params.perspective=0;
	params.p[0].x=0;
	params.p[0].y=0;
	params.p[1].x=(source.width<<16)-1;
	params.p[1].y=0;
	params.p[2].x=(source.width<<16)-1;
	params.p[2].y=(source.height<<16)-1;
	params.p[3].x=0;
	params.p[3].y=(source.height<<16)-1;
	
	camWarping(&source,&resampled,&params);
	
	params.perspective=1;
	params.p[0].x=((source.width<<16)-1)*1/3;
	params.p[0].y=0;
	params.p[1].x=((source.width<<16)-1)*2/3;
	params.p[1].y=0;
	params.p[2].x=(source.width<<16)-1;
	params.p[2].y=(source.height<<16)-1;
	params.p[3].x=0;
	params.p[3].y=(source.height<<16)-1;
	
	camWarping(&source,&warped,&params);
	
	params.perspective=0;
	params.p[0].x=(source.width<<16)/2;
	params.p[0].y=-(source.height<<16)/2;
	params.p[1].x=3*(source.width<<16)/2;
	params.p[1].y=(source.height<<16)/2;
	params.p[2].x=(source.width<<16)/2;
	params.p[2].y=3*(source.height<<16)/2;
	params.p[3].x=-(source.width<<16)/2;
	params.p[3].y=(source.height<<16)/2;
	
	camWarping(&source,&rotated,&params);
    }	
    t2=camGetTimeMs();
    printf("Nearest neighbour warping = %d us\n",t2-t1);

    camSavePGM(&resampled,"output/chess_resampled_nn.pgm");
    camSavePGM(&warped,"output/chess_warped_nn.pgm");
    camSavePGM(&rotated,"output/chess_rotated_nn.pgm");

    t1=camGetTimeMs();
    for (c=0;c<1000;c++)
    {
	params.interpolation=1;
	
	params.perspective=0;
	params.p[0].x=0;
	params.p[0].y=0;
	params.p[1].x=(source.width<<16)-1;
	params.p[1].y=0;
	params.p[2].x=(source.width<<16)-1;
	params.p[2].y=(source.height<<16)-1;
	params.p[3].x=0;
	params.p[3].y=(source.height<<16)-1;
	
	camWarping(&source,&resampled,&params);
	
	params.perspective=1;
	params.p[0].x=((source.width<<16)-1)*1/3;
	params.p[0].y=0;
	params.p[1].x=((source.width<<16)-1)*2/3;
	params.p[1].y=0;
	params.p[2].x=((source.width<<16)-1);
	params.p[2].y=(source.height<<16)-1;
	params.p[3].x=0;
	params.p[3].y=(source.height<<16)-1;
	
	camWarping(&source,&warped,&params);
	
	params.perspective=0;
	params.p[0].x=(source.width<<16)/2;
	params.p[0].y=-(source.height<<16)/2;
	params.p[1].x=3*(source.width<<16)/2;
	params.p[1].y=(source.height<<16)/2;
	params.p[2].x=(source.width<<16)/2;
	params.p[2].y=3*(source.height<<16)/2;
	params.p[3].x=-(source.width<<16)/2;
	params.p[3].y=(source.height<<16)/2;
	
	camWarping(&source,&rotated,&params);
    }
    t2=camGetTimeMs();
    printf("Bilinear interpolation warping = %d us\n",(t2-t1));

    camSavePGM(&resampled,"output/chess_resampled_bilinear.pgm");
    camSavePGM(&warped,"output/chess_warped_bilinear.pgm");
    camSavePGM(&rotated,"output/chess_rotated_bilinear.pgm");

    camDeallocateImage(&source);
    camDeallocateImage(&resampled);
    camDeallocateImage(&warped);
    camDeallocateImage(&rotated);
}

int CircleStructElt[5][5]=
   {{0,1,1,1,0},
    {1,1,1,1,1},
    {1,1,1,1,1},
    {1,1,1,1,1},
    {0,1,1,1,0}};

// Benchmark for labeling and RLE labeling
void example_labeling_benchmark()
{
    CamImage source,dilated,thresholded,labelled;
    CamRLEImage encoded;
    CamMorphoMathsKernel mm_params;
    CamArithmParams arithm_params;
    CamLabellingResults results;
    CamBlobs analysis;
    CamTable LUT;
    int x,y,i;
    int t1,t2,t3;
    int c;

    // Load picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    
    // Dilate that picture
    camAllocateImage(&dilated,source.width,source.height,source.depth);
    for (x=0;x<5;x++) {
	for (y=0;y<5;y++) {
	    mm_params.dilationStructElt[x][y]=CircleStructElt[x][y];
	    mm_params.erosionStructElt[x][y]=0;
	}
    }
    camDilate5x5(&source,&dilated,&mm_params);
    camSavePGM(&dilated,"output/chess_dilated3.pgm");

    camRLEAllocate(&encoded,(source.width*source.height)/4);
    for (i=0;i<128;i++) LUT.t[i]=0;
    for (;i<256;i++) LUT.t[i]=255;

    // Conventional thresholding and labelling

    // Threshold the dilated picture
    camAllocateImage(&thresholded,source.width,source.height,source.depth);
    arithm_params.operation=CAM_ARITHM_THRESHOLD;
    arithm_params.c1=128;
    arithm_params.c2=0;
    arithm_params.c3=1;
    camAllocateImage(&labelled,source.width,source.height,16);
    
    t1=camGetTimeMs();
    for (c=0;c<1000;c++)
    {
	camMonadicArithm(&dilated,&thresholded,&arithm_params);	
	// Label the tresholded image
	camLabelling(&thresholded,&labelled,&results);
	// Analyze the blobs
	camBlobAnalysis1stScan(&labelled,NULL,&results,&analysis);
    }

    printf("Conventional thresholding & labelling\n");
    for (i=0;i<analysis.nbBlobs;i++) {
	printf("Blob #%2d : (%3d,%3d,%3d,%3d) Surface=%d\n",
	    i+1,analysis.blobInfo[i].left,analysis.blobInfo[i].top,
	    analysis.blobInfo[i].width,analysis.blobInfo[i].height,
	    analysis.blobInfo[i].surface);
    }

    // RLE thresholding and labelling
    t2=camGetTimeMs();
    for (c=0;c<1000;c++)
    {
	// RLE encoding
	camRLEEncodeLUT(&dilated,&encoded,&LUT);
	// Labelling
	camRLELabelling(&encoded,&analysis);
    }
    printf("RLE thresholding & labelling\n");
    for (i=0;i<analysis.nbBlobs;i++) {
	printf("Blob #%2d : (%3d,%3d,%3d,%3d) Surface=%d\n",
	    i+1,analysis.blobInfo[i].left,analysis.blobInfo[i].top,
	    analysis.blobInfo[i].width,analysis.blobInfo[i].height,
	    analysis.blobInfo[i].surface);
    }
    t3=camGetTimeMs();

    // Analysis of holes
    camRLEInverse(&encoded);
    camRLELabelling(&encoded,&analysis);
    printf("RLE thresholding & labelling : Holes\n");
    for (i=0;i<analysis.nbBlobs;i++) {
	printf("Blob #%2d : (%3d,%3d,%3d,%3d) Surface=%d\n",
	    i+1,analysis.blobInfo[i].left,analysis.blobInfo[i].top,
	    analysis.blobInfo[i].width,analysis.blobInfo[i].height,
	    analysis.blobInfo[i].surface);
    }

    printf("Conventional thresholding & labelling = %d us\n",(t2-t1));
    printf("RLE thresholding & labelling          = %d us\n",(t3-t2));

    camDeallocateImage(&source);
    camDeallocateImage(&dilated);
    camDeallocateImage(&thresholded);
    camDeallocateImage(&labelled);
    camRLEDeallocate(&encoded);
}

// Binary image processing example
void example_binary()
{
    CamImage source,binary,dilated,inverse,or;
    CamArithmParams arithm_params;
    CamRLEImage encoded;
    CamBlobs results;
    CamMorphoMathsKernel mm_params;
    int i,x,y;

    camLoadPGM(&source,"resources/chess.pgm");
    camAllocateImage(&binary,source.width,source.height,CAM_DEPTH_1U);
    camSet(&binary,0); // Just to make valgrind happy. Not necessary
    arithm_params.operation=CAM_ARITHM_THRESHOLD;
    arithm_params.c1=128;
    arithm_params.c2=0;
    arithm_params.c3=255;
    camMonadicArithm(&source,&binary,&arithm_params);
    camSavePGM(&binary,"output/chess_binary.pgm");

    // Dilate that picture
    camAllocateImage(&dilated,source.width,source.height,CAM_DEPTH_1U);
    camSet(&dilated,0); // Just to make valgrind happy. Not necessary
    for (x=0;x<5;x++) {
	for (y=0;y<5;y++) {
	    mm_params.dilationStructElt[x][y]=CircleStructElt[x][y];
	    mm_params.erosionStructElt[x][y]=0;
	}
    }
    camDilate5x5(&binary,&dilated,&mm_params);
    camSavePGM(&dilated,"output/chess_binary_dilated.pgm");

    // Inverse the picture
    camAllocateImage(&inverse,source.width,source.height,CAM_DEPTH_1U);
    camSet(&inverse,0); // Just to make valgrind happy. Not necessary
    arithm_params.operation=CAM_ARITHM_INVERSE;
    camMonadicArithm(&dilated,&inverse,&arithm_params);
    camSavePGM(&inverse,"output/chess_binary_inverse.pgm");
    
    // Or with the binary image
    camAllocateImage(&or,source.width,source.height,CAM_DEPTH_1U);
    camSet(&or,0); // Just to make valgrind happy. Not necessary
    arithm_params.operation=CAM_ARITHM_OR;
    camDyadicArithm(&binary,&inverse,&or,&arithm_params);
    camSavePGM(&or,"output/chess_binary_or.pgm");

    // Label the image
    camRLEAllocate(&encoded,100000);
    camRLEEncode(&dilated,&encoded);
    printf("Number of runs : %d\n",encoded.nbRuns);
    camRLELabelling(&encoded,&results);

    // Print the results 
    for (i=0;i<results.nbBlobs;i++) {
	printf("Blob #%2d : (%3d,%3d,%3d,%3d) Surface=%d\n",
	    i,results.blobInfo[i].left,results.blobInfo[i].top,
	    results.blobInfo[i].width,results.blobInfo[i].height,
	    results.blobInfo[i].surface);
    }

    camRLEDeallocate(&encoded);
    camDeallocateImage(&source);
    camDeallocateImage(&binary);
    camDeallocateImage(&dilated);
    camDeallocateImage(&inverse);
    camDeallocateImage(&or);
}

static const int camCircle7StructElt[7][7]={
    {0,0,1,1,1,0,0},
    {0,1,1,1,1,1,0},
    {1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1},
    {1,1,1,1,1,1,1},
    {0,1,1,1,1,1,0},
    {0,0,1,1,1,0,0}
};

// Morphomaths example
void example_morpho()
{
    CamImage source,dest;
    CamMorphoMathsKernel mm_params;
    int x,y;
    int t1,t2;

    // Load picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    camAllocateImage(&dest,source.width,source.height,source.depth);
    
    for (x=0;x<5;x++) {
	for (y=0;y<5;y++) {
	    mm_params.dilationStructElt[x][y]=CircleStructElt[x][y];
	    mm_params.erosionStructElt[x][y]=CircleStructElt[x][y];;
	}
    }
    mm_params.operation=CAM_MM_SUBSTRACTION;
    mm_params.source1=CAM_MM_DILATED;
    mm_params.source2=CAM_MM_ERODED;
    t1=camGetTimeMs();
    camMorphoMaths(&source,&dest,&mm_params);
    t2=camGetTimeMs();
    printf("5x5 Circle outplace morphological gradient = %d ms\n",(t2-t1));

    camSavePGM(&dest,"output/chess_gradient.pgm");

    for (x=0;x<5;x++) {
	for (y=0;y<5;y++) {
	    mm_params.dilationStructElt[x][y]=CircleStructElt[x][y];
	    mm_params.erosionStructElt[x][y]=CircleStructElt[x][y];;
	}
    }
    mm_params.operation=CAM_MM_SUBSTRACTION;
    mm_params.source1=CAM_MM_DILATED;
    mm_params.source2=CAM_MM_ERODED;
    t1=camGetTimeMs();
    camMorphoMaths(&source,&source,&mm_params);
    t2=camGetTimeMs();
    printf("5x5 Circle inplace morphological gradient  = %d ms\n",(t2-t1));

    camDeallocateImage(&source);

    // Reload picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    
    t1=camGetTimeMs();
    camDilate5x5(&source,&dest,&mm_params);
    t2=camGetTimeMs();
    printf("5x5 Circle outplace dilation               = %d ms\n",(t2-t1));
    
    t1=camGetTimeMs();
    camDilate5x5(&source,&source,&mm_params);
    t2=camGetTimeMs();
    printf("5x5 Circle inplace dilation                = %d ms\n",(t2-t1));
    
    camDeallocateImage(&source);

    // Reload picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    
    t1=camGetTimeMs();
    camDilateCircle5(&source,&dest);
    t2=camGetTimeMs();
    printf("5x5 Circle outplace dilation               = %d ms\n",(t2-t1));
    
    t1=camGetTimeMs();
    camDilateCircle5(&source,&source);
    t2=camGetTimeMs();
    printf("5x5 Circle inplace dilation                = %d ms\n",(t2-t1));

    camSavePGM(&source,"output/chess_dilate_circle5.pgm");
    
    camDeallocateImage(&source);

    // Reload picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    
    for (y=0;y<7;y++) {
        for (x=0;x<7;x++) {
            mm_params.dilationStructElt[y][x]=camCircle7StructElt[y][x];
        }
    }
    t1=camGetTimeMs();
    camDilate7x7(&source,&dest,&mm_params);
    t2=camGetTimeMs();
    printf("7x7 Circle unoptimized dilation            = %d ms\n",(t2-t1));

    t1=camGetTimeMs();
    camDilateCircle7(&source,&dest);
    t2=camGetTimeMs();
    printf("7x7 Circle outplace dilation               = %d ms\n",(t2-t1));
    
    t1=camGetTimeMs();
    camDilateCircle7(&source,&source);
    t2=camGetTimeMs();
    printf("7x7 Circle inplace dilation                = %d ms\n",(t2-t1));
    
    camSavePGM(&dest,"output/chess_dilate_circle7.pgm");

    camDeallocateImage(&source);
    camDeallocateImage(&dest);
}

// Watershed segmentation example
void example_watershed()
{
    CamImage source,watershed;
    CamTableOfBasins tob;
    int i,j,c;
    char filename[4][30]={"lsun","lsun_32x64e20-bord","lsun_48x80e20","lsun_56x88e20"};
    char s[256];

    for (c=0;c<4;c++) {
        sprintf(s,"resources/%s.pgm",filename[c]);
        camLoadPGM(&source,s);
        camAllocateImage(&watershed,source.width,source.height,CAM_DEPTH_16S);
        camHierarchicalWatershed(&source,&watershed,&tob);

        for (i=0;i<tob.nbBasins;i++) {
            if (tob.tab[i].dynamics<50){
                tob.tab[i].surface=0;
            }
        }
        for (i=0;i<tob.nbBasins;i++) {
            if ((tob.tab[i].surface!=0)||(tob.tab[i].dynamics==CAM_NOT_COMPUTED)) {
                printf("\nBasin #%d : Dynamics = %d, Minimum = %d, Surface = %d, (x,y)=(%d,%d)\n",i,tob.tab[i].dynamics,tob.tab[i].minimum,tob.tab[i].accsurface,tob.tab[i].x,tob.tab[i].y);
                j=i;
                while (tob.tab[j].dynamics!=CAM_NOT_COMPUTED) {
                    printf("->%d",tob.tab[j].flooded);
                    j=tob.tab[j].flooded;
                }
            }
        }

        camHierarchicalWatershedRegions(&watershed,&tob);
        sprintf(s,"output/watershed_regions_%s.pgm",filename[c]);
        camSavePGM(&watershed,s);
        
        camFreeTableOfBasins(&tob);
        camDeallocateImage(&source);
        camDeallocateImage(&watershed);
    }
}

// Watershed segmentation example
void example_watershed2()
{
    CamImage source,watershed,gradient;
    CamTableOfBasins tob;
    CamRLEImage encoded;
    CamBlobs results;
    int i,j;

    camAllocateImage(&source,32,32,CAM_DEPTH_8U);
    camSet(&source,0);
    camDrawCircle(&source,16,16,8,128);
    camFillColor(&source,16,16,255,0);
        
/*
    CamImage color,ds; 
    camLoadBMP(&color,"obstacles3.bmp");
    camAllocateRGBImage(&ds,color.width/2,color.height/2);
    camDownscaling2x2(&color,&ds);
    camDeallocateImage(&color);
    camAllocateImage(&source,ds.width,ds.height,CAM_DEPTH_8U);
    camRGB2Y(&ds,&source);
    camDeallocateImage(&ds);
    camDilateCircle5(&source,&source);
 */
    
    // Compute the watershed of the gradient of the picture
    camAllocateImage(&gradient,source.width,source.height,source.depth);
    camMorphoGradientCircle5(&source,&gradient);
    camSavePGM(&gradient,"output/watershed_source_gradient.pgm");
    camAllocateImage(&watershed,source.width,source.height,CAM_DEPTH_16S);
    camHierarchicalWatershed(&gradient,&watershed,&tob);
    camSavePGM(&watershed,"output/watershed_gradient.pgm");
    for (i=0;i<tob.nbBasins;i++) {
	printf("Basin #%d : Dynamics = %d, Minimum = %d, Surface = %d, (x,y)=(%d,%d)\n",i,tob.tab[i].dynamics,tob.tab[i].minimum,tob.tab[i].accsurface,tob.tab[i].x,tob.tab[i].y);
    }
    printf("\n");

    for (i=0;i<tob.nbBasins;i++) {
        if (tob.tab[i].dynamics<50){
            tob.tab[i].surface=0;
        }
        if (tob.tab[i].accsurface<watershed.height*watershed.width/100) {
            tob.tab[i].surface=0;
        }
    }
    for (i=0;i<tob.nbBasins;i++) {
        if ((tob.tab[i].surface!=0)||(tob.tab[i].dynamics==CAM_NOT_COMPUTED)) {
            printf("\nBasin #%d : Dynamics = %d, Minimum = %d, Surface = %d, (x,y)=(%d,%d)\n",i,tob.tab[i].dynamics,tob.tab[i].minimum,tob.tab[i].accsurface,tob.tab[i].x,tob.tab[i].y);
            j=i;
	    while (tob.tab[j].dynamics!=CAM_NOT_COMPUTED) {
		printf("->%d",tob.tab[j].flooded);
		j=tob.tab[j].flooded;
	    }
	}
    }

    camHierarchicalWatershedRegions(&watershed,&tob);
    camSavePGM(&watershed,"output/watershed_regions.pgm");

    // Labelling
    camRLEAllocate(&encoded, 10000);
    camRLEEncode(&watershed,&encoded);
    camRLELabelling(&encoded,&results);
    
    // Print the results 
    printf("\n");
    for (i=0;i<results.nbBlobs;i++) {
        camRLEBlobMeasures(&results.blobInfo[i], &source);
	printf("Blob #%2d : (%3d,%3d,%3d,%3d) Surface=%d\n Average=%d Min=%d Max=%d\n",
	    i,results.blobInfo[i].left,results.blobInfo[i].top,
	    results.blobInfo[i].width,results.blobInfo[i].height,
	    results.blobInfo[i].surface,results.blobInfo[i].value,results.blobInfo[i].min,results.blobInfo[i].max);
    }

    camRLEDeallocate(&encoded);
    
    camFreeTableOfBasins(&tob);
    camDeallocateImage(&source);
    camDeallocateImage(&watershed);
    camDeallocateImage(&gradient);
}

// Drawing functions example
void example_draw()
{
    CamImage image;
    CamROI roi;
    CamBitmapFont font;
    camAllocateRGBImage(&image,320,240);
    camSet(&image,0);
    camDrawLine(&image,0,0,320,240,CAM_RGB(255,0,0));
    camFillColor(&image,50,100,CAM_RGB(255,0,0),-1);
    camDrawRectangle(&image,100,100,320-100,240-100,CAM_RGB(0,255,0));
    camDrawEllipse(&image,160,120,70,50,CAM_RGB(0,0,255));
    camDrawText16s(&image,"HELLO WORLD",100,120,10,20,0,CAM_RGB(255,255,255));

    camLoadBitmapFont(&font,"resources/fonts/epicpin.bmp");
    camDrawTextBitmap(&image,"Hello World",100,140,&font);
    camFreeBitmapFont(&font);
    
    camLoadBitmapFont(&font,"resources/fonts/ddrsmall.bmp");
    camDrawTextBitmap(&image,"Hello World",100,170,&font);
    camFreeBitmapFont(&font);
      
    camLoadBitmapFont(&font,"resources/fonts/mario3.bmp");
    camDrawTextBitmap(&image,"Hello World",100,180,&font);
    camFreeBitmapFont(&font);
   
    camLoadBitmapFont(&font,"resources/fonts/system-color-font.bmp");
    camDrawTextBitmap(&image,"Hello World",100,190,&font);
    camFreeBitmapFont(&font);
     
    camLoadBitmapFont(&font,"resources/fonts/xenon2.bmp");
    camDrawTextBitmap(&image,"Camellia",-8,-5,&font);
    camFreeBitmapFont(&font);
    
    camSetROI(&roi,0,100,50,120,120);
    image.roi=&roi;
    camDrawLine(&image,320,0,0,240,CAM_RGB(255,255,0));
    camSaveBMP(&image,"output/drawing.bmp");
    camDeallocateImage(&image);
}

// RLE erosion example
void example_rle_erosion()
{
    CamImage source,binary;
    CamRLEImage encoded,eroded;
    int i;
    CamTable LUT;

    camLoadPGM(&source,"resources/chess.pgm");
    camAllocateImage(&binary,source.width,source.height,CAM_DEPTH_8U);
    camRLEAllocate(&encoded,100000);
    camRLEAllocate(&eroded,100000);

    for (i=0;i<128;i++) LUT.t[i]=0;
    for (;i<256;i++) LUT.t[i]=255;
    
    camRLEEncodeLUT(&source,&encoded,&LUT);
    printf("Number of runs : %d\n",encoded.nbRuns);

    camRLEErodeCross(&encoded,&eroded);
    printf("Number of runs : %d\n",eroded.nbRuns);
    camRLEDecode(&eroded,&binary,&LUT);
    camSavePGM(&binary,"output/chess_RLE_eroded_cross.pgm");

    camRLEErode3x3(&encoded,&eroded);
    printf("Number of runs : %d\n",eroded.nbRuns);
    camRLEDecode(&eroded,&binary,&LUT);
    camSavePGM(&binary,"output/chess_RLE_eroded_3x3.pgm");

    camRLEErode3x2(&encoded,&eroded);
    printf("Number of runs : %d\n",eroded.nbRuns);
    camRLEDecode(&eroded,&binary,&LUT);
    camSavePGM(&binary,"output/chess_RLE_eroded_3x2.pgm");

    camRLEDeallocate(&encoded);
    camRLEDeallocate(&eroded);
    camDeallocateImage(&source);
    camDeallocateImage(&binary);
}

// Motion estimation example
void example_me()
{
    CamImage image1,image2;
    CamROI roi1,roi2;
    CamMotionEstimation3DRSParams params;
    CamMotionEstimation3DRSResults results;
    int x,y;

    camLoadPGM(&image1,"resources/chess.pgm");
    camAllocateImage(&image2,image1.width,image1.height,CAM_DEPTH_8U);
    camSet(&image2,0);

    roi1.width=roi2.width=image1.width-10;
    roi1.height=roi2.height=image1.height-8;
    roi1.xOffset=0;
    roi1.yOffset=0;
    roi2.xOffset=10;
    roi2.yOffset=8;

    image1.roi=&roi1;
    image2.roi=&roi2;
    camCopy(&image1,&image2);
    camSavePGM(&image2,"output/chess_translated.pgm");
    camMotionEstimation3DRSInit(&params,0,2,20,16,3,5,0);
    camMotionEstimation3DRS(&image2,&image1,&params,&results);

    for (y=0;y<image2.height/16;y++) {
        for (x=0;x<image2.width/16;x++) {
            printf("(%2d,%2d)",results.vx[x][y],results.vy[x][y]);
        }
        printf("\n");
    }

    camDeallocateImage(&image1);
    camDeallocateImage(&image2);
}

void example_color()
{
    CamImage image1,image2;
    camLoadBMP(&image1,"resources/alfa156.bmp");
    camAllocateYUVImage(&image2,image1.width,image1.height);
    camRGB2YUV(&image1,&image2);
    camYUV2RGB(&image2,&image1);
    camSaveBMP(&image1,"output/alfa156_2.bmp");
    camDeallocateImage(&image1);
    camDeallocateImage(&image2);
}

void example_undistort()
{
    const float intrinsic_matrix[]={729.330f,0,323.557f,0,730.695f,237.079f,0,0,1};
    const float dist_coeffs[]={-0.212201f,0.263761f,0.000363f,0.000936f};
    CAM_FIXED_POINT intrinsic_matrix_fixed[9],dist_coeffs_fixed[4];
    int i;

    CamImage source,dest;
    camLoadBMP(&source,"resources/DSCN2773.bmp");
    camAllocateRGBImage(&dest,source.width,source.height);
    
    // Floating point undistort
    camUndistort(&source,&dest,intrinsic_matrix,dist_coeffs);
    camSaveBMP(&dest,"output/DSCN2773_undistorted.bmp");

    // Fixed point undistort
    for (i=0;i<9;i++) intrinsic_matrix_fixed[i]=CAM_FLOAT2FIXED(intrinsic_matrix[i],20);
    for (i=0;i<4;i++) dist_coeffs_fixed[i]=CAM_FLOAT2FIXED(dist_coeffs[i],20);
    camUndistortFixed(&source,&dest,intrinsic_matrix_fixed,dist_coeffs_fixed); 
    camSaveBMP(&dest,"output/DSCN2773_undistorted_fixed.bmp");
    
    camDeallocateImage(&source);
    camDeallocateImage(&dest);
}

int array[1000000];
void testOptP4()
{
    int c,i,j,t1,t2;
    //int res[2]={1,0};
    for (i=0;i<1000000;i++) array[i]=rand()%100;
    for (c=0;c<2;c++) {
	t1=camGetTimeMs();
	for (i=0,j=0;i<1000000;i++) {
	    j+=(array[i]>50)?1:0;
	    //j+=res[array[i]>50];
	}
	t2=camGetTimeMs();
	printf("Array counting #%d = %d ms\n",c,(t2-t1));
    }
}

// RLE color labelling example
void example_color_labeling()
{
    CamImage source,YUV;
    CamRLEImage encoded;
    CamBlobs results;
    CamTable clusters;
    int i;
    const int limits[3*6]={
    //  Ymin Ymax Umin Umax Vmin Vmax
        0,   60,  0,   255, 0,   255, // Black
        230, 255, 0,   255, 0,   255, // White
        0,   255, 0,   255, 140, 255  // Red
    };
    int color[3]={CAM_RGB(0,0,0),CAM_RGB(255,255,255),CAM_RGB(255,0,0)};

    printf("\nColor Image Labeling example : \n");    

    camLoadBMP(&source,"resources/alfa156.bmp");
    camAllocateYUVImage(&YUV,source.width,source.height);
    camRGB2YUV(&source,&YUV);

    // Label the image
    camRLEAllocate(&encoded,10000);
    clusters.size=3*6;
    for (i=0;i<3*6;i++) clusters.t[i]=limits[i];
    camRLEEncodeColor(&YUV,&encoded,&clusters);
    printf("Number of runs : %d\n",encoded.nbRuns);
    camRLELabeling(&encoded,&results);

    // Print the results 
    for (i=0;i<results.nbBlobs;i++) {
	printf("Blob #%2d : Val=%d (%3d,%3d,%3d,%3d) Surface=%d\n",
	    i,results.blobInfo[i].value,
            results.blobInfo[i].left, results.blobInfo[i].top,
	    results.blobInfo[i].width, results.blobInfo[i].height,
	    results.blobInfo[i].surface);
        camDrawRectangle(&source, results.blobInfo[i].left, results.blobInfo[i].top,
            results.blobInfo[i].left+results.blobInfo[i].width-1,
            results.blobInfo[i].top+results.blobInfo[i].height-1,
            color[results.blobInfo[i].value-1]);
    }
    camSaveBMP(&source,"output/alfa156_color_labeling.bmp");

    camRLEDeallocate(&encoded);
    camDeallocateImage(&source);
    camDeallocateImage(&YUV);
}

void example_scale()
{
    CamImage source,dest;

    printf("\nScaling example : \n");    

    camLoadBMP(&source,"resources/alfa156.bmp");
    camAllocateRGBImage(&dest,source.height,source.width);
    camScale(&source,&dest);
    camSaveBMP(&dest,"output/alfa156_scaled.bmp");

    camDeallocateImage(&source);
    camDeallocateImage(&dest);
}

void example_mask()
{
    CamImage source,dest,mask;
    CamRLEImage encoded_mask;

    // Load picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");    
    camAllocateImage(&dest,source.width,source.height,CAM_DEPTH_8U);
    camAllocateImage(&mask,source.width,source.height,CAM_DEPTH_8U);
    camRLEAllocate(&encoded_mask,10000);

    // Draw a filled circle in mask
    camSet(&mask,0);
    camDrawCircle(&mask,source.width/2,source.height/2,50,255);
    camFillColor(&mask,source.width/2,source.height/2,255,-1);
    camSavePGM(&mask,"output/mask.pgm");

    camRLEEncode(&mask,&encoded_mask);
    camSetRLEMask(&source,&encoded_mask);
    
    camRLEInverse(&encoded_mask);
    // Copy the original picture
    camCopy(&source,&dest);
    camRLEInverse(&encoded_mask);

    camSobelVAbs(&source,&dest);
    camSavePGM(&dest,"output/chess_sobel_mask.pgm");

    camErodeCircle7(&source,&dest);
    camSavePGM(&dest,"output/chess_erode_mask.pgm");

    camDeallocateImage(&source);
    camDeallocateImage(&dest);
    camDeallocateImage(&mask);
    camRLEDeallocate(&encoded_mask);
}

void example_sobel()
{
    int percent=20;
    CamImage image,yuv,thr,sobel_h, sobel_v, sum;
    CamRLEImage thresholded;
    CamROI roi;
    CamArithmParams p;
    CamTable histo;
    int threshold;
    CamTable lut;
    lut.t[0]=0; lut.t[1]=255;

    camLoadBMP(&image,"resources/road1.bmp");
    camAllocateYUVImage(&yuv,image.width,image.height);
    camRGB2YUV(&image,&yuv);
    roi.coi=1; roi.xOffset=8; roi.yOffset=8; roi.width=image.width-16; roi.height=image.height-128;
    yuv.roi=&roi;
    camErodeCircle7(&yuv,&yuv);
    
    // Allocate 16 bits images for the gradient images
    camAllocateImage(&sobel_h,roi.width,roi.height,CAM_DEPTH_12U);
    camAllocateImage(&sobel_v,roi.width,roi.height,CAM_DEPTH_12U);
    camAllocateImage(&sum,roi.width,roi.height,CAM_DEPTH_12U);

    // Compute the 16-bits gradient images
    camSobelHAbs(&yuv,&sobel_h);
    camSobelVAbs(&yuv,&sobel_v);

    // Compute the sum image
    p.operation=CAM_ARITHM_ADD;
    camDyadicArithm(&sobel_h,&sobel_v,&sum,&p);

    // Threshold the result image to keep "percent" pixels
    camHistogram(&sum,&histo);
    threshold=camFindThreshold(&histo,percent);
    camRLEAllocate(&thresholded,roi.width*roi.height);
    // Build a RLE encoded image
    camRLEEncodeThreshold(&sum,&thresholded,threshold);
    camAllocateImage(&thr,roi.width,roi.height,CAM_DEPTH_8U);
    camRLEDecode(&thresholded,&thr,&lut);
    camSavePGM(&thr,"output/threshold.pgm");

    camDeallocateImage(&sobel_h);
    camDeallocateImage(&sobel_v);
    camDeallocateImage(&sum);
    camDeallocateImage(&thr);
    camDeallocateImage(&yuv);
    camDeallocateImage(&image);
    camRLEDeallocate(&thresholded); 
}

void example_high_pass()
{
    CamImage houghCube;
    CamLinearFilterKernel LoG;

    camLoadPGM(&houghCube, "resources/lsun.pgm");
    
    LoG.kernel[0][0]= 0;
    LoG.kernel[0][1]= 0;
    LoG.kernel[0][2]=-1;
    LoG.kernel[0][3]= 0;
    LoG.kernel[0][4]= 0;
    LoG.kernel[1][0]= 0;
    LoG.kernel[1][1]=-1;
    LoG.kernel[1][2]=-2;
    LoG.kernel[1][3]=-1;
    LoG.kernel[1][4]= 0;
    LoG.kernel[2][0]=-1;
    LoG.kernel[2][1]=-2;
    LoG.kernel[2][2]=16;
    LoG.kernel[2][3]=-2;
    LoG.kernel[2][4]=-1;
    LoG.kernel[3][0]= 0;
    LoG.kernel[3][1]=-1;
    LoG.kernel[3][2]=-2;
    LoG.kernel[3][3]=-1;
    LoG.kernel[3][4]= 0;
    LoG.kernel[4][0]= 0;
    LoG.kernel[4][1]= 0;
    LoG.kernel[4][2]=-1;
    LoG.kernel[4][3]= 0;
    LoG.kernel[4][4]= 0;

    LoG.coeff1=1;
    LoG.coeff2=0;

    camLinearFilter5x5(&houghCube, &houghCube, &LoG);
    camSavePGM(&houghCube, "output/lsum_LoG.pgm");

    camDeallocateImage(&houghCube);
}

void example_histogram()
{
    int i;
    CamImage image;
    CamTable histogram;
    camLoadPGM(&image, "resources/lsun.pgm");
    //camAllocateImage(&image, 256, 256, CAM_DEPTH_8U);
    //camSet(&image, 128);
    camHistogram(&image, &histogram);
    camHistogramEqualization(&image, &image, &histogram, CAM_EQUAL_PERFECT, NULL);
    camSavePGM(&image, "output/lsun_equalized.pgm");
    camHistogram(&image, &histogram);
    for (i = 0;  i < histogram.size; i++) {
       printf("[%d : %d]", i, histogram.t[i]);
    }
    camDeallocateImage(&image);
}

void example_harris()
{
#define CONTRAST 128 
    CamImage image;
    CamKeypoints points;
    int xp[]={32,64,32,8,64,100,100,64,32,48,32,16};
    int yp[]={8,32,64,32,64,64,100,100,70,86,102,86};
    int i;

    printf("Harris corner point detection :\n");
    camAllocateImage(&image, 128, 128, CAM_DEPTH_8U);
    camAllocateKeypoints(&points, 128);
    camSet(&image, 0);
    for (i=0;i<4;i++) {
	camDrawLine(&image, xp[i], yp[i], xp[(i+1)%4], yp[(i+1)%4], CONTRAST);
	camDrawLine(&image, xp[i+4], yp[i+4], xp[4+((i+1)%4)], yp[4+((i+1)%4)], CONTRAST);
	camDrawLine(&image, xp[i+8], yp[i+8], xp[8+((i+1)%4)], yp[8+((i+1)%4)], CONTRAST);
    }
    camFillColor(&image, 32, 32, CONTRAST, -1);
    camFillColor(&image, 65, 65, CONTRAST, -1);
    camFillColor(&image, 32, 86, CONTRAST, -1);
    camFixedFilter(&image, &image, CAM_GAUSSIAN_5x5);
    camHarris(&image, &points, 41);

    for (i=0; i < points.nbPoints; i++) {
	printf("x=%d y=%d mark=%d\n", points.keypoint[i]->x, points.keypoint[i]->y, points.keypoint[i]->value);
	if (points.keypoint[i]->value > 2000) 
	    camPlot(&image, points.keypoint[i]->x, points.keypoint[i]->y, 255, CAM_CROSS);
    }
    camSavePGM(&image, "output/harris.pgm");
    camDeallocateImage(&image);    
    camFreeKeypoints(&points);
}

void example_integralimage()
{
    CamImage image, dest;
    int c, t1, t2;
    
    camLoadPGM(&image, "resources/chess.pgm");
    dest.imageData=NULL; /* in order to use automatic allocation */
    
    t1=camGetTimeMs();
    for (c=0;c<1000;c++) {
	camIntegralImage(&image, &dest);
    }	
    t2=camGetTimeMs();
    printf("Integral image computation = %d us\n",t2-t1);

    camSavePGM(&dest, "output/chess_integral.pgm");
    camDeallocateImage(&image);
    camDeallocateImage(&dest);
}

int camKeypointInternalsPrepareOrientation();

void example_feature_points()
{
    CamImage image, integral, dest;
    CamKeypoints points;
    int i;
    int c, t1, t2;
    const int x = 8;
    char str[256];
    FILE *handle;
    int angle;
    double costheta;
    double sintheta;
    CamROI roi;

    const int xp[4] = {-1, 1, 1, -1};
    const int yp[4] = {-1, -1, 1, 1};
    CamWarpingParams params;

    //camKeypointsInternalsPrepareOrientation();
    
    printf("Feature points detection :\n");
    camAllocateImage(&image, 256, 256, CAM_DEPTH_8U);
    camAllocateKeypoints(&points, 1000);

    camSet(&image, 0);
    camDrawRectangle(&image, 102, 120, 156, 152, 255);
    camFillColor(&image, 103, 121, 255, -1);
    camDrawRectangle(&image, 122, 35, 176, 67, 128);
    camFillColor(&image, 123, 36, 128, -1);

#if 1
    angle = 20;
    costheta = cos(angle * 2 * M_PI / 360);
    sintheta = sin(angle * 2 * M_PI / 360);
    for (i = 0; i < 4; i++) {
	params.p[i].x = (int)floor((costheta * xp[i] - sintheta * yp[i]) * 15 + 0.5);
	params.p[i].y = (int)floor((sintheta * xp[i] + costheta * yp[i]) * 15 + 0.5);
	params.p[i].x += 192;
	params.p[i].y += 192;
    }
    for (i = 0; i < 4; i++) {
	camDrawLine(&image, params.p[i].x, params.p[i].y, params.p[(i+1)%4].x, params.p[(i+1)%4].y, 255);
    }
    camFillColor(&image, 192, 192, 255, -1);
 
    angle = 30;
    costheta = cos(angle * 2 * M_PI / 360);
    sintheta = sin(angle * 2 * M_PI / 360);
    for (i = 0; i < 4; i++) {
	params.p[i].x = (int)floor((costheta * xp[i] - sintheta * yp[i]) * 10 + 0.5);
	params.p[i].y = (int)floor((sintheta * xp[i] + costheta * yp[i]) * 10 + 0.5);
	params.p[i].x += 50;
	params.p[i].y += 192;
    }

    for (i = 0; i < 4; i++) {
	camDrawLine(&image, params.p[i].x, params.p[i].y, params.p[(i+1)%4].x, params.p[(i+1)%4].y, 255);
    }
    camFillColor(&image, 50, 192, 255, -1);

#endif
    integral.imageData = NULL; /* in order to use automatic allocation */
    dest.imageData = NULL; 
	
    t1=camGetTimeMs();
    for (c = 0; c < 100; c++)
    {
	camFastHessianDetector(&image, &points, 2000, 0);
    }	
    t2=camGetTimeMs();
    printf("Fast hessian computation = %d us\n",(t2-t1)*10);
    for (i = 0; i < points.nbPoints; i++) {
	printf("x=%d y=%d value=%d scale=%d size=%d angle=%d\n", points.keypoint[i]->x, points.keypoint[i]->y, points.keypoint[i]->value, points.keypoint[i]->scale, points.keypoint[i]->size, points.keypoint[i]->angle);
    }
    camDrawKeypoints(&points, &image, 128);
    camSavePGM(&image, "output/features_reference.pgm");

    camDeallocateImage(&image);
    camDeallocateImage(&integral);
    camDeallocateImage(&dest);
    camFreeKeypoints(&points);
}

void example_feature_points2()
{
    CamImage image, Y1, Y2;
    CamKeypoints points1, points2;
    int i, nbMatches;
    int dist1, dist2, color;
    CamKeypoint *best;
    float ratio = 2.0f/3;

    printf("Feature point detection on Clooney :\n");
    image.imageData = NULL;
    camLoadBMP(&image, "resources/clooney.bmp");
    Y1.imageData = NULL;
    camRGB2Y(&image, &Y1);
    //    camSavePGM(&Y1, "output/clooney.pgm");
    camAllocateKeypoints(&points1, 100000);
    camAllocateKeypoints(&points2, 100000);
    camAllocateImage(&Y2, (((int)(Y1.width*ratio))/8)*8, (((int)(Y1.height*ratio))/8)*8, CAM_DEPTH_8U);
    camScale(&Y1, &Y2);
    camSavePGM(&Y2, "output/clooney2.pgm");

    camFastHessianDetector(&Y1, &points1, 500, 0);
    camFastHessianDetector(&Y2, &points2, 500, 0);
	
    nbMatches = 0;
    
    for (i = 0; i < points1.nbPoints; i++) {
        best = camFindKeypoint(points1.keypoint[i], &points2, &dist1, &dist2);
	if (dist1 < 0.7 * dist2) {
	    if (fabs(ratio * points1.keypoint[i]->x - best->x) > 10) color = CAM_RGB(0, 0 , 255); 
	    else if (fabs(ratio * points1.keypoint[i]->y - best->y) > 10) color = CAM_RGB(0, 0 , 255); 
	    else {
		color = CAM_RGB(0, 255, 0);
		nbMatches++;
	    }
	} else color = CAM_RGB(255, 0, 0);
	camDrawKeypoint(points1.keypoint[i], &image, color);
	printf("x=%d y=%d mark=%d scale=%d meaning=%lf\n", points1.keypoint[i]->x, points1.keypoint[i]->y, points1.keypoint[i]->value, points1.keypoint[i]->scale, ((double)dist1) / dist2);
    }
    printf("# points found on 1st image: %d\n", points1.nbPoints);
    printf("# points found on 2nd image: %d\n", points2.nbPoints);
    printf("# points matched: %d\n", nbMatches);
    
    camSaveBMP(&image, "output/clooney.bmp");
    camDeallocateImage(&image);
    camDeallocateImage(&Y1);
    camDeallocateImage(&Y2);
    camFreeKeypoints(&points1);
    camFreeKeypoints(&points2);
}

void example_feature_points3()
{
    CamImage image, Y;
    CamKeypoints points;
    int i;
   
    printf("Feature point detection on sunflowers :\n");
    image.imageData = NULL;
    camLoadBMP(&image, "resources/sunflower.bmp");
    Y.imageData = NULL;
    camRGB2Y(&image, &Y);
    camAllocateKeypoints(&points, 100000);

    points.nbPoints = 0;
    camFastHessianDetector(&Y, &points, 1500, 0);
	
    for (i = 0; i < points.nbPoints; i++) {
	printf("x=%d y=%d mark=%d scale=%d\n", points.keypoint[i]->x, points.keypoint[i]->y, points.keypoint[i]->value, points.keypoint[i]->scale);
    }
    printf("# points found : %d\n", points.nbPoints);
    camDrawKeypoints(&points, &image, CAM_RGB(255, 0, 0));
    camSaveBMP(&image, "output/sunflower.bmp");
    camDeallocateImage(&image);
    camDeallocateImage(&Y);
    camFreeKeypoints(&points);
}

void example_capture()
{
    void *handle;
    int i;
    CamImage image;

    image.imageData=NULL; // Necessary in C, so that the first capture allocates an image

    handle=camCaptureInit(CAM_CAPTURE_AUTO_SOURCE|CAM_CAPTURE_DISPLAY);
    if (handle) {
	for (i=0;i<30;i++) {
	    printf("Grabbing frame #%d...\n",i);
	    camCapture(handle,&image);
	}
	camSaveBMP(&image,"output/capture.bmp");
	camCaptureOver(handle);
	camDeallocateImage(&image);
    } else {
	printf("No camera found\n");
    }
}

void cpp_example_erosion();
void cpp_example_mask();
void cpp_example_labeling();
void cpp_example_color_labeling();
void cpp_example_histogram();
void cpp_example_hough_circle();
void cpp_example_fixed_filters();
void cpp_example_copy();
void cpp_example_alpha_composite();
void cpp_example_watershed();
void cpp_example_draw();

int main()
{
    camInitBenchmark();

    // Legacy examples
    example_filters();
    example_warping();
    example_labeling_benchmark();
    example_binary();
    example_morpho();
    example_draw();
    example_watershed();
    example_watershed2();
    example_rle_erosion();
    example_me();
    example_color();
    example_undistort();
    example_color_labeling();
    example_scale();
    example_mask();
    example_sobel();
    example_high_pass();
    example_histogram();
    example_harris();
    example_integralimage();
    example_feature_points();
    example_feature_points2();
    example_feature_points3();
    example_capture();

    // C++ reference examples
    cpp_example_erosion();
    cpp_example_mask();
    cpp_example_labeling();
    cpp_example_color_labeling();
    cpp_example_histogram();
    cpp_example_hough_circle();
    cpp_example_fixed_filters();
    cpp_example_copy();
    cpp_example_alpha_composite();
    cpp_example_watershed();
    cpp_example_draw();
    
    return 0;
}


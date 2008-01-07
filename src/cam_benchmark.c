// Benchmark against OpenCV

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "cv.h"
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

int CompareLinearFilters()
{
    CamImage source,dest,dest16;
    CamLinearFilterKernel kernel;
    CamSepFilterKernel sep_kernel;
    int i,j,t1,t2;

    camInitBenchmark();

#ifdef __INTEL_COMPILER
    printf("Compiled with Intel C++ Compiler\n");
#endif

    // Load picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    
    // Sobel filter on the image (signed)
    camAllocateImage(&dest,source.width,source.height,CAM_DEPTH_8U);
    camAllocateImage(&dest16,source.width,source.height,CAM_DEPTH_16S);
    for (i=0;i<3;i++) {
        for (j=0;j<3;j++) {
            kernel.kernel[i][j]=SOBEL_3x3_V[i][j];
        }
    }
    kernel.coeff1=1;
    kernel.coeff2=0;

    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camLinearFilterAbs3x3(&source,&dest,&kernel);
    }
    t2=camGetTimeMs();
    printf("Linear filter 3x3 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_sobel_abs1.pgm");
    
    sep_kernel.x[0]=-1;
    sep_kernel.x[1]=0;
    sep_kernel.x[2]=1;
    sep_kernel.y[0]=1;
    sep_kernel.y[1]=2;
    sep_kernel.y[2]=1;
    sep_kernel.coeff1=1;
    sep_kernel.coeff2=0;

    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camSepFilterAbs3x3(&source,&dest,&sep_kernel);
    }
    t2=camGetTimeMs();
    printf("Linear separable filter 3x3 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_sobel_abs2.pgm");
    
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camSobelVAbs(&source,&dest);
    }
    t2=camGetTimeMs();
    printf("Accelerated linear separable filter 3x3 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_sobel_abs3.pgm");
    
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camSobelVAbs(&source,&dest);
    }
    t2=camGetTimeMs();
    printf("Sobel Filter 3x3 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_sobel_abs4.pgm");
       
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvSobel(&source,&dest16,1,0,3);
        cvConvertScaleAbs(&dest16,&dest,1,0);
    }
    t2=camGetTimeMs();
    printf("Sobel Filter OpenCV 3x3 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_sobel_opencv_abs.pgm");
       
    // Test gaussian filtering
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camFixedFilter(&source,&dest,CAM_GAUSSIAN_3x3);
    }
    t2=camGetTimeMs();
    printf("Gaussian Filter 3x3 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_gauss_3x3.pgm");
       
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvSmooth(&source,&dest,CV_GAUSSIAN,3,3,0,0);
    }
    t2=camGetTimeMs();
    printf("Gaussian Filter OpenCV 3x3 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_gauss_3x3_opencv.pgm");

    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camFixedFilter(&source,&dest,CAM_GAUSSIAN_5x5);
    }
    t2=camGetTimeMs();
    printf("Gaussian Filter 5x5 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_gauss_5x5.pgm");
       
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvSmooth(&source,&dest,CV_GAUSSIAN,5,5,0,0);
    }
    t2=camGetTimeMs();
    printf("Gaussian Filter OpenCV 5x5 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_gauss_5x5_opencv.pgm");

    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camFixedFilter(&source,&dest,CAM_GAUSSIAN_7x7);
    }
    t2=camGetTimeMs();
    printf("Gaussian Filter 7x7 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_gauss_7x7.pgm");
       
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvSmooth(&source,&dest,CV_GAUSSIAN,7,7,0,0);
    }
    t2=camGetTimeMs();
    printf("Gaussian Filter OpenCV 7x7 = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_gauss_7x7_opencv.pgm");

    camDeallocateImage(&source);
    camDeallocateImage(&dest);
    camDeallocateImage(&dest16);
    return 0;
}

int CompareMorpho()
{
    CamImage source,dest;
    int i,t1,t2;
    IplConvKernel *element;

    camInitBenchmark();

#ifdef __INTEL_COMPILER
    printf("Compiled with Intel C++ Compiler\n");
#endif

    // Load picture chess.pgm
    camLoadPGM(&source,"resources/chess.pgm");
    
    // Sobel filter on the image (signed)
    camAllocateImage(&dest,source.width,source.height,CAM_DEPTH_8U);
    camSetBorder(&source,128);

    element=cvCreateStructuringElementEx(7,7,3,3,CV_SHAPE_ELLIPSE,NULL);
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvErode(&source,&dest,element,1);
    }
    t2=camGetTimeMs();
    printf("Circle7 erosion OpenCV = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_erode_circle7_opencv.pgm");
    cvReleaseStructuringElement(&element);
    
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camErodeCircle7(&source,&dest);
    }
    t2=camGetTimeMs();
    printf("Circle7 erosion = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_erode_circle7.pgm");
   
    element=cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_ELLIPSE,NULL);
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvErode(&source,&dest,element,1);
    }
    t2=camGetTimeMs();
    printf("Circle5 erosion OpenCV = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_erode_circle5_opencv.pgm");
    cvReleaseStructuringElement(&element);
    
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camErodeCircle5(&source,&dest);
    }
    t2=camGetTimeMs();
    printf("Circle5 erosion = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_erode_circle5.pgm");
   
    element=cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvErode(&source,&dest,element,1);
    }
    t2=camGetTimeMs();
    printf("Square3 erosion OpenCV = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_erode_square3_opencv.pgm");
    cvReleaseStructuringElement(&element);
    
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camErodeSquare3(&source,&dest);
    }
    t2=camGetTimeMs();
    printf("Square3 erosion = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_erode_square3.pgm");
   
    camDeallocateImage(&source);
    camDeallocateImage(&dest);
    return 0;
}

int CompareBinary()
{
    CamImage grey,source,dest;
    int i,t1,t2;
/*
    IplConvKernel *element;
*/

    camInitBenchmark();

#ifdef __INTEL_COMPILER
    printf("Compiled with Intel C++ Compiler\n");
#endif

    // Load picture chess.pgm
    camLoadPGM(&grey,"resources/chess.pgm");
    
    // Sobel filter on the image (signed)
    camAllocateImage(&source,grey.width,grey.height,CAM_DEPTH_1U);
    camAllocateImage(&dest,grey.width,grey.height,CAM_DEPTH_1U);

/*
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvThreshold(&grey,&source,128,1,0);
    }
    t2=camGetTimeMs();
    printf("Thresholding OpenCV = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_threshold_opencv.pgm");
*/

    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camThreshold(&grey,&dest,128);
    }
    t2=camGetTimeMs();
    printf("Thresholding = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_threshold.pgm");

/*
    element=cvCreateStructuringElementEx(7,7,3,3,CV_SHAPE_ELLIPSE,NULL);
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvErode(&source,&dest,element,1);
    }
    t2=camGetTimeMs();
    printf("Circle7 erosion OpenCV = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_binary_erode_circle7_opencv.pgm");
    cvReleaseStructuringElement(&element);
*/
    
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camErodeCircle7(&source,&dest);
    }
    t2=camGetTimeMs();
    printf("Circle7 erosion = %dus\n",t2-t1); 
   
/*
    element=cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_ELLIPSE,NULL);
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvErode(&source,&dest,element,1);
    }
    t2=camGetTimeMs();
    printf("Circle5 erosion OpenCV = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_binary_erode_circle5_opencv.pgm");
    cvReleaseStructuringElement(&element);
*/
    
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camErodeCircle5(&source,&dest);
    }
    t2=camGetTimeMs();
    printf("Circle5 erosion = %dus\n",t2-t1); 
   
/*
    element=cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_RECT,NULL);
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        cvErode(&source,&dest,element,1);
    }
    t2=camGetTimeMs();
    printf("Square3 erosion OpenCV = %dus\n",t2-t1); 
    camSavePGM(&dest,"output/chess_binary_erode_square3_opencv.pgm");
    cvReleaseStructuringElement(&element);
*/
    
    t1=camGetTimeMs();
    for (i=0;i<1000;i++) {
        camErodeSquare3(&source,&dest);
    }
    t2=camGetTimeMs();
    printf("Square3 erosion = %dus\n",t2-t1); 
   
    camDeallocateImage(&grey);
    camDeallocateImage(&source);
    camDeallocateImage(&dest);
    return 0;
}

void CompareUndistort()
{
    float intrinsic_matrix[]={729.330f,0,323.557f,0,730.695f,237.079f,0,0,1};
    float dist_coeffs[]={-0.212201f,0.263761f,0.000363f,0.000936f};
    CamImage source,Y,dest;
    CamImage LUTX,LUTY;
    int i,t1,t2;
    CvMat im = cvMat( 3, 3, CV_32F, intrinsic_matrix );
    CvMat dc = cvMat( 4, 1, CV_32F, dist_coeffs );
    CAM_FIXED_POINT intrinsic_matrix_fixed[9],dist_coeffs_fixed[4];

    camInitBenchmark();

#ifdef __INTEL_COMPILER
    printf("Compiled with Intel C++ Compiler\n");
#endif

    camLoadBMP(&source,"resources/DSCN2773.bmp");
    camAllocateImage(&Y,source.width,source.height,IPL_DEPTH_8U);
    camAllocateImage(&dest,source.width,source.height,IPL_DEPTH_8U);
    camRGB2Y(&source,&Y);

    printf("Gray scale distortion correction : \n");

    t1=camGetTimeMs();
    for (i=0;i<100;i++) {
        camUndistort(&Y,&dest,intrinsic_matrix,dist_coeffs);
    }
    t2=camGetTimeMs();
    
    printf("Undistort = %dus\n",(t2-t1)*10); 
    camSavePGM(&dest,"output/DSCN2773gs_undistorted.pgm");

    t1=camGetTimeMs();
    for (i=0;i<100;i++) {
        cvUndistort2(&Y,&dest,&im,&dc);
    }
    t2=camGetTimeMs();
    printf("Undistort OpenCV = %dus\n",(t2-t1)*10); 
    camSavePGM(&dest,"output/DSCN2773gs_undistorted_opencv.pgm");  

    for (i=0;i<9;i++) intrinsic_matrix_fixed[i]=CAM_FLOAT2FIXED(intrinsic_matrix[i],20);
    for (i=0;i<4;i++) dist_coeffs_fixed[i]=CAM_FLOAT2FIXED(dist_coeffs[i],20);

    t1=camGetTimeMs();
    for (i=0;i<100;i++) {
        camUndistortFixed(&Y,&dest,intrinsic_matrix_fixed,dist_coeffs_fixed);
    }
    t2=camGetTimeMs();
    printf("Undistort Fixed Point = %dus\n",(t2-t1)*10); 
    camSavePGM(&dest,"output/DSCN2773gs_undistorted_fixed.pgm");  

    camUndistortBuildLUT(&Y,intrinsic_matrix,dist_coeffs,&LUTX,&LUTY);
    t1=camGetTimeMs();
    for (i=0;i<100;i++) {
        camUndistortLUT(&Y,&dest,&LUTX,&LUTY);
    }
    t2=camGetTimeMs();
    printf("Undistort LUT = %dus\n",(t2-t1)*10); 
    camSavePGM(&dest,"output/DSCN2773gs_undistorted_LUT.pgm");
    camDeallocateImage(&LUTX);
    camDeallocateImage(&LUTY);

    camDeallocateImage(&dest);
    camDeallocateImage(&Y);
    
    printf("Color distortion correction : \n");
    camAllocateRGBImage(&dest,source.width,source.height);

    t1=camGetTimeMs();
    for (i=0;i<100;i++) {
        camUndistort(&source,&dest,intrinsic_matrix,dist_coeffs);
    }
    t2=camGetTimeMs();
    
    printf("Undistort = %dus\n",(t2-t1)*10); 
    camSaveBMP(&dest,"output/DSCN2773_undistorted.bmp");

    t1=camGetTimeMs();
    for (i=0;i<100;i++) {
        cvUndistort2(&source,&dest,&im,&dc);
    }
    t2=camGetTimeMs();
    printf("Undistort OpenCV = %dus\n",(t2-t1)*10); 
    camSaveBMP(&dest,"output/DSCN2773_undistorted_opencv.bmp");  

    for (i=0;i<9;i++) intrinsic_matrix_fixed[i]=CAM_FLOAT2FIXED(intrinsic_matrix[i],20);
    for (i=0;i<4;i++) dist_coeffs_fixed[i]=CAM_FLOAT2FIXED(dist_coeffs[i],20);

    t1=camGetTimeMs();
    for (i=0;i<100;i++) {
        camUndistortFixed(&source,&dest,intrinsic_matrix_fixed,dist_coeffs_fixed);
    }
    t2=camGetTimeMs();
    printf("Undistort Fixed Point = %dus\n",(t2-t1)*10); 
    camSaveBMP(&dest,"output/DSCN2773_undistorted_fixed.bmp");  

    camUndistortBuildLUT(&source,intrinsic_matrix,dist_coeffs,&LUTX,&LUTY);
    t1=camGetTimeMs();
    for (i=0;i<100;i++) {
        camUndistortLUT(&source,&dest,&LUTX,&LUTY);
    }
    t2=camGetTimeMs();
    printf("Undistort LUT = %dus\n",(t2-t1)*10); 
    camSaveBMP(&dest,"output/DSCN2773_undistorted_LUT.bmp");
    camDeallocateImage(&LUTX);
    camDeallocateImage(&LUTY);

    camDeallocateImage(&source);
    camDeallocateImage(&dest);
}

int main()
{
    CompareLinearFilters();
    CompareMorpho();
    CompareBinary();
    CompareUndistort();
    return 0;
}

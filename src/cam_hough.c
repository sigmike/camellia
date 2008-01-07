#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "camellia.h"
#include "camellia_internals.h"

typedef struct {
    int rmin, rmax;
    int best, xbest, ybest, rbest;
    CamImage sobel_h, sobel_v;
    CamImage *hough_cube;
} camHoughCircleInternal;

static const unsigned char CamSinLUT[256]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,57,58,59,60,61,62,63,64,65,66,67,68,68,69,70,71,72,73,74,75,75,76,77,78,79,80,81,82,82,83,84,85,86,87,87,88,89,90,91,92,92,93,94,95,96,96,97,98,99,100,100,101,102,103,103,104,105,106,106,107,108,109,109,110,111,112,112,113,114,114,115,116,117,117,118,119,119,120,121,121,122,123,124,124,125,126,126,127,127,128,129,129,130,131,131,132,133,133,134,134,135,136,136,137,137,138,139,139,140,140,141,142,142,143,143,144,144,145,146,146,147,147,148,148,149,149,150,150,151,152,152,153,153,154,154,155,155,156,156,157,157,158,158,159,159,160,160,160,161,161,162,162,163,163,164,164,165,165,166,166,166,167,167,168,168,169,169,169,170,170,171,171,172,172,172,173,173,174,174,174,175,175,175,176,176,177,177,177,178,178,178,179,179,180,180,180,181};
static const unsigned char CamCosLUT[256]={255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,254,254,254,254,254,254,254,254,254,254,254,253,253,253,253,253,253,253,253,253,252,252,252,252,252,252,251,251,251,251,251,251,250,250,250,250,250,249,249,249,249,249,248,248,248,248,248,247,247,247,247,246,246,246,246,245,245,245,245,244,244,244,244,243,243,243,243,242,242,242,241,241,241,241,240,240,240,239,239,239,238,238,238,237,237,237,236,236,236,236,235,235,235,234,234,234,233,233,233,232,232,231,231,231,230,230,230,229,229,229,228,228,228,227,227,227,226,226,225,225,225,224,224,224,223,223,222,222,222,221,221,221,220,220,220,219,219,218,218,218,217,217,216,216,216,215,215,215,214,214,213,213,213,212,212,211,211,211,210,210,210,209,209,208,208,208,207,207,206,206,206,205,205,205,204,204,203,203,203,202,202,201,201,201,200,200,200,199,199,198,198,198,197,197,197,196,196,195,195,195,194,194,194,193,193,192,192,192,191,191,191,190,190,189,189,189,188,188,188,187,187,186,186,186,185,185,185,184,184,184,183,183,183,182,182,181,181,181,180};

#define ACCUMULATE \
    for (rp=r;rp<=r;rp++) \
    if ((x>=0)&&(x<hci->hough_cube[0].width)&&(y>=0)&&(y<hci->hough_cube[0].height)) { \
        ptr=hci->hough_cube[rp-hci->rmin].imageData+x+y*hci->hough_cube[0].widthStep; \
        val=*ptr+accu; \
        if (val<0) val=0; \
        if (val>255) val=255; \
        *ptr=val; \
        if (val>hci->best) { \
            hci->best=val; \
            hci->xbest=x; \
            hci->ybest=y; \
            hci->rbest=r; \
        } }

void camHoughCircleAccumulate(camHoughCircleInternal *hci, int xs, int ys, int accu)
{
    // Draw two lines in the hough cube starting from (x,y) and following the gradient line
    int rp;
    int gx,gy,dx,dy;
    int sh,sv,x,y,r,div;
    unsigned char *ptr;
    int val;
    
    // Retrieve sobel information
    sh=*((short*)(hci->sobel_h.imageData+ys*hci->sobel_h.widthStep)+xs);
    sv=*((short*)(hci->sobel_v.imageData+ys*hci->sobel_v.widthStep)+xs);

    gx=abs(sv);
    gy=abs(sh);

    for (r=hci->rmin;r<=hci->rmax;r++) {
        if (!gx) {
            dx=0;
            dy=r;
        } else if (gx>gy) {
            div=((gy<<8)+128)/gx;
            if (div==256) {printf("Crash!!!\n");div=255;}
            dy=(r*CamSinLUT[div])>>8;
            dx=(r*(CamCosLUT[div]+1))>>8;
        } else if (!gy) {
            dx=r;
            dy=0;
        } else if (gx==gy) {
            dx=(r*180)>>8;
            dy=(r*180)>>8;
        } else {
            div=((gx<<8)+128)/gy;
            if (div==256) {printf("Crash!!!\n");div=255;}
            dx=(r*CamSinLUT[div])>>8;
            dy=(r*(CamCosLUT[div]+1))>>8;
        }
        if (((sh>0)&&(sv>0))||((sh<0)&&(sv<0))) {
            x=xs+dx;
            y=ys+dy;
            ACCUMULATE;
            x=xs-dx;
            y=ys-dy;
            ACCUMULATE;
        } else {
            x=xs+dx;
            y=ys-dy;
            ACCUMULATE;
            x=xs-dx;
            y=ys+dy;
            ACCUMULATE;
        }            
    }
}

// Generate a LUT for cos/sin
void camGenerateSinLUT()
{
    int gy;
    double alpha,res;
    FILE *handle=fopen("cam_LUT.c","wt");
    fprintf(handle,"unsigned char camSinLUT[256]={");
    // Let's assume gx=256
    for (gy=0;gy<256;gy++) {
        alpha=atan(gy/256.0);
        res=sin(alpha);
        fprintf(handle,"%d",(int)(res*256+0.5));
        if (gy!=255) fprintf(handle,",");
    }
    fprintf(handle,"};\n");
    fprintf(handle,"unsigned char camCosLUT[256]={");
    // Let's assume gx=256
    for (gy=0;gy<256;gy++) {
        alpha=atan(gy/256.0);
        res=cos(alpha);
        fprintf(handle,"%d",(int)(res*256+0.5)-1);
        if (gy!=255) fprintf(handle,",");
    }
    fprintf(handle,"};\n");
    fclose(handle);
}
    
int camHoughCircle(CamImage *image, int percent, int rmin, int rmax, int *xc, int *yc, int *rc)
{
    int i,x,c,d,threshold;
    camHoughCircleInternal hci;
    CamImage sobel_h_abs, sobel_v_abs, sum;
    CamArithmParams p;
    CamInternalROIPolicyStruct iROI;
    CamTable histo;
    CamRLEImage thresholded;
    CamRun *run;

    CAM_CHECK(camHoughCircle,camInternalROIPolicy(image, NULL, &iROI, 1));
    CAM_CHECK_ARGS(camHoughCircle,iROI.nChannels==1);
    
    // Internal structure initialisation
    hci.rmin=rmin;
    hci.rmax=rmax;
    hci.best=0;
    hci.xbest=0;
    hci.ybest=0;
    hci.rbest=0;

    // Hough cube init
    hci.hough_cube=(CamImage*)malloc(sizeof(CamImage)*(rmax-rmin+1));
    for (i=0;i<(rmax-rmin+1);i++) {
        camAllocateImage(&hci.hough_cube[i],iROI.srcroi.width,iROI.srcroi.height,CAM_DEPTH_8U);
        camSet(&hci.hough_cube[i],0);
    }
    // Allocate 16 bits images for the gradient images
    camAllocateImage(&hci.sobel_h,iROI.srcroi.width,iROI.srcroi.height,CAM_DEPTH_12S);
    camAllocateImage(&hci.sobel_v,iROI.srcroi.width,iROI.srcroi.height,CAM_DEPTH_12S);
    camAllocateImage(&sobel_h_abs,iROI.srcroi.width,iROI.srcroi.height,CAM_DEPTH_12U);
    camAllocateImage(&sobel_v_abs,iROI.srcroi.width,iROI.srcroi.height,CAM_DEPTH_12U);
    camAllocateImage(&sum,iROI.srcroi.width,iROI.srcroi.height,CAM_DEPTH_12U);

    // Compute the 16-bits gradient images
    camSobelH(image,&hci.sobel_h);
    camSobelV(image,&hci.sobel_v);

    // Compute the sum image
    p.operation=CAM_ARITHM_ABS;
    camMonadicArithm(&hci.sobel_h,&sobel_h_abs,&p);
    camMonadicArithm(&hci.sobel_v,&sobel_v_abs,&p);
    p.operation=CAM_ARITHM_ADD;
    camDyadicArithm(&sobel_h_abs,&sobel_v_abs,&sum,&p);

    // Threshold the result image to keep "percent" pixels
    camHistogram(&sum,&histo);
    threshold=camFindThreshold(&histo,percent);
    camRLEAllocate(&thresholded,iROI.srcroi.width*iROI.srcroi.height);
    camRLEEncodeThreshold(&sum,&thresholded,threshold);

    // RLE image scanning to process all selected pixels
    run=thresholded.runs;
    for (c=0,x=0;c<thresholded.nbRuns;c++,run++) {
        if (run->value) {
            for (d=0;d<run->length;d++) {
                // This point is good enough. Let's add it to the Hough plan
                camHoughCircleAccumulate(&hci,x,run->line,+1);
                x++;
            }
        } else x+=run->length;
        if (x==thresholded.width) x=0;
    }

    // Destroy the hough cube
    for (i=0;i<(rmax-rmin+1);i++) {
        // sprintf(fn,"output/hough_%d.pgm",i); camSavePGM(&hci.hough_cube[i],fn);
        camDeallocateImage(&hci.hough_cube[i]);
    }
    free(hci.hough_cube);

    // Deallocate the gradient images
    camDeallocateImage(&hci.sobel_h);
    camDeallocateImage(&hci.sobel_v);
    camDeallocateImage(&sobel_h_abs);
    camDeallocateImage(&sobel_v_abs);
    camDeallocateImage(&sum);
    camRLEDeallocate(&thresholded);
    
    // Return results
    *xc=hci.xbest+iROI.srcroi.xOffset;
    *yc=hci.ybest+iROI.srcroi.yOffset;
    *rc=hci.rbest;

    camInternalROIPolicyExit(&iROI);
    return hci.best;
}



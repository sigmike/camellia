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

#include <iostream>
#include <string>
#include <stdlib.h>
#include "camellia.h"

extern "C" void cpp_example_erosion()
{
    CamImage source,dest;
    source.load_pgm("resources/chess.pgm");
    source.erode_square3(dest);
    dest.save_pgm("output/chess_erode_cpp.pgm");
}

extern "C" void cpp_example_mask()
{
    CamImage source,dest,mask;
    CamRLEImage encoded_mask;
    
    // Load picture chess.pgm
    source.load_pgm("resources/chess.pgm");    
    mask.alloc(source.width,source.height);

    // Draw a filled circle in mask
    mask.set(0);
    mask.draw_circle(mask.width/2,mask.height/2,50,255);
    mask.fill_color(mask.width/2,mask.height/2,255);

    // Encode the mask and associate it to the source image
    mask.encode(encoded_mask);
    source.set_mask(encoded_mask);
    
    // Copy the mask
    encoded_mask.inverse();
    dest=source; // Copies only the mask!

    // And then compute a sobel inside
    encoded_mask.inverse();
    source.sobel_v_abs(dest); // Sobel only on the mask 
    dest.save_pgm("output/chess_sobel_mask.pgm");

    // use a non compressed mask
    source.set_mask(mask);
    source.set(0);
    source.save_pgm("output/chess_hole.pgm");
}

extern "C" int sortfunc(const void *a, const void *b)
{
    CamBlobInfo *ba=(CamBlobInfo*)a;
    CamBlobInfo *bb=(CamBlobInfo*)b;
    return bb->surface-ba->surface;
}

extern "C" void cpp_example_labeling()
{
    CamImage image,yuv;
    
    // Load picture alfa156.bmp
    image.load_bmp("resources/alfa156.bmp");    
    image.to_yuv(yuv);
    
    // Consider V plane only
    CamROI roi(3,0,0,yuv.width,yuv.height);
    yuv.set_roi(roi);
    
    // Threshold and encode
    CamRLEImage thr;
    yuv.encode_threshold(thr,150);

    // Labeling
    CamBlobs blobs;
    thr.labeling(blobs);
    
    // Print info
    std::cout<<blobs.nbBlobs<<" blobs detected"<<std::endl;
    
    // Draw rectangles on all detected blobs
    for (int i=0;i<blobs.nbBlobs;i++) {
        image.draw_rectangle(blobs[i].left, blobs[i].top,
            blobs[i].left+blobs[i].width-1, blobs[i].top+blobs[i].height-1,
            CAM_RGB(255,0,0));
    }
    image.save_bmp("output/alfa156_labeling.bmp");
    
    // Find out the biggest blob
    qsort((void*)blobs.blobInfo,blobs.nbBlobs,sizeof(CamBlobInfo),sortfunc);
    std::cout<<"The bigger blob is at position ("<<blobs[0].cx<<","<<blobs[0].cy<<") and its surface is "<<blobs[0].surface<<" pixels"<<std::endl;
}

extern "C" void cpp_example_color_labeling()
{
    CamImage source,YUV;
    CamRLEImage encoded;
    CamBlobs blobs;
    CamTable clusters;

    // Load picture alfa156.bmp
    source.load_bmp("resources/alfa156.bmp");
    source.to_yuv(YUV);

    // Set the color clusters
    const int limits[3*6]={
    //  Ymin Ymax Umin Umax Vmin Vmax
        0,   60,  0,   255, 0,   255, // Black
        230, 255, 0,   255, 0,   255, // White
        0,   255, 0,   255, 140, 255  // Red
    };
    clusters.set(limits,18);
    const int cluster_colors[3]={CAM_RGB(0,0,0),CAM_RGB(255,255,255),CAM_RGB(255,0,0)};
    
    // Threshold and encode
    encoded.encode_color(YUV,clusters);
    std::cout<<"Number of runs : "<<encoded.nbRuns<<std::endl;
    
    // Labeling
    encoded.labeling(blobs);
    std::cout<<blobs.nbBlobs<<" blobs detected"<<std::endl;

    // Print and draw the results 
    for (int i=0;i<blobs.nbBlobs;i++) {
        std::cout<<"Blob #"<<i<<" : Val="<<blobs.blobInfo[i].value<< \
            " ("<<blobs.blobInfo[i].left<<","<<blobs.blobInfo[i].top<<","<< \
            blobs.blobInfo[i].width<<","<<blobs.blobInfo[i].height<<") Surface="<<blobs.blobInfo[i].surface<< \
            std::endl;
        source.draw_rectangle(blobs.blobInfo[i].left, blobs.blobInfo[i].top,
            blobs.blobInfo[i].left+blobs.blobInfo[i].width-1,
            blobs.blobInfo[i].top+blobs.blobInfo[i].height-1,
            cluster_colors[blobs.blobInfo[i].value-1]);
    }

    // Save the result
    source.save_bmp("output/alfa156_color_labeling.bmp");
}

extern "C" void cpp_example_histogram()
{
    CamImage source,yuv,result;
    // load picture alfa156.bmp
    source.load_bmp("resources/alfa156.bmp");
    // convert to YUV
    source.to_yuv(yuv);
    // compute the histogram
    yuv.histogram_2_channels(2,3,result);
    // save the result
    result.save_pgm("output/alfa156_histo.pgm");
}

extern "C" int camHoughCircle(CamImage *image, int percent, int rmin, int rmax, int *x, int *y, int *r);

extern "C" void cpp_example_hough_circle()
{
    int x,y,r,res;
    CamImage image(100,100);
    // Draw a filled circle in mask
    image.set(0);
    image.draw_circle(40,40,50,128);
    image.fill_color(40,40,128);
    res=image.hough_circle(10,40,60,x,y,r);
    std::cout<<"Found a circle at ("<<x<<","<<y<<") (radius="<<r<<") (confidence="<<res<<")"<<std::endl;
    
    for (int i=0;i<3;i++) {
        // Load an image with a road sign
        char filename[256];
        sprintf(filename,"resources/road%d.bmp",i+1);
        image.load_bmp(filename);
        CamImage yuv;
        image.to_yuv(yuv);
        CamROI roi(1,8,8,image.width-16,image.height-128);
        yuv.set_roi(roi);
        yuv.fixed_filter(CAM_GAUSSIAN_3x3);
        //if (i==0) yuv.erode_circle7(); // First picture needs filtering
        res=yuv.hough_circle(100,6,25,x,y,r);
        image.draw_circle(x,y,r,CAM_RGB(255,0,0));
        sprintf(filename,"output/road_circle_hough%d.bmp",i+1);
        image.save_bmp(filename);
        std::cout<<"Found a circle at ("<<x<<","<<y<<") (radius="<<r<<") (confidence="<<res<<")"<<std::endl;
    }
}   

extern "C" void cpp_example_copy()
{
    CamImage source,dest;
    // Load an image
    source.load_bmp("resources/alfa156.bmp");
    // Allocate a 4 channels image
    dest.alloc_rgba(source.width,source.height);
    // 3 channels -> 4 channels image
    source.copy(dest);
    // And this works...
    dest.save_bmp("output/copy_alfa156.bmp");

    CamImage yuv;
    // Converts to YUV
    source.to_yuv(yuv);
    // Create a ROI for the V channel
    CamROI roi(yuv,3);
    yuv.set_roi(roi);
    CamImage dest2;
    // Copy the V channel to dest2 image
    yuv.copy(dest2);
    // Save the result
    dest2.save_pgm("output/copy_alfa156_v.pgm");
}

extern "C" void cpp_example_alpha_composite()
{
    CamImage source,dest,mask;
    // Load an image
    source.load_bmp("resources/alfa156.bmp");
    // Allocate a 4 channels image
    mask.alloc_rgba(source.width,source.height);
    // Draw a rectangle in the mask image    
    mask.set(0);
    mask.draw_rectangle(72,20,260,180,CAM_RGBA(0,255,0,64));
    mask.fill_color(73,21,CAM_RGBA(0,255,0,64));
    // Alpha channel compositing
    source.alpha_composite(mask,dest);
    // Save the result
    dest.save_bmp("output/alpha_composite_alfa156.bmp");
}

extern "C" void cpp_example_fixed_filters()
{
    CamImage source,result;
    // load picture chess.pgm
    source.load_pgm("resources/chess.pgm");
    // gaussian filtering 3x3
    source.fixed_filter(result,CAM_GAUSSIAN_3x3);
    // save the result
    result.save_pgm("output/chess_gaussian_3x3.pgm");
    // gaussian filtering 5x5
    source.fixed_filter(result,CAM_GAUSSIAN_5x5);
    // save the result
    result.save_pgm("output/chess_gaussian_5x5.pgm");
    // gaussian filtering 7x7
    source.fixed_filter(result,CAM_GAUSSIAN_7x7);
    // save the result
    result.save_pgm("output/chess_gaussian_7x7.pgm");
}

// Watershed segmentation example
extern "C" void cpp_example_watershed()
{
    int i,j,c;
    const char filename[4][30]={"lsun","lsun_32x64e20-bord","lsun_48x80e20","lsun_56x88e20"};

    for (c=0;c<4;c++) {
        CamImage source, watershed;
        CamTableOfBasins tob;
        std::string s="resources/";
        s+=filename[c];
        s+=".pgm";
        std::cout<<std::endl<<"Watershed on "<<s<<" :";
        source.load_pgm(s.c_str());
        source.hierarchical_watershed(watershed,tob);

        for (i=0;i<tob.nbBasins;i++) {
            if (tob[i].dynamics<40){
                tob.get_rid_of(tob[i]);
            }
        }
        for (i=0;i<tob.nbBasins;i++) {
            if ((tob[i].surface!=0)||(tob[i].dynamics==CAM_NOT_COMPUTED)) {
                std::cout<<std::endl<<"Basin #"<<i<<" : Dynamics = "<<tob[i].dynamics<<", Minimum = "<<tob[i].minimum<< ", Surface = "<<tob[i].accsurface<<", (x,y)=("<<tob[i].x<<","<<tob[i].y<<")";
                j=i;
                while (tob[j].dynamics!=CAM_NOT_COMPUTED) {
                    std::cout<<"->"<<tob[j].flooded;
                    j=tob[j].flooded;
                }
            }
        }
        watershed.hierarchical_watershed_regions(tob);
        s="output/watershed_regions_";
        s+=filename[c];
        s+=".pgm";
        watershed.save_pgm(s.c_str());
    }
    std::cout<<std::endl;
}

// Drawing example
extern "C" void cpp_example_draw()
{
    CamImage image(320,240,CAM_DEPTH_8U,CAM_COLORMODEL_RGB);
    CamBitmapFont font("resources/fonts/xenon2.bmp");
    image.set(0);
    image.draw_line(0,0,320,240,CAM_RGB(255,0,0));
    image.fill_color(50,100,CAM_RGB(255,0,0));
    image.draw_text_bitmap("Hello World",100,140,font);
    image.save_bmp("output/drawing_cpp.bmp");
}

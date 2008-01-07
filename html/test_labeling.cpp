#include <iostream.h>
#include <stdlib.h>
#include "camellia.h"

extern "C" int sortfunc(const void *a, const void *b)
{
    CamBlobInfo *ba=(CamBlobInfo*)a;
    CamBlobInfo *bb=(CamBlobInfo*)b;
    return bb->surface-ba->surface;
}

void cpp_example_labeling()
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

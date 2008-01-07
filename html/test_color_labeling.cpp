#include <iostream>
#include "camellia.h"

void cpp_example_color_labeling()
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
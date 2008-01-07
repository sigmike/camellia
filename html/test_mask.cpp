#include "camellia.h"

void cpp_example_mask()
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
}



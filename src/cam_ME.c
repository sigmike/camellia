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

#include <stdlib.h>
#include "camellia.h"
#include "camellia_internals.h"

#define MAX_CANDIDATES 10

int camMotionEstimation3DRSInit(CamMotionEstimation3DRSParams *params, int seed, int lsearch, int rsearch, int bs, int scans, int candidates, int test0)
{
    params->niter=0;
    params->seed=seed;
    params->lsearch=lsearch;
    params->rsearch=rsearch;
    if ((bs!=8)&&(bs!=16)) return 0;
    params->blockSize=bs;
    params->scans=scans;
    if (candidates>MAX_CANDIDATES) return 0;
    params->candidates=candidates;
    params->test0=test0;
    return 1;
}

int camMotionEstimation3DRS(CamImage *current, CamImage *previous, CamMotionEstimation3DRSParams *params, CamMotionEstimation3DRSResults *results)
{
    int scan;
    int cx[MAX_CANDIDATES],cy[MAX_CANDIDATES];
    int candidates;
    int x,y,sx,sy,i;
    int px,py,dir,np,c,a,b;
    int result;

    sx=current->width/params->blockSize;
    sy=current->height/params->blockSize;

    for (py=0;py<sy;py++) {
        for (px=0;px<sx;px++) {
            results->SAD[px][py]=256*16*16;
            results->vx[px][py]=0;
            results->vy[px][py]=0;
        }
    }
    // Proceed for all the scans
    for (scan=0;scan<params->scans;scan++) {
        for (y=0;y<sy;y++) {
            for (x=0;x<sx;x++) {
                // Check scan direction
                dir=scan&1;
                if (dir) {
                    px=sx-1-x;
                    py=sy-1-y;
                } else {
                    px=x;
                    py=y;
                    dir=-1;
                }
                // Check for (0,0) candidate
                if ((scan==0)&&(params->test0)) {
                    cx[0]=0; cy[0]=0; candidates=1;
                } else candidates=0;
                // Check for temporal candidates
                // Only for the first scan
                if ((params->niter!=0)&&(scan==0)) {
                    cx[candidates]=results->vx[px][py];
                    cy[candidates]=results->vy[px][py];
                    candidates++;
                }
                // Check for spatial candidates
                if (x) {
                    if ((scan==0)||(results->vx[px+dir][py]!=results->vx[px][py])||(results->vy[px+dir][py]!=results->vy[px][py])) {
                        cx[candidates]=results->vx[px+dir][py];
                        cy[candidates]=results->vy[px+dir][py];
                        candidates++;
                    }
                }
                if (y) {
                    if ((scan==0)||(results->vx[px][py+dir]!=results->vx[px][py])||(results->vy[px][py+dir]!=results->vy[px][py])) {
                        cx[candidates]=results->vx[px][py+dir];
                        cy[candidates]=results->vy[px][py+dir];
                        candidates++;
                    }
                }
                // Fill the remaining candidates randomly
                // Based on the best current results, the other proposed candidates
                np=candidates+1;
                while (candidates<params->candidates) {
                    c=rand()%np;
                    if (c==candidates) {
                        // Choose the best results as a basis for randomness
                        if (scan) {
                            // We already have a best result
                            cx[candidates]=results->vx[px][py];
                            cy[candidates]=results->vy[px][py];
                        } else {
                            // Full random search
                            cx[candidates]=(rand()%((params->rsearch<<1)+1))-params->rsearch;
                            cy[candidates]=(rand()%((params->rsearch<<1)+1))-params->rsearch;
                        }
                    } else {
                        cx[candidates]=cx[c];
                        cy[candidates]=cy[c];
                    }
                    do {
                        a=(rand()%((params->lsearch<<1)+1))-params->lsearch;
                        b=(rand()%((params->lsearch<<1)+1))-params->lsearch;
                    } while ((a==0)&&(b==0));
                    cx[candidates]+=a;
                    cy[candidates]+=b;
                    candidates++;
                }
            
                if (params->blockSize==16) {
                    for (i=0;i<candidates;i++) {
                        if (cx[i]<-100) {
                            return 0;
                        }
                        result=camSAD16x16(previous, current, px<<4, py<<4, cx[i], cy[i]);
                        if (result<results->SAD[px][py]) {
                            results->SAD[px][py]=result;
                            results->vx[px][py]=cx[i];
                            results->vy[px][py]=cy[i];
                        }
                    }
                } else if (params->blockSize==8) {
                    results->SAD[px][py]=256*8*8;
                    for (i=0;i<candidates;i++) {
                        result=camSAD8x8(previous, current, px<<3, py<<3, cx[i], cy[i]);
                        if (result<results->SAD[px][py]) {
                            results->SAD[px][py]=result;
                            results->vx[px][py]=cx[i];
                            results->vy[px][py]=cy[i];
                        }
                    }
                } else return 0;
            }
        }
    }
    params->niter++;
    return 1;
}

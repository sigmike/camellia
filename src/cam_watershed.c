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

#ifndef CAM_INLINED
#define CAM_INLINED

/* Histogram computation
 * C code */
#include <stdlib.h>
#include "camellia.h"
#include "camellia_internals.h"

int camWatershedCompare(const void *a, const void *b) {
    return ( *(int*)a - *(int*)b );
}

#define CAM_WATERSHED1D_MAXSIZE 1024
#define NEIGHBORHOOD 8

int camWatershed1D(int *t, int s, int *res)
{
    int flood[CAM_WATERSHED1D_MAXSIZE];
    int sorted[CAM_WATERSHED1D_MAXSIZE][2];
    int i,n,p,p1,p2,k1,k2;

    CAM_CHECK_ARGS(camWatershed1D, (s<=CAM_WATERSHED1D_MAXSIZE));

    for (i=0;i<s;i++) {
	sorted[i][0]=t[i];
	sorted[i][1]=i; // Index
	flood[i]=-1;
	res[i]=0;
    }
    qsort(sorted,s,sizeof(int)*2,camWatershedCompare);

    for (i=0;i<s;i++) {
	// Retrieve the current point
	p=sorted[i][1];
	
	// Look at the neighbors. Is it a watershed?
	n=0;
	if ((p>0) && (flood[p-1]!=-1)) {
	    // The left neighbor has already been flooded
	    n++;
	    flood[p]=flood[p-1];
	}
	if ((p<s-1) && (flood[p+1]!=-1)) {
	    // The right neighbor has already been flooded
	    n++;
	    flood[p]=flood[p+1];
	}
	if (n==0) { // This is a lower point
	    flood[p]=p;
	} else if (n==2) { // This is a watershed point !
	    // What is the depth of this watershed ?
	    k1=p-1;
	    while (flood[k1]!=k1) k1=flood[k1];
	    p1=t[k1];
	    k2=p+1;
	    while (flood[k2]!=k2) k2=flood[k2];
	    p2=t[k2];
	    // And destroy the smaller catchment bassin
	    if (p1>p2) {
		res[p]=t[p]-p1;
		flood[k1]=flood[k2];
		//flood[p]=flood[k2]; // Not needed, since this watershed is not reachable any more
	    } else {
		res[p]=t[p]-p2;
		flood[k2]=flood[k1];
		//flood[p]=flood[k1]; // Not needed, since this watershed is not reachable any more
	    }
	}
    }
    return 1;
}

#define CAM_DELTA 2048

// Queue management
#define FIFO_SIZE	CAM_MAX_SCANLINE*CAM_MAX_FRAME_HEIGHT
#define FIFO_FIRST()	queue[first++]; if (first==FIFO_SIZE) first=0
#define FIFO_ADD(x)	queue[last++]=(x); if (last==FIFO_SIZE) last=0
#define FIFO_EMPTY()	(first==last)

#define WSHED	-32768
#define INIT    -32767	// Initial value for destination picture
#define MASK    -32766	// Initial value given to each level
#define INQUEUE -32765	// Value of a pixel when it is inserted in the queue

void camInitTableOfBasins(CamTableOfBasins *t) 
{
    t->sizeMax=CAM_DELTA;
    if ((t->tab=(CamBasin*)malloc(t->sizeMax*sizeof(CamBasin)))==NULL)
	exit(0);
    t->nbBasins=0;
}

void camAddBasin(CamTableOfBasins *t)
{
    if ((++t->nbBasins)>=t->sizeMax) {
	t->sizeMax+=CAM_DELTA;
	if ((t->tab=(CamBasin*)realloc(t->tab,t->sizeMax*sizeof(CamBasin)))==NULL)
	    exit(0);
    }
}

void camFreeTableOfBasins(CamTableOfBasins *t)
{
    if (t->tab) free(t->tab);
    t->nbBasins=0;
    t->sizeMax=0;
    t->tab=NULL;
}

int camHierarchicalWatershedRegions(CamImage *watershed, CamTableOfBasins *tob)
{
    int i,b,c,d,x,y;
    int *queue,first=0,last=0,p,pp;
    int width,height,left,top;
    int neighbor[NEIGHBORHOOD];
    short *im,*ptr=(short*)watershed->imageData;

    CAM_CHECK_ARGS(camHierarchicalWatershedRegions, ((watershed->nChannels==1)&&(watershed->depth==CAM_DEPTH_16S)));

    // Memory allocation
    queue=(int*)malloc(sizeof(int)*FIFO_SIZE);

    // Initialize neighborhood
#if NEIGHBORHOOD==8
    neighbor[0]=-(watershed->widthStep/2)-1;
    neighbor[1]=-(watershed->widthStep/2);
    neighbor[2]=-(watershed->widthStep/2)+1;
    neighbor[3]=-1;
    neighbor[4]=+1;
    neighbor[5]=+(watershed->widthStep/2)-1;
    neighbor[6]=+(watershed->widthStep/2);
    neighbor[7]=+(watershed->widthStep/2)+1;
#endif
    
    // ROI (Region Of Interest) management
    if (watershed->roi) {
	width=watershed->roi->width;
	height=watershed->roi->height;
        left=watershed->roi->xOffset; top=watershed->roi->yOffset;
    } else {
	left=0; top=0;
        width=watershed->width;
	height=watershed->height;
    }
   
    for (y=0;y<height;y++) {
        im=((short*)watershed->imageData)+(top+y)*watershed->widthStep/2+left;
        for (x=0;x<width;x++,im++) {    
            d=*im;
            if (d==WSHED) *im=0;
            else if (d!=0) {
                c=(d<0)?-d:d;
                if (tob->tab[c-1].surface==0) {
                    b=c-1;
                    while ((tob->tab[b].surface==0)&&(tob->tab[b].dynamics!=CAM_NOT_COMPUTED)) {
                        b=tob->tab[b].flooded;
                    } 
                    if (b!=c-1) {
                        // b is the first basin in the tree that was not deselected (i.e. dynamics set to 0)
                        p=im-ptr;
                        FIFO_ADD(p);
                        ptr[p]=b+1;
                        while (!FIFO_EMPTY()) {
                            p=FIFO_FIRST();
                            // For all pixel p'' in the neighborhood of p'
                            for (i=0;i<NEIGHBORHOOD;i++) {
                                pp=p+neighbor[i];
                                if ((ptr[pp]==c)||(ptr[pp]==-(c))) {
                                    FIFO_ADD(pp);
                                    ptr[pp]=b+1;
                                }
                            }
                        }
                    }
                } else if (d<0) *im=-d;
            }
        }
    }

    free(queue);
    return 1;
}

#ifdef CAM_GENERATE_FULL_CODE

// 8 and 16 bits pixel size code generation

#undef CAM_PIXEL
#define CAM_PIXEL unsigned char
#define camHierarchicalWatershed camHierarchicalWatershed8
#define camHierarchicalWatershedContours camHierarchicalWatershedContours8
#include "cam_watershed.c"
#undef camHierarchicalWatershed
#undef camHierarchicalWatershedContours
  
#undef CAM_PIXEL
#define CAM_PIXEL unsigned short
#define camHierarchicalWatershed camHierarchicalWatershed16
#define camHierarchicalWatershedContours camHierarchicalWatershedContours16
#include "cam_watershed.c"
#undef camHierarchicalWatershed
#undef camHierarchicalWatershedContours
  
int camHierarchicalWatershed(CamImage *image, CamImage *dest, CamTableOfBasins *tob)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camHierarchicalWatershed16(image,dest,tob);
    } else {
	return camHierarchicalWatershed8(image,dest,tob);
    }
}

int camHierarchicalWatershedContours(CamImage *image, CamImage *dest, CamTableOfBasins *tob)
{
    if ((image->depth&CAM_DEPTH_MASK)>8) {
	return camHierarchicalWatershedContours16(image,dest,tob);
    } else {
	return camHierarchicalWatershedContours8(image,dest,tob);
    }
}

#else
#include "cam_watershed.c"
#endif

#else
int camHierarchicalWatershed(CamImage *source, CamImage *dest, CamTableOfBasins *tob)
{
    CamTable histo;
    int cumulated[1<<(sizeof(CAM_PIXEL)*8)];
    int *queue,first=0,last=0;
    int basin1,basin2,basin1p,basin2p,label=0;
    CAM_PIXEL *src,*srcx;
    CamImage c;
    CamROI roi,roi2;
    int *it,ptr,ptr2,p,pp,ppp;
    int width,height,left,top,b,i,x,y,h,flag;
    short *dst;
    int neighbor[NEIGHBORHOOD];
    
    CAM_CHECK_ARGS(camHierarchicalWatershed, ((source->nChannels==1)&&((source->depth&CAM_DEPTH_MASK)==(sizeof(CAM_PIXEL)*8))));
    CAM_CHECK_ARGS(camHierarchicalWatershed, ((dest->nChannels==1)&&(dest->depth==CAM_DEPTH_16S)));
    CAM_CHECK_ARGS(camHierarchicalWatershed, (dest->widthStep*sizeof(CAM_PIXEL)==source->widthStep*sizeof(unsigned short)));

    // ROI (Region Of Interest) management
    if (source->roi) {
	width=source->roi->width-2;
	height=source->roi->height-2;
        src=(CAM_PIXEL*)(source->imageData+source->roi->yOffset*source->widthStep)+source->roi->xOffset;
        dst=((short*)dest->imageData)+source->roi->yOffset*source->widthStep+source->roi->xOffset;
        left=source->roi->xOffset; top=source->roi->yOffset;
        roi=*source->roi;
	camReduceROI(&roi,1);
    } else {
	src=(CAM_PIXEL*)(source->imageData);
        dst=(short*)dest->imageData;
	left=0; top=0;
        width=source->width-2;
	height=source->height-2;
	camSetROI(&roi,0,1,1,width,height);
    }

    // Memory allocation
    queue=(int*)malloc(sizeof(int)*FIFO_SIZE);
    it=(int*)malloc(sizeof(int)*(width*height+1));

    // Compute the histogram for the source image (with ROI reduced by 1 pixel on each side)
    c=*source;
    c.roi=&roi;
    camHistogram(&c,&histo);

    // Fill the dest image with INIT values
    c=*dest;
    c.roi=&roi;
    camSet(&c,INIT);
    // Set borders to WSHED
    c.roi=&roi2;
    camSetROI(&roi2,0,roi.xOffset-1,roi.yOffset-1,width+2,1);
    camSet(&c,WSHED);
    camSetROI(&roi2,0,roi.xOffset-1,roi.yOffset+height,width+2,1);
    camSet(&c,WSHED);
    camSetROI(&roi2,0,roi.xOffset-1,roi.yOffset,1,height);
    camSet(&c,WSHED);
    camSetROI(&roi2,0,roi.xOffset+width,roi.yOffset,1,height);
    camSet(&c,WSHED);

    // Initialize neighborhood
#if NEIGHBORHOOD==8
    neighbor[0]=-source->widthStep-1;
    neighbor[1]=-source->widthStep;
    neighbor[2]=-source->widthStep+1;
    neighbor[3]=-1;
    neighbor[4]=+1;
    neighbor[5]=+source->widthStep-1;
    neighbor[6]=+source->widthStep;
    neighbor[7]=+source->widthStep+1;
#endif
    
    // Computes cumulated histogram
    cumulated[0]=0;
    for (i=1;i<(1<<(source->depth&CAM_DEPTH_MASK));i++) {
	cumulated[i]=cumulated[i-1]+histo.t[i-1];
    }
    
    // Sort all the pixels
    for (y=0;y<height;y++) {
        srcx=(CAM_PIXEL*)((char*)(src)+(y+1)*source->widthStep)+1;
	for (x=0;x<width;x++,srcx++) {
	    it[cumulated[*srcx]++]=srcx-src;
	}
    }

    // Init the table of basins
    camInitTableOfBasins(tob);

    // Special case : All the pixels have the same gray level
    if (src[it[width*height-1]]==src[it[0]]) {
	// In that case, there is only one bassin
	camAddBasin(tob);
	tob->tab[0].dynamics=CAM_NOT_COMPUTED;
	tob->tab[0].minimum=src[it[0]];
	tob->tab[0].flooded=0;
	tob->tab[0].x=1;
	tob->tab[0].y=1;
        tob->tab[0].surface=width*height;
        tob->tab[0].accsurface=width*height;
	free(it);
	free(queue);
	return 1;
    }

    it[width*height]=it[0];

    for (h=0,ptr=0,ptr2=0;h<(1<<(source->depth&CAM_DEPTH_MASK));h++) {
	// Geodesic SKIZ of level h-1 into level h
	ptr2=ptr;
	// For all pixels p with I[p]=h
	// All these pixels are directly adressable thanks to the initial pixels sort
	while (src[p=it[ptr2]]==h) {
	    ptr2++;

	    // If there is a pixel p' in the neighborhood with dst[p']>0
	    for (flag=0,i=0;i<NEIGHBORHOOD;i++) {
		pp=p+neighbor[i];
		if (dst[pp]>0) {
		    flag=1;
		    break;
		}
	    }
	    if (flag) {
		dst[p]=INQUEUE;
		FIFO_ADD(p);
	    } else {
		dst[p]=MASK;
	    }
	}

	while (!FIFO_EMPTY()) {
	    p=FIFO_FIRST();
	    // For all pixels p' belonging to p's neighborhood
	    for (flag=0,i=0;i<NEIGHBORHOOD;i++) {
		pp=p+neighbor[i];
		if (dst[pp]>0) {
		    // p' belongs to an already labelled basin
		    if (dst[p]==INQUEUE) {
			dst[p]=dst[pp];
                        tob->tab[dst[p]-1].surface++;
		    } else if ((dst[p]>0)&&(dst[p]!=dst[pp])) {
			basin1=basin1p=dst[p]-1;
			basin2=basin2p=dst[pp]-1;
			// It is necessary to update to dynamics of basins
			// We go up the flooding list to retrieve the two
			// big basins that flood
			while (tob->tab[basin1].dynamics!=CAM_NOT_COMPUTED) {
			    basin1=tob->tab[basin1].flooded;
			}
			while (tob->tab[basin2].dynamics!=CAM_NOT_COMPUTED) {
			    basin2=tob->tab[basin2].flooded;
			}
			if (basin1!=basin2) {
			    // It is a saddle point : we compare the minima
			    if (tob->tab[basin1].minimum<tob->tab[basin2].minimum) {
				// In that case, basin1 is flooding basin2
				tob->tab[basin2].flooded=basin1;
				// And we can compute the dynamics of basin2
				tob->tab[basin2].dynamics=src[p]-tob->tab[basin2].minimum;
			    } else {
				// In that case, basin2 is flooding basin1
				tob->tab[basin1].flooded=basin2;
				// And we can compute the dynamics of basin1
				tob->tab[basin1].dynamics=src[p]-tob->tab[basin1].minimum;
				// Point in the watershed between p and p'
				dst[p]=dst[pp];
                                tob->tab[basin1p].surface--;
				tob->tab[basin2p].surface++;
			    }
			} else {
			    // Which one has the lower dynamic ?
			    if (tob->tab[basin2p].dynamics>tob->tab[basin1p].dynamics) {
				// Point in the watershed between p and p'
				dst[p]=dst[pp];
                                tob->tab[basin1p].surface--;
				tob->tab[basin2p].surface++;                                
			    }
			}
		    }
		}
	    }
	    for (i=0;i<NEIGHBORHOOD;i++) {
	    	pp=p+neighbor[i];
		if (dst[pp]==MASK) {
		    dst[pp]=INQUEUE;
		    FIFO_ADD(pp);
		}
	    }
	}

	// Some new minima have appeared ?
	// For any pixel p with src[p]=h
	while (src[p=it[ptr]]==h) {
	    ptr++;

	    if (dst[p]==MASK) {
		label++;
		FIFO_ADD(p);
		dst[p]=label;
		// Append this new basin to the table of basins
		camAddBasin(tob);
		tob->tab[label-1].dynamics=CAM_NOT_COMPUTED;
		tob->tab[label-1].minimum=h;
		tob->tab[label-1].flooded=0;
		tob->tab[label-1].x=(p%source->widthStep)+left;
		tob->tab[label-1].y=(p/source->widthStep)+top;
                tob->tab[label-1].surface=1;
                tob->tab[label-1].accsurface=0;

		while (!FIFO_EMPTY()) {
		    pp=FIFO_FIRST();
		    // For all pixel p'' in the neighborhood of p'
		    for (i=0;i<NEIGHBORHOOD;i++) {
			ppp=pp+neighbor[i];
			if (dst[ppp]==MASK) {
			    FIFO_ADD(ppp);
			    dst[ppp]=label;
                            tob->tab[label-1].surface++;
			}
		    }
		}
	    }
	}			
    }

    // Memory free
    free(it);
    free(queue);

    // Accumulate surface to the top of the tree
    for (i=0;i<tob->nbBasins;i++) {
        tob->tab[i].accsurface+=tob->tab[i].surface;
	b=i;
	while (tob->tab[b].dynamics!=CAM_NOT_COMPUTED) {
            tob->tab[tob->tab[b].flooded].accsurface+=tob->tab[i].surface;
	    b=tob->tab[b].flooded;
        }
    }

    return 1;
}

int camHierarchicalWatershedContours(CamImage *source, CamImage *dest, CamTableOfBasins *tob)
{
    CamTable histo;
    int cumulated[1<<(sizeof(CAM_PIXEL)*8)];
    int *queue,first=0,last=0;
    int basin1,basin2,basin1p,basin2p,label=0;
    CAM_PIXEL *src,*srcx;
    CamImage c;
    CamROI roi,roi2;
    int *it,ptr,ptr2,p,pp,ppp;
    int width,height,left,top,b,i,x,y,h,flag;
    short *dst;
    int neighbor[NEIGHBORHOOD];

    CAM_CHECK_ARGS(camHierarchicalWatershedContours, ((source->nChannels==1)&&((source->depth&CAM_DEPTH_MASK)==(sizeof(CAM_PIXEL)*8))));
    CAM_CHECK_ARGS(camHierarchicalWatershedContours, ((dest->nChannels==1)&&(dest->depth==CAM_DEPTH_16S)));
    CAM_CHECK_ARGS(camHierarchicalWatershedContours, (dest->widthStep*sizeof(CAM_PIXEL)==source->widthStep*sizeof(unsigned short)));

    // ROI (Region Of Interest) management
    if (source->roi) {
	width=source->roi->width-2;
	height=source->roi->height-2;
        src=(CAM_PIXEL*)(source->imageData+source->roi->yOffset*source->widthStep)+source->roi->xOffset;
        dst=((short*)dest->imageData)+source->roi->yOffset*source->widthStep+source->roi->xOffset;
        left=source->roi->xOffset; top=source->roi->yOffset;
        roi=*source->roi;
	camReduceROI(&roi,1);
    } else {
	src=(CAM_PIXEL*)(source->imageData);
        dst=(short*)dest->imageData;
	left=0; top=0;
        width=source->width-2;
	height=source->height-2;
	camSetROI(&roi,0,1,1,width,height);
    }

    // Memory allocation
    queue=(int*)malloc(sizeof(int)*FIFO_SIZE);
    it=(int*)malloc(sizeof(int)*(width*height+1));

    // Compute the histogram for the source image (with ROI reduced by 1 pixel on each side)
    c=*source;
    c.roi=&roi;
    camHistogram(&c,&histo);

    // Fill the dest image with INIT values
    c=*dest;
    c.roi=&roi;
    camSet(&c,INIT);
    // Set borders to WSHED
    c.roi=&roi2;
    camSetROI(&roi2,0,roi.xOffset-1,roi.yOffset-1,width+2,1);
    camSet(&c,WSHED);
    camSetROI(&roi2,0,roi.xOffset-1,roi.yOffset+height,width+2,1);
    camSet(&c,WSHED);
    camSetROI(&roi2,0,roi.xOffset-1,roi.yOffset,1,height);
    camSet(&c,WSHED);
    camSetROI(&roi2,0,roi.xOffset+width,roi.yOffset,1,height);
    camSet(&c,WSHED);

    // Initialize neighborhood
#if NEIGHBORHOOD==8
    neighbor[0]=-source->widthStep-1;
    neighbor[1]=-source->widthStep;
    neighbor[2]=-source->widthStep+1;
    neighbor[3]=-1;
    neighbor[4]=+1;
    neighbor[5]=+source->widthStep-1;
    neighbor[6]=+source->widthStep;
    neighbor[7]=+source->widthStep+1;
#endif
    
    // Computes cumulated histogram
    cumulated[0]=0;
    for (i=1;i<(1<<(source->depth&CAM_DEPTH_MASK));i++) {
	cumulated[i]=cumulated[i-1]+histo.t[i-1];
    }
    
    // Sort all the pixels
    for (y=0;y<height;y++) {
        srcx=(CAM_PIXEL*)((char*)(src)+(y+1)*source->widthStep)+1;
	for (x=0;x<width;x++,srcx++) {
	    it[cumulated[*srcx]++]=srcx-src;
	}
    }

    // Init the table of basins
    camInitTableOfBasins(tob);

    // Special case : All the pixels have the same gray level
    if (src[it[width*height-1]]==src[it[0]]) {
	// In that case, there is only one bassin
	camAddBasin(tob);
	tob->tab[0].dynamics=CAM_NOT_COMPUTED;
	tob->tab[0].minimum=src[it[0]];
	tob->tab[0].flooded=0;
	tob->tab[0].x=1;
	tob->tab[0].y=1;
        tob->tab[0].surface=width*height;
        tob->tab[0].accsurface=width*height;
	free(it);
	free(queue);
	return 1;
    }

    it[width*height]=it[0];

    for (h=0,ptr=0,ptr2=0;h<(1<<(source->depth&CAM_DEPTH_MASK));h++) {
	// Geodesic SKIZ of level h-1 into level h
	ptr2=ptr;
	// For all pixels p with I[p]=h
	// All these pixels are directly adressable thanks to the initial pixels sort
	while (src[p=it[ptr2]]==h) {
	    ptr2++;

	    // If there is a pixel p' in the neighborhood with dst[p']>0
	    for (flag=0,i=0;i<NEIGHBORHOOD;i++) {
		pp=p+neighbor[i];
		if (dst[pp]>0) {
		    flag=1;
		    break;
		}
	    }
	    if (flag) {
		dst[p]=INQUEUE;
		FIFO_ADD(p);
	    } else {
		dst[p]=MASK;
	    }
	}

	while (!FIFO_EMPTY()) {
	    p=FIFO_FIRST();
	    // For all pixels p' belonging to p's neighborhood
	    for (flag=0,i=0;i<NEIGHBORHOOD;i++) {
		pp=p+neighbor[i];
		if (dst[pp]>0) {
		    // p' belongs to an already labelled basin
		    if (dst[p]==INQUEUE) {
			dst[p]=dst[pp];
                        tob->tab[dst[p]-1].surface++;
		    } else if ((dst[p]>0)&&(dst[p]!=dst[pp])) {
			basin1=basin1p=dst[p]-1;
			basin2=basin2p=dst[pp]-1;
			// It is necessary to update to dynamics of basins
			// We go up the flooding list to retrieve the two
			// big basins that flood
			while (tob->tab[basin1].dynamics!=CAM_NOT_COMPUTED) {
			    basin1=tob->tab[basin1].flooded;
			}
			while (tob->tab[basin2].dynamics!=CAM_NOT_COMPUTED) {
			    basin2=tob->tab[basin2].flooded;
			}
			if (basin1!=basin2) {
			    // It is a saddle point : we compare the minima
			    if (tob->tab[basin1].minimum<tob->tab[basin2].minimum) {
				// In that case, basin1 is flooding basin2
				tob->tab[basin2].flooded=basin1;
				// And we can compute the dynamics of basin2
				tob->tab[basin2].dynamics=src[p]-tob->tab[basin2].minimum;
				// Point in the watershed between p and p'
				dst[p]=-(basin2p+1);
                                tob->tab[basin1p].surface--;
                                tob->tab[basin2p].surface++;
			    } else {
				// In that case, basin2 is flooding basin1
				tob->tab[basin1].flooded=basin2;
				// And we can compute the dynamics of basin1
				tob->tab[basin1].dynamics=src[p]-tob->tab[basin1].minimum;
				// Point in the watershed between p and p'
				dst[p]=-(basin1p+1);
			    }
			} else {
			    // Which one has the lower dynamic ?
			    if (tob->tab[basin2p].dynamics<tob->tab[basin1p].dynamics) {
				// Point in the watershed between p and p'
				dst[p]=-(basin2p+1);
                                tob->tab[basin1p].surface--;
                                tob->tab[basin2p].surface++;  
			    } else {
				// Point in the watershed between p and p'
				dst[p]=-(basin1p+1);
			    }
			}
			flag=1;
			break;
		    }
		}
	    }
	    if (!flag) {
		for (i=0;i<NEIGHBORHOOD;i++) {
		    pp=p+neighbor[i];
		    if (dst[pp]==MASK) {
			dst[pp]=INQUEUE;
			FIFO_ADD(pp);
		    }
		}
	    }
	}

	// Some new minima have appeared ?
	// For any pixel p with src[p]=h
	while (src[p=it[ptr]]==h) {
	    ptr++;

	    if (dst[p]==MASK) {
		label++;
		FIFO_ADD(p);
		dst[p]=label;
		// Append this new basin to the table of basins
		camAddBasin(tob);
		tob->tab[label-1].dynamics=CAM_NOT_COMPUTED;
		tob->tab[label-1].minimum=h;
		tob->tab[label-1].flooded=0;
		tob->tab[label-1].x=(p%source->widthStep)+left;
		tob->tab[label-1].y=(p/source->widthStep)+top;
                tob->tab[label-1].surface=1;
                tob->tab[label-1].accsurface=0;

		while (!FIFO_EMPTY()) {
		    pp=FIFO_FIRST();
		    // For all pixel p'' in the neighborhood of p'
		    for (i=0;i<NEIGHBORHOOD;i++) {
			ppp=pp+neighbor[i];
			if (dst[ppp]==MASK) {
			    FIFO_ADD(ppp);
			    dst[ppp]=label;
                            tob->tab[label-1].surface++;
			}
		    }
		}
	    }
	}			
    }

    // Memory free
    free(it);
    free(queue);

    // Accumulate surface to the top of the tree
    for (i=0;i<tob->nbBasins;i++) {
        tob->tab[i].accsurface+=tob->tab[i].surface;
	b=i;
	while (tob->tab[b].dynamics!=CAM_NOT_COMPUTED) {
            tob->tab[tob->tab[b].flooded].accsurface+=tob->tab[i].surface;
	    b=tob->tab[b].flooded;
        }
    }

    return 1;
}
#endif // CAM_INLINED


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

/* RLE Labelling Kernel
 * C code */

#include <stdio.h>
#include "camellia.h"
#include "camellia_internals.h"

#define CAM_ENCODE1U_OPTIMIZE

#define parent blob

#define YUV422_CLASSIFY() \
    for (j=0;j<nbClusters;j++) { \
        for (i=0;i<3;i++) { \
            val=*ptr[i]; \
            if ((val<limits[(j*3+i)*2])||(val>limits[(j*3+i)*2+1])) break; \
        } \
        if (i==3) break; \
    } \
    if (j==nbClusters) j=0; else j++;
                    
int camRLEEncodeColorYUV422(CamImage *source, CamRLEImage *dest, CamTable *clusters)
{
    int x,y,i,j,nbRuns,l;
    int width,height;
    int nbClusters;
    CAM_PIXEL *srcptr[3],*ptr[3],m;
    CamRun *newRun;
    int val;
    int *limits=clusters->t;

    nbClusters=clusters->size/6;

    // ROI (Region Of Interest) management
    if (source->roi) {
	srcptr[0]=(CAM_PIXEL*)(source->imageData+ source->roi->yOffset*source->widthStep + source->roi->xOffset*sizeof(CAM_PIXEL));
	srcptr[1]=(CAM_PIXEL*)(source->imageData + source->widthStep*source->height + (source->roi->yOffset*(source->widthStep>>1) + (source->roi->xOffset>>1))*sizeof(CAM_PIXEL));
	srcptr[2]=srcptr[1]+((source->widthStep*source->height)>>1);
	width=source->roi->width;
	height=source->roi->height;
    } else {
	srcptr[0]=(CAM_PIXEL*)source->imageData;
	srcptr[1]=srcptr[0]+source->widthStep*source->height;
	srcptr[2]=srcptr[1]+((source->widthStep*source->height)>>1);
	width=source->width;
	height=source->height;
    }

    dest->height=height;
    dest->width=width;

    // Automatic allocation
    CAM_CHECK_ARGS(camRLEEncodeColor,(dest->nSize==sizeof(CamRLEImage)));
    if (dest->allocated==0) camRLEAllocate(dest,height*width+2);

    // Put a run at the begin to have a reference starting point
    newRun=&dest->runs[0];
    newRun->value=-1;
    newRun->length=0;
    newRun->parent=-1;
    newRun->line=-1;
    nbRuns=1;
	    
    // Encode the whole ROI
    for (y=0;y<height;y++) {
	CAM_PIXEL n;
	x=0;
	for (i=0;i<3;i++) ptr[i]=srcptr[i];
        YUV422_CLASSIFY();
        m=n=j;
	do {
	    l=x;
	    x++;
            ptr[0]++;
	    ptr[1]+=(x&1);
	    ptr[2]+=(x&1);
	   
            while (x<width) {
                YUV422_CLASSIFY();
                n=j;
                if (n==m) {
                    x++;
		    ptr[0]++;
		    ptr[1]+=(x&1);
		    ptr[2]+=(x&1);
		} else break;
	    }
              
	    newRun=&dest->runs[nbRuns];
	    newRun->value=m;
	    newRun->length=x-l;
	    newRun->parent=nbRuns;
	    newRun->line=y;
	    nbRuns++;
	    
	    m=n;
	} while (x<width);	
	if (nbRuns>=dest->allocated-width) {
	    dest->nbRuns=0;
	    camRLEReallocate(dest,dest->allocated*2);
            // camSetErrorStr("number or runs is too high");
	    return 0;
	}
	srcptr[0]=(CAM_PIXEL*)(((char*)srcptr[0])+source->widthStep);
	srcptr[1]=(CAM_PIXEL*)(((char*)srcptr[1])+(source->widthStep>>1));
	srcptr[2]=(CAM_PIXEL*)(((char*)srcptr[2])+(source->widthStep>>1));
    }

    // Add one last run, just to ease blob analysis/reconstruction
    newRun=&dest->runs[nbRuns];
    newRun->value=-1;
    newRun->length=0;
    newRun->parent=-1;
    newRun->line=-1;
    if (nbRuns+1>=dest->allocated-width) {
	dest->nbRuns=0;
        camRLEReallocate(dest,dest->allocated*2);
   	// camSetErrorStr("number or runs is too high");
	return 0;
    }

    dest->nbRuns=nbRuns;
    return 1;    
}

#define CLASSIFY() \
    for (j=0;j<nbClusters;j++) { \
        for (i=0;i<nChannels;i++) { \
            val=ptr[i*jump]; \
            if ((ptr[i*jump]<limits[(j*nChannels+i)*2])||(ptr[i*jump]>limits[(j*nChannels+i)*2+1])) break; \
        } \
        if (i==nChannels) break; \
    } \
    if (j==nbClusters) j=0; else j++;
 
int camRLEEncodeColor(CamImage *source, CamRLEImage *dest, CamTable *clusters)
{
    int x,y,i,j,nbRuns,l;
    int width,height;
    int nChannels, nbClusters;
    CAM_PIXEL *srcptr,*ptr,m;
    CamRun *newRun;
    int jump,incr;
    int val;
    int *limits=clusters->t;

    CAM_CHECK_ARGS(camRLEEncodeColor,source->imageData!=NULL);
    CAM_CHECK_ARGS(camRLEEncodeColor,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camRLEEncodeColor,(source->depth&CAM_DEPTH_MASK)>=8);

    nChannels=source->nChannels;
    nbClusters=clusters->size/(nChannels*2);

    // ROI (Region Of Interest) management
    if (source->dataOrder==CAM_DATA_ORDER_PLANE) {
        if (*((int*)source->channelSeq)==*((int*)"422")) return camRLEEncodeColorYUV422(source, dest, clusters); 
        jump=source->imageSize/nChannels;
        incr=1;
    } else {
        jump=1;
        incr=nChannels;
    }
    if (source->roi) {
	srcptr=(CAM_PIXEL*)(source->imageData+source->roi->yOffset*source->widthStep+source->roi->xOffset*sizeof(CAM_PIXEL)*incr);
	width=source->roi->width;
	height=source->roi->height;
    } else {
	srcptr=(CAM_PIXEL*)source->imageData;
	width=source->width;
	height=source->height;
    }

    dest->height=height;
    dest->width=width;

    // Automatic allocation
    CAM_CHECK_ARGS(camRLEEncodeColor,(dest->nSize==sizeof(CamRLEImage)));
    if (dest->allocated==0) camRLEAllocate(dest,height*width+2);

    // Put a run at the begin to have a reference starting point
    newRun=&dest->runs[0];
    newRun->value=-1;
    newRun->length=0;
    newRun->parent=-1;
    newRun->line=-1;
    nbRuns=1;
	    
    // Encode the whole ROI
    for (y=0;y<height;y++) {
	CAM_PIXEL n;
	x=0;
	ptr=srcptr;
        CLASSIFY();
        m=n=j;
	do {
	    l=x;
	    x++;
            ptr+=incr;
	   
            while (x<width) {
                CLASSIFY();
                n=j;
                if (n==m) {
                    ptr+=incr;
                    x++;
                } else break;
            }
              
	    newRun=&dest->runs[nbRuns];
	    newRun->value=m;
	    newRun->length=x-l;
	    newRun->parent=nbRuns;
	    newRun->line=y;
	    nbRuns++;
	    
	    m=n;
	} while (x<width);	
	if (nbRuns>=dest->allocated-width) {
	    dest->nbRuns=0;
	    camRLEReallocate(dest,dest->allocated*2);
            // camSetErrorStr("number or runs is too high");
	    return 0;
	}
	srcptr=(CAM_PIXEL*)(((char*)srcptr)+source->widthStep);
    }

    // Add one last run, just to ease blob analysis/reconstruction
    newRun=&dest->runs[nbRuns];
    newRun->value=-1;
    newRun->length=0;
    newRun->parent=-1;
    newRun->line=-1;
    if (nbRuns+1>=dest->allocated-width) {
	dest->nbRuns=0;
        camRLEReallocate(dest,dest->allocated*2);
   	// camSetErrorStr("number or runs is too high");
	return 0;
    }

    dest->nbRuns=nbRuns;
    return 1;    
}

#ifndef CAM_ENCODE1U_OPTIMIZE

int camRLEEncode1U(CamImage *source, CamRLEImage *dest)
{
    int x,y,nbRuns,l,m,n;
    int width,height;
    int sol_offset,bit_offset;
    unsigned char *srcptr,*cpsrcptr,valchar;
    CamRun *newRun;

    // ROI (Region Of Interest) management
    if (source->roi) {
	sol_offset=source->roi->xOffset%8;
	srcptr=(unsigned char*)(source->imageData+source->roi->yOffset*source->widthStep+source->roi->xOffset/8);
	width=source->roi->width;
	height=source->roi->height;
    } else {
	srcptr=(unsigned char*)source->imageData;
	sol_offset=0;
	width=source->width;
	height=source->height;
    }

    dest->height=height;
    dest->width=width;

    // Automatic allocation
    CAM_CHECK_ARGS(camRLEEncode1U,(dest->nSize==sizeof(CamRLEImage)));
    if (dest->allocated==0) camRLEAllocate(dest,height*width+2);

    // Put a run at the begin to have a reference starting point
    newRun=&dest->runs[0];
    newRun->value=-1;
    newRun->length=0;
    newRun->parent=-1;
    newRun->line=-1;
    nbRuns=1;
	    
    // Encode the whole ROI
    // Bit by bit processing
    for (y=0;y<height;y++) {
	cpsrcptr=srcptr;
	bit_offset=sol_offset;
	valchar=*srcptr;
	valchar<<=bit_offset;
	x=0;
	m=valchar & 0x80;
	do {
	    l=x;	    
	    do {
		x++;
		if (x<width) {
		    if (bit_offset==7) {
			bit_offset=0;
			valchar=*(++srcptr);
		    } else {
			bit_offset++;
			valchar<<=1;
		    }
		    n=valchar & 0x80;
		} else break;
	    } while (n==m);
	    
	    newRun=&dest->runs[nbRuns];
	    newRun->value=m>>7;
	    newRun->length=x-l;
	    newRun->parent=nbRuns;
	    newRun->line=y;
	    nbRuns++;
	    
	    m=n;
	} while (x<width);	
	if (nbRuns>=dest->allocated-width) {
	    dest->nbRuns=0;
            camRLEReallocate(dest,dest->allocated*2);
            // camSetErrorStr("number or runs is too high");
	    return 0;
	}
	srcptr=cpsrcptr+source->widthStep;
    }

    // Add one last run, just to ease blob analysis/reconstruction
    newRun=&dest->runs[nbRuns];
    newRun->value=-1;
    newRun->length=0;
    newRun->parent=-1;
    newRun->line=-1;
    if (nbRuns+1>=dest->allocated-width) {
	dest->nbRuns=0;
        camRLEReallocate(dest,dest->allocated*2);
        // camSetErrorStr("number or runs is too high");
	return 0;
    }

    dest->nbRuns=nbRuns;
    return 1;        
}

#else

int camRLEEncode1U(CamImage *source, CamRLEImage *dest)
{
    int x,y,z,nbRuns,l,m,n,i,numbytes,j,valchar;
    int width,height;
    int sol_offset,bit_offset;
    unsigned char *srcptr,*cpsrcptr;
    CamRun *newRun;

    // Insert automatically generated code
#include "cam_RLE_labelling_table1.c"
#include "cam_RLE_labelling_table2.c"

    // ROI (Region Of Interest) management
    if (source->roi) {
	sol_offset=source->roi->xOffset%8;
	srcptr=(unsigned char*)(source->imageData+source->roi->yOffset*source->widthStep+source->roi->xOffset/8);
	width=source->roi->width;
	height=source->roi->height;
	// The first byte (maybe incomplete) and the last byte if incomplete
	// are encoded bit-by-bit
	numbytes=(width-(8-(source->roi->xOffset%8)))/8;
    } else {
	srcptr=(unsigned char*)source->imageData;
	sol_offset=0;
	width=source->width;
	height=source->height;
	// Compute the number of bytes for byte-by-byte encoding
	numbytes=width/8-1; // The first byte is encoded bit-by-bit (could be optimized)
    }

    dest->height=height;
    dest->width=width;
    
    // Automatic allocation
    CAM_CHECK_ARGS(camRLEEncode1U,(dest->nSize==sizeof(CamRLEImage)));
    if (dest->allocated==0) camRLEAllocate(dest,height*width+2);

    // Put a run at the beginning to have a reference starting point
    newRun=&dest->runs[0];
    newRun->value=-1;
    newRun->length=0;
    newRun->parent=-1;
    newRun->line=-1;
    nbRuns=1;
	    
    // Encode the whole ROI
    // We start each line with a bit by bit processing,
    // until we get aligned with a byte boundary
    for (y=0;y<height;y++) {
	cpsrcptr=srcptr;
	bit_offset=sol_offset;
	valchar=*srcptr;
	valchar<<=bit_offset;
	x=0;
	m=valchar & 0x80;
	do {
	    l=x;	    
	    do {
		x++;
		if (x<width) {
		    if (bit_offset==7) {
			// We have reached a byte boundary
			// Let's start the byte-by-byte RLE encoding process
			m>>=7;
			for (i=0;i<numbytes;i++) {
			    valchar=*(++srcptr);
			    // n is the initial value for the set of runs
			    // m is the current value for the run
			    n=m;
			    z=x;
			    for (j=index[n][valchar];j<index[n][valchar+1];j++) {
				z+=table[n][j];
				newRun=&dest->runs[nbRuns];
				newRun->value=m;
				newRun->length=z-l;
				newRun->parent=nbRuns;
				newRun->line=y;
				nbRuns++;
				m=1-m; // m=!m;
				l=z;
			    }
			    x+=8;
			}
			m<<=7;
			bit_offset=0;
			if (x<width) {
			    valchar=*(++srcptr);
			    n=valchar & 0x80;
			} else n=!m;
		    } else {
			bit_offset++;
			valchar<<=1;
			n=valchar & 0x80;
		    }
		} else break;
	    } while (n==m);
	    
	    // Write the last run of the line if need be
	    if (x!=l) {
		newRun=&dest->runs[nbRuns];
		newRun->value=m>>7;
		newRun->length=x-l;
		newRun->parent=nbRuns;
		newRun->line=y;
		nbRuns++;
	    }
	    
	    m=n;
	} while (x<width);	
	if (nbRuns>=dest->allocated-width) {
	    dest->nbRuns=0;
            camRLEReallocate(dest,dest->allocated*2);
            // camSetErrorStr("number or runs is too high");
	    return 0;
	}
	srcptr=cpsrcptr+source->widthStep;
    }

    // Add one last run, just to ease blob analysis/reconstruction
    newRun=&dest->runs[nbRuns];
    newRun->value=-1;
    newRun->length=0;
    newRun->parent=-1;
    newRun->line=-1;
    if (nbRuns+1>=dest->allocated-width) {
	dest->nbRuns=0;
        camRLEReallocate(dest,dest->allocated*2);
        // camSetErrorStr("number or runs is too high");
	return 0;
    }

    dest->nbRuns=nbRuns;
    return 1;        
}

void camRLEEncodeTableGenerator()
{
    int i,c,val,fval,run,index,bitval,n;
    FILE *f1,*f2;
    f1=fopen("cam_RLE_labelling_table1.c","wt");
    f2=fopen("cam_RLE_labelling_table2.c","wt");
    fprintf(f1,"const int index[2][257]={\n   {");
    fprintf(f2,"const int table[2][1024]={\n  {");
    for (fval=0;fval<2;fval++) {
	index=0;
	n=0;
	for (c=0;c<256;c++) {
	    fprintf(f1,"%d,",index);
	    val=fval;
	    run=0;
	    for (i=0;i<8;i++) {
		bitval=((c<<i)&0xff)>>7;
		if (bitval==val) {
		    run++;
		} else {
		    val=bitval;
		    fprintf(f2,"%s%d",((n)?((n%16)?",":",\n   "):""),run);
		    n++;
		    run=1;
		    index++;
		}
	    }
	}
	if (fval==0) {
	    fprintf(f1,"%d},\n   {",index);
	    fprintf(f2,"},\n  {");
	} else {
	    fprintf(f1,"%d}\n};\n",index);
	    fprintf(f2,"}\n};\n");
	}
    }
    fclose(f1);
    fclose(f2);
}
#endif

// Connect components using four-connecteness so that the runs each
// identify the global parent of the connected region they are a part of.
// It does this by scanning adjacent rows and merging where similar
// colors overlap.
int camRLELabelling(CamRLEImage *src, CamBlobAnalysisResults *results)
{
    int x1,x2;
    int l1,l2;
    CamRun r1,r2;
    int i,p,s,n;
    int num=src->nbRuns;
    int width=src->width;
    CamRun *run=src->runs;
    
    if (src->nbRuns == 0) {
	results->nbBlobs = 0;	
	return 0;
    }

    l1 = l2 = 1;
    x1 = x2 = 0;
    
    // Lower scan begins on second line, so skip over first
    while (x1 < width) {
	x1 += run[l1++].length;
    }
    x1 = 0;
    
    // Do the remaining lines in lock step
    r1 = run[l1];
    r2 = run[l2];
    s = l1;
    while (l1 < num) {
	if (r1.value==r2.value && r1.value) { 
	    if ((x1>=x2 && x1<x2+r2.length) || (x2>=x1 && x2<x1+r1.length)) {
		if(s != l1) {
		    run[l1].parent = r1.parent = r2.parent;
		    s = l1;
		} else {
		    // Find terminal roots of each path
		    n = r1.parent;
		    while (n != run[n].parent) n = run[n].parent;
		    p = r2.parent;
		    while (p != run[p].parent) p = run[p].parent;
		    
		    // Must use smaller of two to preserve DAGness!
		    if(n < p) {
			run[p].parent = n;
		    } else {
			run[n].parent = p;
		    }
		}
	    }
	}
	
	// Move to next point where values may change
	if (x1+r1.length < x2+r2.length) {
	    x1 += r1.length;
	    r1 = run[++l1];
	} else {
	    x2 += r2.length;
	    r2 = run[++l2];
	}
    }
    
    // Now we need to compress all parent paths
    // This is the traditional second scan
    for (i=1; i<num; i++) {
	p = run[i].parent;
	if (p > i) {
	    while (p != run[p].parent) p = run[p].parent;
	    run[i].parent = p;
	} else {
	    run[i].parent = run[p].parent;
	}
    }
    
    // Now, let's run the blob analysis
    return camRLEBlobAnalysis(src,results);
}

// Takes the list of runs and formats them into a region table,
// gathering the various statistics we want along the way.
// Implemented as a single pass over the array of runs.
int camRLEBlobAnalysis(CamRLEImage *src, CamBlobAnalysisResults *results)
{
    int x,y,i;
    int b,n,a;
    CamRun *r;
    int nbRuns=src->nbRuns;
    int width=src->width;

    // Sum of integers over range [x,x+w)
    #define RANGE_SUM(x,w) (w*(2*x + w-1) / 2)
    #define MIN(x,y) (((x)<(y))?(x):(y))
    #define MAX(x,y) (((x)>(y))?(x):(y))

    results->nbBlobs=0; // Just in case it would fail...
    
    x = y = n = 0;
    for (i=1; i<nbRuns; i++) {
	r = &src->runs[i];
	
	if (r->value) {
	    if (r->parent == i) {
		// Add new region if this run is a root (i.e. self parented)
		src->runs[i].parent = b = n;  // Renumber to point to region id
		results->blobInfo[b].id = n;
		results->blobInfo[b].surface = r->length;
		results->blobInfo[b].left = x;
		results->blobInfo[b].top = y;
		results->blobInfo[b].width = r->length;
		results->blobInfo[b].height = 1;
		results->blobInfo[b].cx = RANGE_SUM(x, r->length);
		results->blobInfo[b].cy = y * r->length;
		results->blobInfo[b].value = r->value;
		results->blobInfo[b].first = r;
		results->blobInfo[b].last = r;
		n++;
		if(n >= CAM_LABEL_MAX_BLOBS) return 0;
	    } else {
		// Otherwise update region stats incrementally
		b = src->runs[r->parent].parent;
		src->runs[i].parent = b; // Update to point to region id
		results->blobInfo[b].surface += r->length;
		if (x < results->blobInfo[b].left) {
		    results->blobInfo[b].width += results->blobInfo[b].left - x;
		    results->blobInfo[b].left = x;
		}   
		results->blobInfo[b].width = MAX(x + r->length - results->blobInfo[b].left, results->blobInfo[b].width);
		results->blobInfo[b].height = y - results->blobInfo[b].top+1; // Last set by lowest run
		results->blobInfo[b].cx += RANGE_SUM(x, r->length);
		results->blobInfo[b].cy += y * r->length;
		results->blobInfo[b].last = r;
	    }
	} else r->parent=-1;
	
	// Step to next location
	x = (x + r->length) % width;
	y += (x == 0);
    }
    
    // Calculate centroids from stored temporaries
    for (i=0; i<n; i++) {
	a = results->blobInfo[i].surface;
	results->blobInfo[i].cx = results->blobInfo[i].cx / a;
	results->blobInfo[i].cy = results->blobInfo[i].cy / a;
    }
    
    results->nbBlobs=n;
    return 1;
}

#undef CAM_PIXEL
#define CAM_PIXEL unsigned char
#define camRLEEncode camRLEEncode8
#define MAP(x) x
#undef PARAM
#include "cam_RLE_labelling_code.c"
#undef camRLEEncode
#undef MAP
#define camRLEEncode camRLEEncodeLUT8
#define MAP(x) LUT->t[x]
#define PARAM CamLUT *LUT
#include "cam_RLE_labelling_code.c"
#undef camRLEEncode
#undef MAP
#undef PARAM
#define camRLEEncode camRLEEncodeThreshold8
#define MAP(x) x>=threshold
#define PARAM int threshold
#include "cam_RLE_labelling_code.c"
#undef camRLEEncode
#undef MAP
#undef PARAM
#define camRLEEncode camRLEEncodeThresholdInv8
#define MAP(x) x<threshold
#define PARAM int threshold
#include "cam_RLE_labelling_code.c"
#undef camRLEEncode
#undef MAP
#undef PARAM

#undef CAM_PIXEL
#define CAM_PIXEL unsigned short
#define camRLEEncode camRLEEncode16
#define MAP(x) x
#undef PARAM
#include "cam_RLE_labelling_code.c"
#undef camRLEEncode
#undef MAP
#define camRLEEncode camRLEEncodeLUT16
#define MAP(x) LUT->t[x]
#define PARAM CamLUT *LUT
#include "cam_RLE_labelling_code.c"
#undef camRLEEncode
#undef MAP
#undef PARAM
#define camRLEEncode camRLEEncodeThreshold16
#define MAP(x) x>=threshold
#define PARAM int threshold
#include "cam_RLE_labelling_code.c"
#undef camRLEEncode
#undef MAP
#undef PARAM
#define camRLEEncode camRLEEncodeThresholdInv16
#define MAP(x) x<threshold
#define PARAM int threshold
#include "cam_RLE_labelling_code.c"
#undef camRLEEncode
#undef MAP
#undef PARAM

int camRLEEncode(CamImage *source, CamRLEImage *dest)
{
    // Binary images processing 
    if (source->depth==CAM_DEPTH_1U) {
	return camRLEEncode1U(source,dest);
    } else if ((source->depth&CAM_DEPTH_MASK)==8) {
        return camRLEEncode8(source,dest);
    } else if ((source->depth&CAM_DEPTH_MASK)<=16) {
        return camRLEEncode16(source,dest);
    }
    return 0;	   
}

int camRLEEncodeLUT(CamImage *source, CamRLEImage *dest, CamLUT *LUT)
{
    if ((source->depth&CAM_DEPTH_MASK)==8) {
        return camRLEEncodeLUT8(source,dest,LUT);
    } else if ((source->depth&CAM_DEPTH_MASK)<=16) {
        return camRLEEncodeLUT16(source,dest,LUT);
    }
    return 0;	   
}

int camRLEEncodeThreshold(CamImage *source, CamRLEImage *dest, int threshold)
{
    if ((source->depth&CAM_DEPTH_MASK)==8) {
        return camRLEEncodeThreshold8(source,dest,threshold);
    } else if ((source->depth&CAM_DEPTH_MASK)<=16) {
        return camRLEEncodeThreshold16(source,dest,threshold);
    }
    return 0;	   
}

int camRLEEncodeThresholdInv(CamImage *source, CamRLEImage *dest, int threshold)
{
    if ((source->depth&CAM_DEPTH_MASK)==8) {
        return camRLEEncodeThresholdInv8(source,dest,threshold);
    } else if ((source->depth&CAM_DEPTH_MASK)<=16) {
        return camRLEEncodeThresholdInv16(source,dest,threshold);
    }
    return 0;	   
}

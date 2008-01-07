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
#include <string.h>
#include <math.h>
#include "camellia.h"
#include "camellia_internals.h"

/* PBM load/save utilities
 */

/* Get the next non-comment line from the PBM file f into the buffer b.
 */
static void pbm_getln(FILE *f, char *b)
{
    int i;
    char c;
    
    // Read the next significant line (non-comment) from f into buffer b
    do
    {
	// Read the next line
	i = 0;
	do
	{
	    fscanf (f, "%c", &c);
	    b[i++] = c;
	    if (c == '\n') b[i] = '\0';
	} while (c != '\n');
    } while (b[0]=='\n' || b[0] == '#');
}

static void get_num_pbm(FILE *f, char *b, int *bi, int *res)
{
    int i;
    char str[80];
    
    while (b[*bi]==' ' || b[*bi]=='\t' || b[*bi]=='\n' || b[*bi]=='\r')
    {
	if (b[*bi] == '\n' || b[*bi]=='\r') 
	{
	    pbm_getln (f, b);
	    *bi = 0;
	} else
	    *bi += 1;
    }
    
    i = 0;
    while (b[*bi]>='0' && b[*bi]<='9')
	str[i++] = b[(*bi)++];
    str[i] = '\0';
    sscanf (str, "%d", res);
}

int camLoadPGM(CamImage *im, char *fn)
{
    int i,j,k,n,m,bi, b;
    unsigned char ucval;
    int val;
    long here;
    char buf1[256];
    FILE *f;
    
    strcpy(buf1,fn);
    f=fopen(buf1,"rb");
    if (f==NULL) {
        camError("camLoadPGM","Can't open specified PGM file");
	return 0;
    }
    
    pbm_getln(f,buf1);
    if (buf1[0]=='P') {
	switch (buf1[1])
	{
	case '1':       k=1; break;
	case '2':       k=2; break;
	case '3':       k=3; break;
	case '4':       k=4; break;
	case '5':       k=5; break;
	case '6':       k=6; break;
	default:        k=0; camError("camLoadPGM","Not a PBM/PGM/PPM file."); return 0;
	}
    } else k=0;
    bi = 2;
    
    get_num_pbm(f,buf1,&bi,&m);         // Number of columns
    get_num_pbm(f,buf1,&bi,&n);         // Number of rows
    if (k!=1 && k!=4) get_num_pbm(f,buf1,&bi,&b); // Max value
    else b=1;
    
    // Binary file? Re-open as 'rb'
    if (k>3) {
	here = ftell(f);
	fclose(f);
	f = fopen(fn,"rb");       
	here--;
        if (fseek(f,here,0) != 0) {
	    camError("camLoadPGM","Unable to load binary PGM files");
	    return 0;
	}
    }
    
    // Allocate the image
    if (k==3 || k==6) {       // Colour
	camError("camLoadPGM","Can't load color images");
	return 0;
    } else  {
        if (b>255) {
            camAllocateImage(im, m, n, CAM_DEPTH_16U);
            for (i=0; i<n; i++) {
                for (j=0; j<m; j++) {
                    if (k<3) {
                        fscanf(f, "%d", &val);
                        *((unsigned short*)(im->imageData+i*im->widthStep)+j) = (unsigned short)val;
                    } else {
                        fscanf(f, "%c", &ucval);
                        *((unsigned short*)(im->imageData+i*im->widthStep)+j) = (unsigned short)ucval;
                    }
                }
            }
        } else {
            camAllocateImage(im, m, n, CAM_DEPTH_8U);
            for (i=0; i<n; i++) {
                for (j=0; j<m; j++) {
                    if (k<3) {
                        fscanf(f, "%d", &val);
                        *((unsigned char*)im->imageData+j+i*im->widthStep) = (unsigned char)val;
                    } else {
                        fscanf(f, "%c", &ucval);
                        *((unsigned char*)im->imageData+j+i*im->widthStep) = ucval;
                    }
                }
            }
        }
    }
    fclose(f);
    return 1;
}

int camSavePGM(CamImage *image, char *filename)
{
    FILE *f;
    int i,j,k,perline,pixel;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camSavePGM,camInternalROIPolicy(image, NULL, &iROI, 0));
    CAM_CHECK_ARGS(camSavePGM,iROI.nChannels==1);

    if (iROI.srcroi.width > 64) perline = 64;
    else perline = iROI.srcroi.width-1;
    f = fopen(filename, "w");
    if (f == 0) {
	camError("camSavePGM","Can't open output file");
	return 0;
    }
    
    fprintf(f,"P2\n"); 
    if (image->depth==CAM_DEPTH_1U) {
	fprintf(f, "%d %d 255\n", iROI.srcroi.width, iROI.srcroi.height);
    } else if ((image->depth&CAM_DEPTH_MASK)==32) {
        fprintf(f, "%d %d 4294967295\n", iROI.srcroi.width, iROI.srcroi.height);
    } else {
        fprintf(f, "%d %d %d\n", iROI.srcroi.width, iROI.srcroi.height, (int)((1<<(image->depth&CAM_DEPTH_MASK))-1));
    }
    k = 0;
    if ((image->depth&CAM_DEPTH_MASK)==1) {
        CAM_CHECK_ARGS(camSavePGM,image->roi==NULL);
	for (i=0; i<iROI.srcroi.height; i++) {
	    for (j=0; j<iROI.srcroi.width; j++) {
                pixel=(((*((unsigned char*)image->imageData+(j>>3)+i*image->widthStep))<<(j&0x07))&0x80)?255:0;
		fprintf(f, "%d ", pixel);
		k++;
		if (k > perline) {
		    fprintf(f, "\n");
		    k = 0;
		}
	    }
	}
    } else if ((image->depth&CAM_DEPTH_MASK)==8) {
	for (i=0; i<iROI.srcroi.height; i++) {
	    for (j=0; j<iROI.srcroi.width; j++) {
		pixel=(int)*((unsigned char*)image->imageData+iROI.srcchoffset+(iROI.srcroi.xOffset+j)*iROI.srcinc+(iROI.srcroi.yOffset+i)*image->widthStep);
		fprintf(f, "%d ", pixel);
		k++;
		if (k > perline) {
		    fprintf(f, "\n");
		    k = 0;
		}
	    }
	}
    } else if ((image->depth&CAM_DEPTH_MASK)<=16) {
	for (i=0; i<iROI.srcroi.height; i++) {
	    for (j=0; j<iROI.srcroi.width; j++) {
		pixel=(int)*((unsigned short*)(image->imageData+iROI.srcchoffset+(iROI.srcroi.yOffset+i)*image->widthStep)+(iROI.srcroi.xOffset+j)*iROI.srcinc);
		fprintf(f, "%d ", pixel);
		k++;
		if (k > perline) {
		    fprintf(f, "\n");
		    k = 0;
		}
	    }
	}
    } else if ((image->depth&CAM_DEPTH_MASK)==32) {
	for (i=0; i<iROI.srcroi.height; i++) {
	    for (j=0; j<iROI.srcroi.width; j++) {
		pixel=(int)*((unsigned long*)(image->imageData+iROI.srcchoffset+(iROI.srcroi.yOffset+i)*image->widthStep)+(iROI.srcroi.xOffset+j)*iROI.srcinc);
		fprintf(f, "%d ", pixel);
		k++;
		if (k > perline) {
		    fprintf(f, "\n");
		    k = 0;
		}
	    }
	}
    } else {
	fclose(f);
        camError("camSavePGM","Unsupported image format");
	return 0;
    }
    fprintf(f, "\n");
    fclose(f);

    return 1;
}

int camSaveRawPGM(CamImage *image, char *filename)
{
    FILE *f;
    int i,j;
    unsigned char pixel;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camSaveRawPGM, camInternalROIPolicy(image, NULL, &iROI, 0));
    CAM_CHECK_ARGS(camSaveRawPGM, iROI.nChannels==1);
    CAM_CHECK_ARGS(camSaveRawPGM, (image->depth & CAM_DEPTH_MASK) == 8);

    f = fopen(filename, "w");
    if (f == 0) {
	camError("camSaveRawPGM","Can't open output file");
	return 0;
    }
    
    fprintf(f, "P5\n%d %d\n255\n", iROI.srcroi.width, iROI.srcroi.height);
    for (i = 0; i < iROI.srcroi.height; i++) {
	for (j = 0; j < iROI.srcroi.width; j++) {
	    pixel = *((unsigned char*)image->imageData + iROI.srcchoffset + (iROI.srcroi.xOffset + j) * iROI.srcinc + (iROI.srcroi.yOffset + i) * image->widthStep);
	    fputc(pixel, f);
	}
    }
    fclose(f);

    return 1;
}

static void put32(unsigned long value, FILE* outf)
{
    fputc(value & 0xff,outf);
    fputc((value >>  8) & 0xff,outf);
    fputc((value >> 16) & 0xff,outf);
    fputc((value >> 24) & 0xff,outf); 
}

static void put16(unsigned int value, FILE* outf)
{
    fputc(value & 0xff,outf);
    fputc((value >>  8) & 0xff,outf);
}

// Contribution by Winfried Gerhke (Philips Hamburg)
int camSaveBMP(CamImage *image, char *filename)
{
    long  width,height;
    int   h,w,r,g,b;
    FILE *outfile;
    char buf1[64];
      
    CAM_CHECK_ARGS2(camSaveBMP,image->imageData!=NULL,"image is not allocated");
    CAM_CHECK_ARGS(camSaveBMP,image->dataOrder==CAM_DATA_ORDER_PIXEL);
    
    strcpy(buf1, filename);
    outfile = fopen(buf1, "wb");
    if (outfile == NULL) 
    {
	printf("Can't open output file.\n");
	exit(0);
    }
    
    width  = image->width;
    height = image->height;
    
    fputc('B',outfile);
    fputc('M',outfile);
    put32(54+width*height*3, outfile);
    put32(0L, outfile);
    put32(54, outfile);
    
    put32(40L, outfile);
    put32(width, outfile);
    put32(height, outfile);
    put16(1, outfile);
    put16(24, outfile);
    put32(0, outfile);
    put32(width*height*3, outfile);
    put32(10000, outfile);
    put32(10000, outfile);
    put32(0, outfile);
    put32(0, outfile);
    
    if ((image->depth&CAM_DEPTH_MASK)==1) {
	for (h=height-1; h>=0; h--)
	{
	    for (w=0; w<width; w++)
	    {
		r = b = g = (((*((unsigned char*)image->imageData+(w>>3)+h*image->widthStep))<<(w&0x07))&0x80)?255:0;
		
		putc(b,outfile);   /* B */
		putc(g,outfile);   /* G */
		putc(r,outfile);   /* R */
	    }
	    
	    switch ((width*3) % 4)
	    {
	    case 1:  putc(0,outfile);
	    case 2:  putc(0,outfile);
	    case 3:  putc(0,outfile);
	    }
	}
    } else if ((image->depth&CAM_DEPTH_MASK)==8) {
	for (h=height-1; h>=0; h--)
	{
            if (image->nChannels>=3) {
                if ((image->channelSeq[0]=='R')&&(image->channelSeq[1]=='G')&&(image->channelSeq[2]=='B')) {
		    for (w=0; w<width; w++)
                    {
                        r = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep);
                        g = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep+1);
                        b = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep+2);

                        putc(b,outfile);   /* B */
                        putc(g,outfile);   /* G */
                        putc(r,outfile);   /* R */
                    }
                } else if ((image->channelSeq[0]=='G')&&(image->channelSeq[1]=='R')&&(image->channelSeq[2]=='B')) {
		    for (w=0; w<width; w++)
                    {
                        g = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep);
                        r = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep+1);
                        b = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep+2);

                        putc(b,outfile);   /* B */
                        putc(g,outfile);   /* G */
                        putc(r,outfile);   /* R */
                    }
                } else { /* In all other cases, including YUV, records in BGR */
                    for (w=0; w<width; w++)
                    {
                        b = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep);
                        g = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep+1);
                        r = (int)*((unsigned char*)image->imageData+w*image->nChannels+h*image->widthStep+2);

                        putc(b,outfile);   /* B */
                        putc(g,outfile);   /* G */
                        putc(r,outfile);   /* R */
                    }
                }
            } else if (image->nChannels==1) {
                for (w=0; w<width; w++)
                {
                    r = b = g = (int)*((unsigned char*)image->imageData+w+h*image->widthStep);
                    
                    putc(b,outfile);   /* B */
                    putc(g,outfile);   /* G */
                    putc(r,outfile);   /* R */
                }
            }

	    switch ((width*3) % 4)
	    {
	    case 1:  putc(0,outfile);
	    case 2:  putc(0,outfile);
	    case 3:  putc(0,outfile);
	    }
	}
    } else if ((image->depth&CAM_DEPTH_MASK)==8) {
	camError("camSaveBMP","Can't save 16-bit deep BMP images");
    }
    
    fclose (outfile);
    return 1;
}

static unsigned long get32(FILE* inf)
{
    unsigned long c1,c2,c3,c4;
    
    c1 = fgetc(inf);
    c2 = fgetc(inf);
    c3 = fgetc(inf);
    c4 = fgetc(inf);
    return(c1+(c2<<8)+(c3<<16)+(c4<<24));
}

static unsigned long get16(FILE* inf)
{
    unsigned long c1,c2;
    
    c1 = fgetc(inf);
    c2 = fgetc(inf);
    return(c1+(c2<<8));
}

static unsigned long get8(FILE* inf)
{
    
    return(fgetc(inf));
}

int camLoadBMP(CamImage *im, char *fn)
{
    long  width,height,bpp,bitmap_start,comp;
    char buf1[256],str[256];
    int h,w;
    int r,g,b;
    FILE *f;
     
    strcpy(buf1, fn);
    f = fopen(buf1, "rb");
    if (f==NULL)
    {
	sprintf(str,"Can't open the BMP file named '%s'", buf1);
	camError("camLoadBMP",str);
        return 0;
    }
    
    if (get8(f) != 'B')
    {
	camError("camLoadBMP","Input file does not have BMP format");
	return 0;
    }
    
    if (get8(f) != 'M')
    {
	camError("camLoadBMP","Input file does not have BMP format");
	return 0;
    }
    
    get32(f);
    get32(f);
    bitmap_start = get32(f);
    
    get32(f);
    width  = get32(f);
    height = get32(f);
    get16(f);
    bpp = get16(f);
    if (bpp != 24)
    {
	sprintf(str,"Unsupported bpp value: %ld bpp", bpp);
	camError("camLoadBMP",str);
	return 0;
    }
    comp = get32(f);
    if (comp != 0)
    {
	camError("camLoadBMP","Only uncompressed BMP files supported");
	return 0;
    }
    
    fseek(f,bitmap_start,SEEK_SET); // Go to start of bitmap
    
    camAllocateRGBImage(im, width, height);
    
    for (h=height-1; h>=0; h--)
    {
	for (w=0; w<width; w++)
	{
	    b = get8(f);   /* B */
	    g = get8(f);   /* G */
	    r = get8(f);   /* R */
	    
	    *((unsigned char*)im->imageData+w*3+h*im->widthStep)   = r;
	    *((unsigned char*)im->imageData+w*3+h*im->widthStep+1) = g;
	    *((unsigned char*)im->imageData+w*3+h*im->widthStep+2) = b;
	}
	
	switch ((width*3) % 4)
	{
	case 1:  get8(f);
	case 2:  get8(f);
	case 3:  get8(f);
	}
    }
      
    fclose (f);
    return 1;
}

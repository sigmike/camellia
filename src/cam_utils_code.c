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

int camSet(CamImage *image, int fillValue);

int camCopy(CamImage* source, CamImage* dest)
{
    int x,y,i,n,c,special=0;
    int width,height;
    CAM_PIXEL *srcptr,*cpsrcptr;
    CAM_PIXEL_DST *dstptr,*cpdstptr;
    CamROI roi,roi2,*tmp,*tmp2;
#ifdef CAM_SATURATE
    int valpix;
    int valmin=0, valmax=255;
#endif

    CamRun *run;
    int startx,endx;
    CamInternalROIPolicyStruct iROI;

    if (dest->imageData) {
        // Special cases : 
        // Grey to color conversion : source is one channel, and dest is several channels (3 or 4)
        if (((source->nChannels==1)||((source->roi)&&(source->roi->coi)))
            &&(dest->nChannels!=1)&&((dest->roi==NULL)||(dest->roi->coi==0))) special=1;
        // 3 to 4 channels conversion : RGB->RGBA. Adding an alpha channel
        else if ((source->nChannels==3)&&(dest->nChannels==4)&&
            ((source->roi==NULL)||(source->roi->coi==0))&&
            ((dest->roi==NULL)||(dest->roi->coi==0))) special=2;
        // 4 to 3 channels conversion : RGBA -> RGB. Removing an alpha channel
        else if ((source->nChannels==4)&&(dest->nChannels==3)&&
            ((source->roi==NULL)||(source->roi->coi==0))&&
            ((dest->roi==NULL)||(dest->roi->coi==0))) special=2;
    }
    if (special) {
        tmp=dest->roi;
        if (dest->roi) {
            roi=*tmp;
        } else {
            camSetMaxROI(&roi,dest);
        }
        dest->roi=&roi;
        roi.coi=1;
        n=dest->nChannels;
        if (n==4) n=3; // Doesn't copy to the alpha channel
        if (special==2) {
            tmp2=source->roi;
            if (source->roi) {
                roi2=*tmp2;
            } else {
                camSetMaxROI(&roi2,source);
            }
            source->roi=&roi2;
            roi2.coi=1;
        }
    } else n=1;

    for (c=0;c<n;c++) {
        
        // ROI (Region Of Interest) management
        CAM_CHECK(camCopy,camInternalROIPolicy(source, dest, &iROI, 1));
        CAM_CHECK_ARGS(camCopy,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
        CAM_CHECK_ARGS(camCopy,(source->depth&CAM_DEPTH_MASK)>=8);
        CAM_CHECK_ARGS(camCopy,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL_DST)*8));
        CAM_CHECK_ARGS(camCopy,(dest->depth&CAM_DEPTH_MASK)>=8);
        CAM_CHECK_ARGS(camCopy,(source->depth&CAM_DEPTH_SIGN)==(dest->depth&CAM_DEPTH_SIGN));
        
        width=iROI.srcroi.width;
        height=iROI.srcroi.height;
        srcptr=(CAM_PIXEL*)iROI.srcptr;
        dstptr=(CAM_PIXEL_DST*)iROI.dstptr;
        
        INIT_MASK_MANAGEMENT;

        if (source->dataOrder==CAM_DATA_ORDER_PIXEL) {
            if (dest->dataOrder==CAM_DATA_ORDER_PIXEL) {
                for (y=0;y<height;y++) {
                    cpsrcptr=srcptr; cpdstptr=dstptr;
                    BEGIN_MASK_MANAGEMENT( \
                        srcptr=cpsrcptr+startx*iROI.srcinc;\
                        dstptr=cpdstptr+startx*iROI.dstinc;\
                    )
                        
#if CAM_FAST==8
                        if ((iROI.srcinc==iROI.dstinc)&&(iROI.srcinc==iROI.nChannels)) {
                            memcpy(dstptr,srcptr,(endx-startx)*iROI.nChannels);
                        } else
#else
#if CAM_FAST==16
                        if ((iROI.srcinc==iROI.dstinc)&&(iROI.srcinc==iROI.nChannels)) {
                            memcpy(dstptr,srcptr,((endx-startx)*iROI.nChannels)<<1);
                        } else
#endif
#endif
                        {
                            for (x=startx;x<endx;x++) {
                                for (i=0;i<iROI.nChannels;i++) {
#ifdef CAM_SATURATE
                                    valpix=*(srcptr+i);
                                    if (valpix<valmin) valpix=valmin;
                                    else if (valpix>valmax) valpix=valmax;
                                    *(dstptr+i)=valpix;
#else
                                    *(dstptr+i)=(CAM_PIXEL_DST)*(srcptr+i);
#endif
                                }
                                srcptr+=iROI.srcinc;
                                dstptr+=iROI.dstinc;
                        }   }
                        
                    END_MASK_MANAGEMENT;

                    srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
                    dstptr=(CAM_PIXEL_DST*)(((char*)cpdstptr)+dest->widthStep);
                }
            } else {
                for (y=0;y<height;y++) {
                    cpsrcptr=srcptr; cpdstptr=dstptr;
                    BEGIN_MASK_MANAGEMENT( \
                            srcptr=cpsrcptr+startx*iROI.srcinc;\
                            dstptr=cpdstptr+startx; )
                        
                        for (x=startx;x<endx;x++) {
                            for (i=0;i<iROI.nChannels;i++) {
#ifdef CAM_SATURATE
                                valpix=*(srcptr+i);
                                if (valpix<valmin) valpix=valmin;
                                else if (valpix>valmax) valpix=valmax;
                                *(dstptr+i*iROI.dstpinc)=valpix;
#else
                                *(dstptr+i*iROI.dstpinc)=(CAM_PIXEL_DST)*(srcptr+i);
#endif
                            }
                            srcptr+=iROI.srcinc;
                            dstptr++;
                        }
                        
                    END_MASK_MANAGEMENT;
                    
                    srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
                    dstptr=(CAM_PIXEL_DST*)(((char*)cpdstptr)+dest->widthStep);
                }
            }
        } else {
            if (dest->dataOrder==CAM_DATA_ORDER_PIXEL) {
                for (y=0;y<height;y++) {
                    cpsrcptr=srcptr; cpdstptr=dstptr;
                    BEGIN_MASK_MANAGEMENT( \
                            srcptr=cpsrcptr+startx; \
                            dstptr=cpdstptr+startx*iROI.dstinc; )                    
                        
                        for (x=startx;x<endx;x++) {
                            for (i=0;i<iROI.nChannels;i++) {
#ifdef CAM_SATURATE
                                valpix=*(srcptr+i*iROI.srcpinc);
                                if (valpix<valmin) valpix=valmin;
                                else if (valpix>valmax) valpix=valmax;
                                *(dstptr+i)=valpix;
#else
                                *(dstptr+i)=(CAM_PIXEL_DST)*(srcptr+i*iROI.srcpinc);
#endif
                            }
                            srcptr++;
                            dstptr+=iROI.dstinc;
                        }
                    
                    END_MASK_MANAGEMENT;
                   
                    srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
                    dstptr=(CAM_PIXEL_DST*)(((char*)cpdstptr)+dest->widthStep);
                }
            } else {
                for (y=0;y<height;y++) {
                    cpsrcptr=srcptr; cpdstptr=dstptr;
                    BEGIN_MASK_MANAGEMENT( \
                            srcptr=cpsrcptr+startx; \
                            dstptr=cpdstptr+startx; )
                        
                        for (x=startx;x<endx;x++) {
                            for (i=0;i<iROI.nChannels;i++) {
#ifdef CAM_SATURATE
                                valpix=*(srcptr+i*iROI.srcpinc);
                                if (valpix<valmin) valpix=valmin;
                                else if (valpix>valmax) valpix=valmax;
                                *(dstptr+i*iROI.dstpinc)=valpix;
#else
                                *(dstptr+i*iROI.dstpinc)=(CAM_PIXEL_DST)*(srcptr+i*iROI.srcpinc);
#endif
                            }
                            srcptr++; dstptr++;
                        }
                   
                    END_MASK_MANAGEMENT;

                    srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
                    dstptr=(CAM_PIXEL_DST*)(((char*)cpdstptr)+dest->widthStep);
                }
            }
        }
        
        camInternalROIPolicyExit(&iROI);

        // Special cases
        if (special) {
            roi.coi++;
            if (special==2) {
                roi2.coi++;
            }
        }
    }

    // Special cases : 
    if (special) {
        if (dest->nChannels==4) {
            roi.coi=4;
            camSet(dest,0); // Fill the alpha channel. All pixels are set transparent
        }
        dest->roi=tmp; // Restore roi
        if (special==2) source->roi=tmp2;
    }

    return 1;
}

#ifndef CAM_COPY_ONLY

int camZoom2x(CamImage* source, CamImage* dest)
{
    int x,y,valpix;
    int width,height;
    CAM_PIXEL *srcptr,*dstptr,*cpsrcptr,*cpdstptr;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camZoom2x,camInternalROIPolicy(source, dest, &iROI, 0));
    CAM_CHECK_ARGS(camZoom2x,iROI.nChannels==1);
    CAM_CHECK_ARGS(camZoom2x,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camZoom2x,(source->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camZoom2x,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camZoom2x,(dest->depth&CAM_DEPTH_MASK)>=8);
    
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    srcptr=(CAM_PIXEL*)iROI.srcptr;
    dstptr=(CAM_PIXEL*)iROI.dstptr;

    for (y=0;y<height;y++) {
        cpsrcptr=srcptr; cpdstptr=dstptr;
	for (x=0;x<width;x++,srcptr+=iROI.srcinc,dstptr+=(iROI.dstinc<<1)) {
	    valpix=*srcptr;
	    *dstptr=valpix;
	    *(dstptr+iROI.dstinc)=valpix;
	    *(dstptr+iROI.dstlinc)=valpix;
	    *(dstptr+iROI.dstlinc+iROI.dstinc)=valpix;
	}
	srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+source->widthStep);
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep*2);
    }

    return 1;
}

// Process all channels
int camDecimateNN(CamImage *src, CamImage *dest, int factor)
{
    int x,y,i;
    int width,height;
    CAM_PIXEL *srcptr,*dstptr,*cpsrcptr,*cpdstptr;

    CAM_CHECK_ARGS(camDecimateNN,(src->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camDecimateNN,(src->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camDecimateNN,(dest->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camDecimateNN,(dest->depth&CAM_DEPTH_MASK)>=8);
    CAM_CHECK_ARGS(camDecimateNN,(src->depth&CAM_DEPTH_MASK)==(dest->depth&CAM_DEPTH_MASK));
    CAM_CHECK_ARGS(camDecimateNN,(src->nChannels==dest->nChannels));
    CAM_CHECK_ARGS(camDecimateNN,src->dataOrder==CAM_DATA_ORDER_PIXEL);
    CAM_CHECK_ARGS(camDecimateNN,dest->dataOrder==CAM_DATA_ORDER_PIXEL);

    // ROI (Region Of Interest) management
    if (src->roi) {
	srcptr=(CAM_PIXEL*)(src->imageData+src->roi->yOffset*src->widthStep+src->roi->xOffset*sizeof(CAM_PIXEL)*src->nChannels);
	width=src->roi->width;
	height=src->roi->height;
    } else {
	srcptr=(CAM_PIXEL*)src->imageData;
	width=src->width;
	height=src->height;
    }
    if (dest->roi) {
	dstptr=(CAM_PIXEL*)(dest->imageData+dest->roi->yOffset*dest->widthStep+dest->roi->xOffset*sizeof(CAM_PIXEL)*dest->nChannels);
        CAM_CHECK_ARGS(camDecimateNN,width==dest->roi->width*factor);
        CAM_CHECK_ARGS(camDecimateNN,height==dest->roi->height*factor);
    } else {
	dstptr=(CAM_PIXEL*)dest->imageData;
        CAM_CHECK_ARGS(camDecimateNN,width==dest->width*factor);
        CAM_CHECK_ARGS(camDecimateNN,height==dest->height*factor);
    }

    for (y=0;y<height;y++) {
        cpsrcptr=srcptr; cpdstptr=dstptr;
	for (x=0;x<width/factor;x++) {
	    for (i=0;i<src->nChannels;i++,srcptr++,dstptr++) {
	        *dstptr=*srcptr;
	    }
	    srcptr+=src->nChannels*(factor-1);
	}
	srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+src->widthStep);
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);
    }

    return 1;
}

// Process all channels
int camDownScaling2x2(CamImage *src, CamImage *dest)
{
    int x,y,n;
    int width,height;
    CAM_PIXEL *srcptr,*dstptr,*cpsrcptr,*cpdstptr;

    CAM_CHECK_ARGS(camDownscaling2x2, (src->nChannels==1)||(src->dataOrder==CAM_DATA_ORDER_PIXEL));
    CAM_CHECK_ARGS(camDownscaling2x2, (dest->nChannels==1)||(dest->dataOrder==CAM_DATA_ORDER_PIXEL));
    CAM_CHECK_ARGS(camDownscaling2x2, (src->nChannels==dest->nChannels));
    CAM_CHECK_ARGS(camDownscaling2x2, ((src->depth&CAM_DEPTH_MASK)==(sizeof(CAM_PIXEL)*8)));
    CAM_CHECK_ARGS(camDownscaling2x2, ((dest->depth&CAM_DEPTH_MASK)==(sizeof(CAM_PIXEL)*8)));
    CAM_CHECK_ARGS(camDownscaling2x2, ((src->width==dest->width*2)&&(src->height==dest->height*2)));

    // ROI (Region Of Interest) management
    if (src->roi) {
	srcptr=(CAM_PIXEL*)(src->imageData+src->roi->yOffset*src->widthStep+src->roi->xOffset*sizeof(CAM_PIXEL)*src->nChannels);
	width=src->roi->width;
	height=src->roi->height;
    } else {
	srcptr=(CAM_PIXEL*)src->imageData;
	width=src->width;
	height=src->height;
    }
    if (dest->roi) {
	dstptr=(CAM_PIXEL*)(dest->imageData+dest->roi->yOffset*dest->widthStep+dest->roi->xOffset*sizeof(CAM_PIXEL)*dest->nChannels);
        CAM_CHECK_ARGS(camDownscaling2x2, width==dest->roi->width*2);
        CAM_CHECK_ARGS(camDownscaling2x2, height==dest->roi->height*2);
    } else {
	dstptr=(CAM_PIXEL*)dest->imageData;
        CAM_CHECK_ARGS(camDownscaling2x2, width==dest->width*2);
        CAM_CHECK_ARGS(camDownscaling2x2, height==dest->height*2);
    }

    for (y=0;y<height;y+=2) {
        cpsrcptr=srcptr; cpdstptr=dstptr;
        for (x=0;x<(width>>1);x++,srcptr+=src->nChannels) {
            for (n=0;n<src->nChannels;n++,dstptr++,srcptr++) {
                *dstptr=((int)(*srcptr)+(int)(*(srcptr+src->nChannels))+(int)(*(srcptr+src->widthStep))+(int)(*(srcptr+src->widthStep+src->nChannels)))>>2;
            }
        }
	srcptr=(CAM_PIXEL*)(((char*)cpsrcptr)+src->widthStep*2);
	dstptr=(CAM_PIXEL*)(((char*)cpdstptr)+dest->widthStep);
    }

    return 1;
}

int camSet(CamImage *image, int fillValue)
{
    int x,y;
    int width,height,i;
    int fv;
    CAM_PIXEL *imptr,*tmpptr;
    CamRun *run;
    int startx,endx;
    CamInternalROIPolicyStruct iROI;

    // ROI (Region Of Interest) management
    CAM_CHECK(camSet,camInternalROIPolicy(image, NULL, &iROI, 1));
    CAM_CHECK_ARGS(camSet, (image->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camSet, (image->depth&CAM_DEPTH_MASK)>=8);
    
    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    imptr=(CAM_PIXEL*)iROI.srcptr;

    // Mask management
    if (iROI.mask) {
        run=iROI.mask->runs+1; // Skip the first dummy run
    } else {
        run=NULL;
    }

    if (image->dataOrder==CAM_DATA_ORDER_PIXEL) {
        for (y=0;y<height;y++) {
            tmpptr=imptr;
            startx=0;
            do {
                // Mask management
                if (!iROI.mask) {
                    endx=width;
                } else {
                    while ((run->value==0)&&(run->line==y)) {
                        startx+=run->length;
                        run++;
                    }
                    if (run->line!=y) break; // Go directly to next line
                    endx=startx+run->length;
                    
                    imptr=tmpptr+startx*iROI.srcinc;
                }                    
                
#if CAM_FAST==8
                if ((iROI.nChannels==1)&&(iROI.srcinc==1)) {
                    memset(imptr,fillValue,endx-startx);
                } else
#endif
                {
                    for (x=startx;x<endx;x++) {
                        fv=fillValue;
                        for (i=0;i<iROI.nChannels;i++) {
                            *(imptr+i)=fv;
                            fv>>=8;
                        }
                        imptr+=iROI.srcinc;
                }   }
                
                if (iROI.mask) {
                    startx+=run->length;
                    run++;
                }
            } while ((run)&&(run->line==y));
            imptr=(CAM_PIXEL*)(((char*)tmpptr)+image->widthStep);
        }
    } else {
        for (y=0;y<height;y++) {
            tmpptr=imptr;
            startx=0;
            do {
                // Mask management
                if (!iROI.mask) {
                    endx=width;
                } else {
                    while ((run->value==0)&&(run->line==y)) {
                        startx+=run->length;
                        run++;
                    }
                    if (run->line!=y) break; // Go directly to next line
                    endx=startx+run->length;
                    
                    imptr=tmpptr+startx*iROI.srcinc;
                }                    
                
#if CAM_FAST==8
                fv=fillValue;
                for (i=0;i<iROI.nChannels;i++) {
                    memset(imptr+i*iROI.srcpinc,fv,endx-startx);
                    fv>>=8;
                }
#else
                for (x=startx;x<endx;x++) {
                    fv=fillValue;
                    for (i=0;i<iROI.nChannels;i++) {
                        *(imptr+i*iROI.srcpinc)=fv;
                        fv>>=8;
                    }
                    imptr++;
                } 
#endif
                    
                if (iROI.mask) {
                    startx+=run->length;
                    run++;
                }
            } while ((run)&&(run->line==y));
            imptr=(CAM_PIXEL*)(((char*)tmpptr)+image->widthStep);
        }
    }

    camInternalROIPolicyExit(&iROI);
    return 1;
}

#endif // CAM_COPY_ONLY


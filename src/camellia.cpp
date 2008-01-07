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
#include <string.h>
#include "camellia.h"

CamImage::CamImage(int width, int height, int depth, int colormodel)
{
    camAllocateImageEx(this, width, height, depth, colormodel);
}

CamImage::CamImage(const CamImage& image)
{
    camClone((CamImage*)&image,this);
}

CamImage::~CamImage()
{
    camDeallocateImage(this);
}

CamImage& CamImage::operator=(const CamImage &image)
{
    if (imageData) camDeallocateImage(this);
    camClone((CamImage*)&image,this);
    return *this;
}

CamImage* CamImage::clone() const
{
    CamImage *image=new CamImage;
    camClone((CamImage*)this,image);
    return image;
}

CamImage* CamImage::copy() const
{
    CamImage *image=new CamImage;
    camCopy((CamImage*)this,image);
    return image;
}

bool CamImage::copy(CamImage &dest) const
{
    return (camCopy((CamImage*)this,&dest))?true:false;
}

bool CamImage::alloc(int width, int height, int depth, int channelseq)
{
    if (imageData) deallocate();
    return (camAllocateImageEx(this, width, height, depth, channelseq))?true:false;
}

bool CamImage::fill_header(int width, int height, int depth, int channelseq)
{
    if (imageData) deallocate();
    return (camFillImageHeader(this, width, height, depth, channelseq))?true:false;
}

bool CamImage::alloc_rgb(int width, int height)
{
    if (imageData) deallocate();
    return (camAllocateRGBImage(this, width, height))?true:false;
}

bool CamImage::alloc_rgba(int width, int height)
{
    if (imageData) deallocate();
    return (camAllocateRGBAImage(this, width, height))?true:false;
}

bool CamImage::alloc_bgr(int width, int height)
{
    if (imageData) deallocate();
    return (camAllocateBGRImage(this, width, height))?true:false;
}

bool CamImage::alloc_bgra(int width, int height)
{
    if (imageData) deallocate();
    return (camAllocateBGRAImage(this, width, height))?true:false;
}

bool CamImage::alloc_yuv(int width, int height)
{
    if (imageData) deallocate();
    return (camAllocateYUVImage(this, width, height))?true:false;
}

bool CamImage::alloc_hls(int width, int height)
{
    if (imageData) deallocate();
    return (camAllocateHLSImage(this, width, height))?true:false;
}

bool CamImage::deallocate()
{
    return (camDeallocateImage(this))?true:false;
}

bool CamImage::set_roi(const CamROI &roi) {
    this->roi=(CamROI*)&roi;
    return true;
}

bool CamImage::alpha_composite(const CamImage& source2, CamImage& dest) const
{
    return (camAlphaComposite((CamImage*)this,(CamImage*)&source2,&dest))?true:false;
}

void CamImage::get_pixels(char **s, int *sz) const
{
    if (this->imageData) {
        *s=new char[this->imageSize];
        memcpy(*s,this->imageData,this->imageSize);
        *sz=this->imageSize;
    } else {
        *s=NULL;
        *sz=0;
    }
}

bool CamImage::set_pixels(const char *pixels, int sz)
{
    if (this->imageData) {
        if (sz!=this->imageSize) {
            camError("CamImage::set_pixels","Wrong number of pixels");
            return false;
        } else {
            memcpy(this->imageData,pixels,sz);
            return true;
        }
    } else {
        camError("CamImage::set_pixels","The image must be allocated before calling this function");
        return false;
    }
}

void CamImage::inspect(char **s, int *sz) const
{
    char str[256],ch[8];
    if (this->imageData) {
        if (this->nChannels>1) {
            if (this->colorModel[0]=='R') {
                if (this->nChannels==4)
                    strcpy(ch,"s(RGBA)");
                else strcpy(ch,"s(RGB)");
            } else if (this->colorModel[0]=='Y') {
                strcpy(ch,"s(YUV)");
            }
        } else strcpy(ch,"");
        sprintf(str,"CamImage : %dx%d, %dch%s, %dbpp",
            this->width, this->height, this->nChannels, ch,
            (this->depth&CAM_DEPTH_MASK));
    } else {
        sprintf(str,"CamImage : Unallocated");
    }
    *sz=strlen(str);
    *s=new char[*sz+1];
    strcpy(*s,str);
}

bool CamImage::load_pgm(const char *filename)
{
    if (imageData) deallocate();
    return (camLoadPGM(this, (char*)filename))?true:false;
}

bool CamImage::save_pgm(const char *filename) const
{
    return (camSavePGM((CamImage*)this, (char*)filename))?true:false;
}

bool CamImage::save_raw_pgm(const char *filename) const
{
    return (camSaveRawPGM((CamImage*)this, (char*)filename))?true:false;
}

bool CamImage::load_bmp(const char *filename)
{
    if (imageData) deallocate();
    return (camLoadBMP(this, (char*)filename))?true:false;
}

bool CamImage::save_bmp(const char *filename) const
{
    return (camSaveBMP((CamImage*)this, (char*)filename))?true:false;
}

bool CamImage::view() const
{
    return (camView((CamImage*)this))?true:false;
}

bool CamImage::set(int color)
{
    return (camSet(this,color))?true:false;
}

int CamImage::erode_square3()
{
    return camErodeSquare3(this,this);
}

int CamImage::erode_square3(CamImage &dest) const
{
    return camErodeSquare3((CamImage*)this,&dest);
}

int CamImage::erode_circle5()
{
    return camErodeCircle5(this,this);
}

int CamImage::erode_circle5(CamImage &dest) const
{
    return camErodeCircle5((CamImage*)this,&dest);
}

int CamImage::erode_circle7()
{
    return camErodeCircle7(this,this);
}

int CamImage::erode_circle7(CamImage &dest) const
{
    return camErodeCircle7((CamImage*)this,&dest);
}

int CamImage::dilate_square3()
{
    return camDilateSquare3(this,this);
}

int CamImage::dilate_square3(CamImage &dest) const
{
    return camDilateSquare3((CamImage*)this,&dest);
}

int CamImage::dilate_circle5()
{
    return camDilateCircle5(this,this);
}

int CamImage::dilate_circle5(CamImage &dest) const
{
    return camDilateCircle5((CamImage*)this,&dest);
}

int CamImage::dilate_circle7()
{
    return camDilateCircle7(this,this);
}

int CamImage::dilate_circle7(CamImage &dest) const
{
    return camDilateCircle7((CamImage*)this,&dest);
}

int CamImage::morpho_gradient_square3()
{
    return camMorphoGradientSquare3(this,this);
}

int CamImage::morpho_gradient_square3(CamImage &dest) const
{
    return camMorphoGradientSquare3((CamImage*)this,&dest);
}

int CamImage::morpho_gradient_circle5()
{
    return camMorphoGradientCircle5(this,this);
}

int CamImage::morpho_gradient_circle5(CamImage &dest) const
{
    return camMorphoGradientCircle5((CamImage*)this,&dest);
}

int CamImage::morpho_gradient_circle7()
{
    return camMorphoGradientCircle7(this,this);
}

int CamImage::morpho_gradient_circle7(CamImage &dest) const
{
    return camMorphoGradientCircle7((CamImage*)this,&dest);
}

int CamImage::morpho_maths(const CamMorphoMathsKernel &ker)
{
    return camMorphoMaths(this,this,(CamMorphoMathsKernel*)&ker);
}

int CamImage::morpho_maths(CamImage &dest, const CamMorphoMathsKernel &k) const
{
    return camMorphoMaths((CamImage*)this,(CamImage*)&dest,(CamMorphoMathsKernel*)&k);
}

int CamImage::erode_3x3(const CamMorphoMathsKernel &ker)
{
    return camErode3x3(this,this,(CamMorphoMathsKernel*)&ker);
}

int CamImage::erode_3x3(CamImage &dest, const CamMorphoMathsKernel &k) const
{
    return camErode3x3((CamImage*)this,(CamImage*)&dest,(CamMorphoMathsKernel*)&k);
}

int CamImage::dilate_3x3(const CamMorphoMathsKernel &ker)
{
    return camDilate3x3(this,this,(CamMorphoMathsKernel*)&ker);
}

int CamImage::dilate_3x3(CamImage &dest, const CamMorphoMathsKernel &k) const
{
    return camDilate3x3((CamImage*)this,(CamImage*)&dest,(CamMorphoMathsKernel*)&k);
}

int CamImage::erode_5x5(const CamMorphoMathsKernel &ker)
{
    return camErode5x5(this,this,(CamMorphoMathsKernel*)&ker);
}

int CamImage::erode_5x5(CamImage &dest, const CamMorphoMathsKernel &k) const
{
    return camErode5x5((CamImage*)this,(CamImage*)&dest,(CamMorphoMathsKernel*)&k);
}

int CamImage::dilate_5x5(const CamMorphoMathsKernel &ker)
{
    return camDilate5x5(this,this,(CamMorphoMathsKernel*)&ker);
}

int CamImage::dilate_5x5(CamImage &dest, const CamMorphoMathsKernel &k) const
{
    return camDilate5x5((CamImage*)this,(CamImage*)&dest,(CamMorphoMathsKernel*)&k);
}

int CamImage::erode_7x7(const CamMorphoMathsKernel &ker)
{
    return camErode7x7(this,this,(CamMorphoMathsKernel*)&ker);
}

int CamImage::erode_7x7(CamImage &dest, const CamMorphoMathsKernel &k) const
{
    return camErode7x7((CamImage*)this,(CamImage*)&dest,(CamMorphoMathsKernel*)&k);
}

int CamImage::dilate_7x7(const CamMorphoMathsKernel &ker)
{
    return camDilate7x7(this,this,(CamMorphoMathsKernel*)&ker);
}

int CamImage::dilate_7x7(CamImage &dest, const CamMorphoMathsKernel &k) const
{
    return camDilate7x7((CamImage*)this,(CamImage*)&dest,(CamMorphoMathsKernel*)&k);
}

int CamImage::linear_filter_3x3(const CamLinearFilterKernel &ker)
{
    return camLinearFilter3x3(this,this,(CamLinearFilterKernel*)&ker);
}

int CamImage::linear_filter_3x3(CamImage &dest, const CamLinearFilterKernel &k) const
{
    return camLinearFilter3x3((CamImage*)this,(CamImage*)&dest,(CamLinearFilterKernel*)&k);
}

int CamImage::linear_filter_5x5(const CamLinearFilterKernel &ker)
{
    return camLinearFilter5x5(this,this,(CamLinearFilterKernel*)&ker);
}

int CamImage::linear_filter_5x5(CamImage &dest, const CamLinearFilterKernel &k) const
{
    return camLinearFilter5x5((CamImage*)this,(CamImage*)&dest,(CamLinearFilterKernel*)&k);
}

int CamImage::linear_filter_abs_3x3(const CamLinearFilterKernel &ker)
{
    return camLinearFilterAbs3x3(this,this,(CamLinearFilterKernel*)&ker);
}

int CamImage::linear_filter_abs_3x3(CamImage &dest, const CamLinearFilterKernel &k) const
{
    return camLinearFilterAbs3x3((CamImage*)this,(CamImage*)&dest,(CamLinearFilterKernel*)&k);
}

int CamImage::linear_filter_abs_5x5(const CamLinearFilterKernel &ker)
{
    return camLinearFilterAbs5x5(this,this,(CamLinearFilterKernel*)&ker);
}

int CamImage::linear_filter_abs_5x5(CamImage &dest, const CamLinearFilterKernel &k) const
{
    return camLinearFilterAbs5x5((CamImage*)this,(CamImage*)&dest,(CamLinearFilterKernel*)&k);
}

bool CamImage::sobel_v()
{
    return (camSobelV(this,this))?true:false;
}

bool CamImage::sobel_h()
{
    return (camSobelH(this,this))?true:false;
}

bool CamImage::sobel_v_abs()
{
    return (camSobelVAbs(this,this))?true:false;
}

bool CamImage::sobel_h_abs()
{
    return (camSobelHAbs(this,this))?true:false;
}

bool CamImage::sobel_v(CamImage &dest) const
{
    return (camSobelV((CamImage*)this,(CamImage*)&dest))?true:false;
}

bool CamImage::sobel_h(CamImage &dest) const
{
    return (camSobelH((CamImage*)this,(CamImage*)&dest))?true:false;
}

bool CamImage::sobel_v_abs(CamImage &dest) const
{
    return (camSobelVAbs((CamImage*)this,(CamImage*)&dest))?true:false;
}

bool CamImage::sobel_h_abs(CamImage &dest) const
{
    return (camSobelHAbs((CamImage*)this,(CamImage*)&dest))?true:false;
}

int CamImage::sep_filter_3x3(const CamSepFilterKernel &ker)
{
    return camSepFilter3x3(this,this,(CamSepFilterKernel*)&ker);
}

int CamImage::sep_filter_3x3(CamImage &dest, const CamSepFilterKernel &k) const
{
    return camSepFilter3x3((CamImage*)this,(CamImage*)&dest,(CamSepFilterKernel*)&k);
}

int CamImage::sep_filter_5x5(const CamSepFilterKernel &ker)
{
    return camSepFilter5x5(this,this,(CamSepFilterKernel*)&ker);
}

int CamImage::sep_filter_5x5(CamImage &dest, const CamSepFilterKernel &k) const
{
    return camSepFilter5x5((CamImage*)this,(CamImage*)&dest,(CamSepFilterKernel*)&k);
}

int CamImage::sep_filter_7x7(const CamSepFilterKernel &ker)
{
    return camSepFilter7x7(this,this,(CamSepFilterKernel*)&ker);
}

int CamImage::sep_filter_7x7(CamImage &dest, const CamSepFilterKernel &k) const
{
    return camSepFilter7x7((CamImage*)this,(CamImage*)&dest,(CamSepFilterKernel*)&k);
}

int CamImage::sep_filter_abs_3x3(const CamSepFilterKernel &ker)
{
    return camSepFilterAbs3x3(this,this,(CamSepFilterKernel*)&ker);
}

int CamImage::sep_filter_abs_3x3(CamImage &dest, const CamSepFilterKernel &k) const
{
    return camSepFilterAbs3x3((CamImage*)this,(CamImage*)&dest,(CamSepFilterKernel*)&k);
}

int CamImage::sep_filter_abs_5x5(const CamSepFilterKernel &ker)
{
    return camSepFilterAbs5x5(this,this,(CamSepFilterKernel*)&ker);
}

int CamImage::sep_filter_abs_5x5(CamImage &dest, const CamSepFilterKernel &k) const
{
    return camSepFilterAbs5x5((CamImage*)this,(CamImage*)&dest,(CamSepFilterKernel*)&k);
}

int CamImage::sep_filter_abs_7x7(const CamSepFilterKernel &ker)
{
    return camSepFilterAbs7x7(this,this,(CamSepFilterKernel*)&ker);
}

int CamImage::sep_filter_abs_7x7(CamImage &dest, const CamSepFilterKernel &k) const
{
    return camSepFilterAbs7x7((CamImage*)this,(CamImage*)&dest,(CamSepFilterKernel*)&k);
}

bool CamImage::fixed_filter(int filter)
{
    return (camFixedFilter(this,this,filter))?true:false;
}

bool CamImage::fixed_filter(CamImage &dest, int filter) const
{
    return (camFixedFilter((CamImage*)this,(CamImage*)&dest,filter))?true:false;
}

bool CamImage::draw_line(int x1, int y1, int x2, int y2, int color)
{
    return (camDrawLine(this,x1,y1,x2,y2,color))?true:false;
}

bool CamImage::accumulate_line(int x1, int y1, int x2, int y2, int acc)
{
    return (camAccumulateLine(this,x1,y1,x2,y2,acc))?true:false;
}

bool CamImage::draw_rectangle(int x1, int y1, int x2, int y2, int color)
{
    return (camDrawRectangle(this,x1,y1,x2,y2,color))?true:false;
}

bool CamImage::draw_text_16s(const char *text, int x, int y, int cwidth, int cheight, int orientation, int color)
{
    return (camDrawText16s(this,(char*)text,x,y,cwidth,cheight,orientation,color))?true:false;
}

bool CamImage::draw_text_bitmap(const char *text, int x, int y, const CamBitmapFont &font)
{
    return (camDrawTextBitmap(this,(char*)text,x,y,(CamBitmapFont*)&font))?true:false;
}

bool CamImage::draw_circle(int x, int y, int r, int color)
{
    return (camDrawCircle(this,x,y,r,color))?true:false;
}

bool CamImage::draw_ellipse(int x, int y, int rx, int ry, int color)
{
    return (camDrawEllipse(this,x,y,rx,ry,color))?true:false;
}

bool CamImage::plot(int x, int y, int color, int kind)
{
    return (camPlot(this,x,y,color,kind))?true:false;
}

int CamImage::fill_color(int x, int y, int fillcolor, int tolerance)
{
    return camFillColor(this,x,y,fillcolor,tolerance);
}

bool CamImage::warping(CamImage &dest, int interpolation_method, bool perspective, const CamPoint &ul, const CamPoint &ur, const CamPoint &lr, const CamPoint &ll) const
{
    CamWarpingParams params;

    params.interpolation = interpolation_method;
    params.perspective = (perspective)?1:0;
    params.p[0] = ul;
    params.p[1] = ur;
    params.p[2] = lr;
    params.p[3] = ll;
    return (camWarping((CamImage*)this, &dest, &params))?true:false;
}

bool CamImage::scale(CamImage &dest) const
{
    return (camScale((CamImage*)this,&dest))?true:false;
}

bool CamImage::set_mask(const CamRLEImage &mask)
{
    return (camSetRLEMask(this,(CamRLEImage*)&mask))?true:false;
}

bool CamImage::set_mask(const CamImage &mask)
{
    return (camSetMask(this,(CamImage*)&mask))?true:false;
}

bool CamImage::apply_lut(const CamTable &lut)
{
    return (camApplyLUT(this,this,(CamTable*)&lut))?true:false;
}
 
bool CamImage::apply_lut(CamImage &dest, const CamTable &lut) const
{
    return (camApplyLUT((CamImage*)this,&dest,(CamTable*)&lut))?true:false;
}

CamImage *CamImage::to_y() const
{
    CamImage *image=new CamImage;
    camRGB2Y((CamImage*)this,image);
    return image;
}

bool CamImage::to_y(CamImage &dest) const
{
    return (camRGB2Y((CamImage*)this,&dest))?true:false;
}

CamImage *CamImage::to_yuv() const
{
    CamImage *image=new CamImage;
    camRGB2YUV((CamImage*)this,image);
    return image;
}

bool CamImage::to_yuv(CamImage &dest) const
{
    return (camRGB2YUV((CamImage*)this,&dest))?true:false;
}

CamImage *CamImage::to_hls() const
{
    CamImage *image=new CamImage;
    camRGB2HLS((CamImage*)this,image);
    return image;
}

bool CamImage::to_hls(CamImage &dest) const
{
    return (camRGB2HLS((CamImage*)this,&dest))?true:false;
}

CamImage *CamImage::to_rgb() const
{
    CamImage *image=new CamImage;
    camYUV2RGB((CamImage*)this,image);
    return image;
}

bool CamImage::to_rgb(CamImage &dest) const
{
    return (camYUV2RGB((CamImage*)this,&dest))?true:false;
}

CamRLEImage* CamImage::encode() const
{
    CamRLEImage *image=new CamRLEImage;
    camRLEEncode((CamImage*)this,image);
    return image;
}

CamRLEImage* CamImage::encode_lut(const CamTable &LUT) const
{
    CamRLEImage *image=new CamRLEImage;
    camRLEEncodeLUT((CamImage*)this,image,(CamTable*)&LUT);
    return image;
}

CamRLEImage* CamImage::encode_threshold(int threshold) const
{
    CamRLEImage *image=new CamRLEImage;
    camRLEEncodeThreshold((CamImage*)this,image,threshold);
    return image;
}

CamRLEImage* CamImage::encode_threshold_inv(int threshold) const
{
    CamRLEImage *image=new CamRLEImage;
    camRLEEncodeThresholdInv((CamImage*)this,image,threshold);
    return image;
}

CamRLEImage* CamImage::encode_color(const CamTable &clusters) const
{
    CamRLEImage *image=new CamRLEImage;
    camRLEEncodeColor((CamImage*)this,image,(CamTable*)&clusters);
    return image;
}

bool CamImage::encode(CamRLEImage& dest) const
{
    return (camRLEEncode((CamImage*)this,&dest))?true:false;
}

bool CamImage::encode_lut(CamRLEImage& dest, const CamTable &LUT) const
{
    return (camRLEEncodeLUT((CamImage*)this,&dest,(CamTable*)&LUT))?true:false;
}

bool CamImage::encode_threshold(CamRLEImage& dest, int threshold) const
{
    return (camRLEEncodeThreshold((CamImage*)this,&dest,threshold))?true:false;
}

bool CamImage::encode_threshold_inv(CamRLEImage& dest, int threshold) const
{
    return (camRLEEncodeThresholdInv((CamImage*)this,&dest,threshold))?true:false;
}

int CamImage::threshold(int threshold)
{
    return camThreshold(this,this,threshold);
}

int CamImage::threshold(CamImage &dest, int threshold) const
{
    return camThreshold((CamImage*)this,&dest,threshold);
}

int CamImage::threshold_inv(int threshold)
{
    return camThresholdInv(this,this,threshold);
}

int CamImage::threshold_inv(CamImage &dest, int threshold) const
{
    return camThresholdInv((CamImage*)this,&dest,threshold);
}

int CamImage::abs()
{
    return camAbs(this,this);
}

int CamImage::abs(CamImage &dest) const
{
    return camAbs((CamImage*)this,&dest);
}

int CamImage::arithm(int operation, int c1, int c2, int c3)
{
    CamArithmParams params;
    params.operation=operation;
    params.c1=c1;
    params.c2=c2;
    params.c3=c3;
    return camMonadicArithm(this,this,&params);
}
 
int CamImage::arithm(CamImage& dest, int operation, int c1, int c2, int c3) const
{
    CamArithmParams params;
    params.operation=operation;
    params.c1=c1;
    params.c2=c2;
    params.c3=c3;
    return camMonadicArithm((CamImage*)this,&dest,&params);
}

int CamImage::arithm(const CamImage& source2, CamImage& dest, int operation, int c1, int c2, int c3, int c4) const
{
    CamArithmParams params;
    params.operation=operation;
    params.c1=c1;
    params.c2=c2;
    params.c3=c3;
    return camDyadicArithm((CamImage*)this,(CamImage*)&source2,&dest,&params);
}

CamMeasuresResults CamImage::measures() const
{
    CamMeasuresResults r;
    camMeasures((CamImage*)this,&r);
    return r;
}

float CamImage::average_deviation(int average) const
{
    return camMeasureAverageDeviation((CamImage*)this,average);
}

bool CamImage::sum_hv(CamTable& hsum, CamTable &vsum) const
{
    return (camSumHV((CamImage*)this,&hsum,&vsum))?true:false;
}

bool CamImage::sum_h(CamTable& sum) const
{
    return (camSumH((CamImage*)this,&sum))?true:false;
}

bool CamImage::sum_v(CamTable& sum) const
{
    return (camSumV((CamImage*)this,&sum))?true:false;
}

int CamImage::histogram(CamTable& histo) const
{
    return camHistogram((CamImage*)this,&histo);
}

bool CamImage::histogram_equalization(CamImage &dest, const CamTable &src_histo, int option, CamImage *work)
{
    return (camHistogramEqualization((CamImage*)this, (CamImage*)&dest, (CamTable*)&src_histo, option, work))?true:false;
}

bool CamImage::histogram_2_channels(int ch1, int ch2, CamImage &result, int size) const
{
    return (camHistogram2Channels((CamImage*)this,ch1,ch2,&result,size))?true:false;
}

int CamImage::find_threshold(int percent) const
{
    CamTable t;
    this->histogram(t);
    return camFindThreshold(&t,percent);
}

int CamImage::hough_circle(int percent, int rmin, int rmax, int &xc, int &yc, int &rc) const
{
    return camHoughCircle((CamImage*)this,percent,rmin,rmax,&xc,&yc,&rc);
}

int CamImage::hierarchical_watershed(CamImage &ws, CamTableOfBasins &tob) const
{
    if (ws.imageData==NULL) {
        ws.alloc(this->width, this->height, CAM_DEPTH_16S);
    }
    if (tob.sizeMax!=0) camFreeTableOfBasins(&tob);
    return camHierarchicalWatershed((CamImage*)this, &ws, &tob);
}

int CamImage::hierarchical_watershed_contours(CamImage &ws, CamTableOfBasins &tob) const
{
    if (ws.imageData==NULL) {
        ws.alloc(this->width, this->height, CAM_DEPTH_16S);
    }
    if (tob.sizeMax!=0) camFreeTableOfBasins(&tob);
    return camHierarchicalWatershedContours((CamImage*)this, &ws, &tob);
}

int CamImage::hierarchical_watershed_regions(const CamTableOfBasins &tob)
{
    return camHierarchicalWatershedRegions(this,(CamTableOfBasins*)&tob);
}

bool CamImage::draw_keypoints(const CamKeypoints &points, int color)
{
    return (camDrawKeypoints((CamKeypoints*)&points, (CamImage*)this, color)) ? true : false; 
}

bool CamImage::draw_keypoint(const CamKeypoint &point, int color)
{
    return (camDrawKeypoint((CamKeypoint*)&point, (CamImage*)this, color)) ? true : false; 
}

bool CamImage::harris(CamKeypoints &points, int k) const
{   
    return (camHarris((CamImage*)this, (CamKeypoints*)&points, k)) ? true : false; 
}

bool CamImage::integral_image(CamImage &dest) const
{
    return (camIntegralImage((CamImage*)this, (CamImage*)&dest)) ? true : false; 
}

CamImage *CamImage::integral_image() const
{
    CamImage *dest = new CamImage;
    if (camIntegralImage((CamImage*)this, dest)) return dest;
    delete dest;
    return NULL;
}

bool CamImage::fast_hessian_detector(CamKeypoints &points, int threshold, int options) const
{
    return (camFastHessianDetector((CamImage*)this, (CamKeypoints*)&points, threshold, options)) ? true : false; 
}

// CamROI class management

CamROI CamROI::intersect(const CamROI &roi) const
{
    CamROI res;
    camROIIntersect((CamROI*)this,(CamROI*)&roi,&res);
    return res;
}

bool CamROI::clip(CamImage &image)
{
    return (camClip(this,&image))?true:false;
}

bool CamROI::reduce(int i)
{
    return (camReduceROI(this,i))?true:false;
}

bool CamROI::enlarge(int i)
{
    return (camEnlargeROI(this,i))?true:false;
}

int &CamTable::operator[](int n)
{
    if ((n>=0)&&(n<CAM_TABLE_SIZE)&&(n<size)) return t[n];
    else camError("CamTable","Index out of range");
    return t[0];
}

CamBlobInfo& CamBlobs::operator[](int index)
{
    if ((index>=0)&&(index<nbBlobs)) return blobInfo[index];
    else camError("CamBlobs","Index out of range");
    return blobInfo[0];
}

CamRLEImage::CamRLEImage(int nbruns)
{
    camRLEAllocate(this,nbruns);
}

CamRLEImage::CamRLEImage(const CamRLEImage &image)
{
    camRLEClone((CamRLEImage*)&image,this);
}

CamRLEImage::~CamRLEImage()
{
    camRLEDeallocate(this);
}

CamRLEImage& CamRLEImage::operator=(const CamRLEImage &image)
{
    camRLEDeallocate(this);
    camRLEClone((CamRLEImage*)&image,this);
    return *this;
}

CamRLEImage* CamRLEImage::clone() const
{
    CamRLEImage *image=new CamRLEImage;
    camRLEClone((CamRLEImage*)this,image);
    return image;
}

bool CamRLEImage::alloc(int nbruns)
{
    return (camRLEAllocate(this,nbruns))?true:false;
}

bool CamRLEImage::realloc(int nbruns)
{
    return (camRLEReallocate(this,nbruns))?true:false;
}

bool CamRLEImage::encode(const CamImage &image)
{
    return (camRLEEncode((CamImage*)&image,this))?true:false;
}

bool CamRLEImage::encode_lut(const CamImage &image, const CamTable &LUT)
{
    return (camRLEEncodeLUT((CamImage*)&image,this,(CamTable*)&LUT))?true:false;
}

bool CamRLEImage::encode_threshold(const CamImage &image, int threshold)
{
    return (camRLEEncodeThreshold((CamImage*)&image,this,threshold))?true:false;
}

bool CamRLEImage::encode_threshold_inv(const CamImage &image, int threshold)
{
    return (camRLEEncodeThresholdInv((CamImage*)&image,this,threshold))?true:false;
}

bool CamRLEImage::encode_color(const CamImage &image, const CamTable &clusters)
{
    return (camRLEEncodeColor((CamImage*)&image,this,(CamTable*)&clusters))?true:false;
}

bool CamRLEImage::labeling(CamBlobAnalysisResults &results)            
{
    return (camRLELabeling(this,&results))?true:false;
}

CamBlobAnalysisResults *CamRLEImage::labeling()            
{
    CamBlobAnalysisResults *res=new CamBlobAnalysisResults;
    if (!camRLELabeling(this,res)) return NULL;
    return res;
}

bool CamRLEImage::blob_analysis(CamBlobAnalysisResults &results) const 
{
    return (camRLEBlobAnalysis((CamRLEImage*)this,&results))?true:false;
}

bool CamRLEImage::apply_lut(const CamTable &LUT)                         
{
    return (camRLEApplyLUT(this,this,(CamTable*)&LUT))?true:false;
}

bool CamRLEImage::apply_lut(CamRLEImage &dest, const CamTable &LUT) const
{
    return (camRLEApplyLUT((CamRLEImage*)this,&dest,(CamTable*)&LUT))?true:false;;
}

bool CamRLEImage::decode(CamImage &dest) const                         
{
    return (camRLEDecode((CamRLEImage*)this,&dest,NULL))?false:true;
}

bool CamRLEImage::decode(CamImage &dest, const CamTable &LUT) const      
{
    return (camRLEDecode((CamRLEImage*)this,&dest,(CamTable*)&LUT))?false:true;
}

bool CamRLEImage::decode_blobs(CamImage &dest) const                   
{
    return (camRLEDecodeBlobs((CamRLEImage*)this,&dest,NULL))?false:true;
}

bool CamRLEImage::decode_blobs(CamImage &dest, const CamTable &LUT) const
{
    return (camRLEDecodeBlobs((CamRLEImage*)this,&dest,(CamTable*)&LUT))?false:true;
}

bool CamRLEImage::inverse()                           
{
    return (camRLEInverse(this))?true:false;
}

bool CamRLEImage::erode_cross(CamRLEImage &dest) const
{
    return (camRLEErodeCross((CamRLEImage*)this,&dest))?true:false;
}

bool CamRLEImage::erode_3x3(CamRLEImage &dest) const
{
    return (camRLEErode3x3((CamRLEImage*)this,&dest))?true:false;
}

bool CamRLEImage::erode_3x2(CamRLEImage &dest) const 
{
    return (camRLEErode3x2((CamRLEImage*)this,&dest))?true:false;
}

CamRLEImage* CamRLEImage::erode_cross() const
{
    CamRLEImage *dest=new CamRLEImage(this->allocated);
    camRLEErodeCross((CamRLEImage*)this,dest);
    return dest;
}

CamRLEImage* CamRLEImage::erode_3x3() const
{
    CamRLEImage *dest=new CamRLEImage(this->allocated);    
    camRLEErode3x3((CamRLEImage*)this,dest);
    return dest;
}

CamRLEImage* CamRLEImage::erode_3x2() const 
{
    CamRLEImage *dest=new CamRLEImage(this->allocated);
    camRLEErode3x2((CamRLEImage*)this,dest);
    return dest;
}

CamTableOfBasins::~CamTableOfBasins()
{
    camFreeTableOfBasins(this);
}

CamBitmapFont::CamBitmapFont(const char* filename)
{
    first_char=33; nb_chars=0; masks=NULL; letters=NULL;
    camLoadBitmapFont(this, (char*)filename);
}

CamBitmapFont::~CamBitmapFont()
{
    camFreeBitmapFont(this);
}

bool CamBitmapFont::load(const char *filename)
{
    camFreeBitmapFont(this);
    return (camLoadBitmapFont(this, (char*)filename))?true:false;
}

CamKeypoints::CamKeypoints(int nbruns)
{
    camAllocateKeypoints(this, nbruns);
}

CamKeypoints::~CamKeypoints()
{
    camFreeKeypoints(this);
}

bool CamKeypoints::alloc(int nbp)
{
    if (nbPoints != 0) return false;
    return (camAllocateKeypoints(this, nbp))?true:false;
}

bool CamKeypoints::realloc(int nbpoints)
{
    return (camReallocateKeypoints(this, nbpoints))?true:false;
}

CamKeypoint& CamKeypoints::operator[](int index)
{
    if ((index >= 0) && (index < nbPoints)) return *keypoint[index];
    else camError("CamKeypoints", "Index out of range");
    return *keypoint[0];
}

bool CamKeypoints::draw(CamImage &image, int color) const
{
    return (camDrawKeypoints((CamKeypoints*)this, &image, color))?true:false;
}

int CamKeypoints::matching(const CamKeypoints **models, int nbModels, CamKeypointsMatches &matches) const
{
    return camKeypointsMatching((CamKeypoints*)this, (CamKeypoints**)models, nbModels, &matches);
}
    
int CamKeypoints::matching2(const CamKeypoints &points, CamKeypointsMatches &matches) const
{
    return camKeypointsMatching2((CamKeypoints*)this, (CamKeypoints*)&points, &matches);
}

int CamKeypoints::matchingKdTree(const CamKeypointsKdTree &kdTree, CamKeypointsMatches &matches, int explore) const 
{
    return camKeypointsMatchingKdTree((CamKeypoints*)this, kdTree.root, &matches, explore);
} 

bool CamKeypoints::add(CamKeypoint &p)
{
    if (nbPoints >= allocated) return false;
    keypoint[nbPoints++] = &p;
    p.set = this;
    return true;
}

CamKeypoint *CamKeypointsKdTree::find(const CamKeypoint *point, int explore, int *dist1, int *dist2) const
{
    return camFindKeypointKdTree((CamKeypoint*)point, this->root, explore, dist1, dist2);
}

bool CamKeypoint::draw(CamImage &image, int color) const
{
    return (camDrawKeypoint((CamKeypoint*)this, &image, color))?true:false;
}
   
bool CamKeypoint::set_descriptor(const int* const des, int sz)
{
    if (sz < 0 || sz > 128) return false; 
    for (int i = 0; i < sz; i++) {
      descriptor[i] = des[i];
    }
    size = sz;
    return true;
}

CamPoint CamPoint::apply_affine_transform(const CamAffineTransform &t) const
{
    CamPoint result;
    camApplyAffineTransform((CamPoint*)this, &result, (CamAffineTransform*)&t);
    return result; 
}

CamKeypointsMatches::CamKeypointsMatches(int size)
{
    camAllocateKeypointsMatches(this, size);
}

CamKeypointsMatches::~CamKeypointsMatches()
{
    camFreeKeypointsMatches(this);
}

CamAffineTransform CamKeypointsMatches::find_affine_transform(int *error) const
{
    CamAffineTransform t;
    camFindAffineTransform((CamKeypointsMatches*)this, &t, error);
    return t;
}

CamAffineTransform CamKeypointsMatches::find_affine_transform2(int *error) const
{
    CamAffineTransform t;
    camFindAffineTransform2((CamKeypointsMatches*)this, &t, error);
    return t;
}

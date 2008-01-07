/** \file camellia.h
 *  \brief Camellia Image Processing Library header file
 *  \author Bruno STEUX (Mines Paris / ParisTech)
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

#ifndef _CAMELLIA_H_
#define _CAMELLIA_H_

#define CAM_VERSION "2.7.0 : Bastille ($Rev$)"

#include <stdlib.h>

/*******************************************
 * Compilation options :                   *
 */
// Pixel definition
#define CAM_PIXEL unsigned char
#define CAM_SIGNED_PIXEL signed char

// Max image size
#define CAM_MAX_SCANLINE 1280
#define CAM_MAX_FRAME_HEIGHT 1024

// 64-bit processor or memory bandwidth?
#define CAM_64BITS

// Pentium4 optimizations?
#define CAM_OPT_P4

// Big endian architecture?
//#define CAM_BIG_ENDIAN

// Generate 8 AND 16 bits pixel size code
#define CAM_GENERATE_FULL_CODE

/*                                         *
 *******************************************/ 
#ifdef _WIN32
#define CAM_INT64 __int64
#define CAM_UINT64 unsigned __int64
#else
#define CAM_INT64 long long
#define CAM_UINT64 unsigned long long
#endif

#ifdef __INTEL_COMPILER
#define CAM_ALIGN16 __declspec(align(16))
#else
#define CAM_ALIGN16
#endif

#define CAM_FIXED_POINT signed long
#define CAM_FLOAT2FIXED(x,dot_pos) ((CAM_FIXED_POINT)((x)*(1<<dot_pos)))

#define CAM_DEPTH_SIGN 0x80000000               
#define CAM_DEPTH_MASK 0x7FFFFFFF               

#define CAM_DEPTH_1U     1
#define CAM_DEPTH_8U     8
#define CAM_DEPTH_10U   10
#define CAM_DEPTH_12U   12
#define CAM_DEPTH_16U   16
#define CAM_DEPTH_32U	32

#define CAM_DEPTH_8S  (CAM_DEPTH_SIGN| 8)
#define CAM_DEPTH_10S (CAM_DEPTH_SIGN|10)
#define CAM_DEPTH_12S (CAM_DEPTH_SIGN|12)
#define CAM_DEPTH_16S (CAM_DEPTH_SIGN|16)
#define CAM_DEPTH_32S (CAM_DEPTH_SIGN|32)

#define CAM_DATA_ORDER_PIXEL  0
#define CAM_DATA_ORDER_PLANE  1

#define CAM_ORIGIN_TL 0

#define CAM_ALIGN_4BYTES   4
#define CAM_ALIGN_8BYTES   8
#define CAM_ALIGN_16BYTES 16
#define CAM_ALIGN_32BYTES 32

#define CAM_ALIGN_DWORD   CAM_ALIGN_4BYTES
#define CAM_ALIGN_QWORD   CAM_ALIGN_8BYTES
 
#define CAM_BORDER_CONSTANT   0
#define CAM_BORDER_REPLICATE  1

/*---  Indexes to access IplImage.BorderMode[],IplImage.BorderConst[]  ----*/
#define CAM_SIDE_TOP_INDEX    0
#define CAM_SIDE_BOTTOM_INDEX 1
#define CAM_SIDE_LEFT_INDEX   2
#define CAM_SIDE_RIGHT_INDEX  3

/*----------  values of argument of iplSetBorderMode(,,border,)  ----------*/
#define CAM_SIDE_TOP        (1<<CAM_SIDE_TOP_INDEX)
#define CAM_SIDE_BOTTOM     (1<<CAM_SIDE_BOTTOM_INDEX)
#define CAM_SIDE_LEFT       (1<<CAM_SIDE_LEFT_INDEX)
#define CAM_SIDE_RIGHT      (1<<CAM_SIDE_RIGHT_INDEX)
#define CAM_SIDE_ALL	    (CAM_SIDE_RIGHT | CAM_SIDE_TOP | CAM_SIDE_LEFT | CAM_SIDE_BOTTOM)

#ifdef CAM_BIG_ENDIAN
#define CAM_FC( ch0, ch1, ch2, ch3 ) \
    ((int)(ch3)|((int)(ch2)<<8)|((int)(ch1)<<16)|((int)(ch0)<<24))
#else
#define CAM_FC( ch0, ch1, ch2, ch3 ) \
    ((int)(ch0)|((int)(ch1)<<8)|((int)(ch2)<<16)|((int)(ch3)<<24))
#endif

#ifndef SWIG
#define CAM_COLORMODEL_RGB   CAM_FC('R','G','B',000)
#define CAM_COLORMODEL_RGBA  CAM_FC('R','G','B','A')
#define CAM_COLORMODEL_YUV   CAM_FC('Y','U','V',000)
#define CAM_COLORMODEL_GREY  CAM_FC('G',000,000,000)
#define CAM_CHANNELSEQ_RGB   CAM_FC('R','G','B',000)
#define CAM_CHANNELSEQ_RGBA  CAM_FC('R','G','B','A')
#define CAM_CHANNELSEQ_YUV   CAM_FC('Y','U','V',000)
#define CAM_CHANNELSEQ_GREY  CAM_FC('G',000,000,000)
#define CAM_CHANNELSEQ_BGR   CAM_FC('B','G','R',000)
#define CAM_CHANNELSEQ_BGRA  CAM_FC('B','G','R','A')
#endif // SWIG

#define CAM_HEADER_ONLY (1<<29)

#ifdef __cplusplus
struct CamImage;
struct CamTableOfBasins;
struct CamBitmapFont;
struct CamKeypoints;
struct CamKeypoint;

/// The IPL Region Of Interest structure
struct CamROI {
#else
/// The IPL Region Of Interest structure
typedef struct {
#endif
    /// The channel of interest number
    /** This parameter indicates which channel in the original image will
     *  be affected by processing taking place in the region of interest;
     *  coi equal to 0 indicates that all channel will be affected
     */
    int             coi;		
    int             xOffset;		///< The offset from the origin of the rectangular ROI.				
    int             yOffset;		///< The offset from the origin of the rectangular ROI.				
    int             width;		///< The size of the rectangular ROI
    int             height;		///< The size of the rectangular ROI
    
    // C++ Wrapping
#ifdef __cplusplus
    CamROI() {coi=0; xOffset=0; yOffset=0; width=0; height=0;}
    CamROI(int _coi, int _xOffset, int _yOffset, int _width, int _height) {
        coi=_coi; xOffset=_xOffset; yOffset=_yOffset; width=_width; height=_height;
    }
    CamROI(const CamImage &image, int _coi=0);
    ~CamROI() {};
    CamROI intersect(const CamROI &roi) const; ///< C++ wrapping for function camROIIntersect()
    bool clip(CamImage &image);
    bool reduce(int i);
    bool enlarge(int i);
};

struct CamLinearFilterKernel;
struct CamMorphoMathsKernel;
struct CamRLEImage;
struct CamTable;
struct CamMeasuresResults;
struct CamSepFilterKernel;
struct CamAffineTransform;

/// The structure used to identify a point in 2D
struct CamPoint {
    int x; ///< x coordinate
    int y; ///< y coordinate
    
    CamPoint apply_affine_transform(const CamAffineTransform &t) const;
    CamPoint(int xp, int yp) {x = xp; y = yp;}
    CamPoint() {x = 0; y = 0;}
};
#else
} CamROI;

/// The structure used to identify a point in 2D
typedef struct {
    int x; ///< x coordinate
    int y; ///< y coordinate
} CamPoint;
#endif

#define CAM_POINT   1
#define CAM_CROSS   2
#define CAM_CIRCLE  4

/// The famous IplImage/CamImage structure, today's standard structure for image processing.
/** This structure originally comes from IPL (Image Processing Library) by Intel.
 *  This standard structure is also used in OpenCV (Open Source Computer Vision Libary).
 *  The IplImage structure is the preferred structure for image processing with RT-Maps
 *  (Real-Time "des Mines" Automotive Prototyping System).
 */
#ifdef __cplusplus
struct CamImage {
#else
typedef struct {
#endif
#ifdef SWIG
%immutable;
#endif
    int             nSize;              ///< Size of CamImage struct
    int             id;                 ///< Frame Id (user dependant)
    int             nChannels;		///< Number of channels in the image (generally 1,3 or 4)
    int             alphaChannel;	///< Alpha channel number (0 if there is no alpha channel in the image)
    
    /// Bit depth of pixels.
    /** Can be one of CAM_DEPTH_1U, CAM_DEPTH_8U, CAM_DEPTH_8S, CAM_DEPTH_10U, CAM_DEPTH_10S, CAM_DEPTH_12U, CAM_DEPTH_12S, CAM_DEPTH_16U, CAM_DEPTH_16S, CAM_DEPTH_32U, CAM_DEPTH_32S
     */
    int             depth;              
    char            colorModel[4];	///< A four-character string describing the color model: "RGB", "GRAY", "HLS", etc.
    
    /// The sequence of color channels
    /** Can be one of the following: "G", "GREY", "BGR", "BGRA", "RGB", "RGBA", "HLS", "YUV"
     */
    char            channelSeq[4];		
    int             dataOrder;		///< CAM_DATA_ORDER_PIXEL or CAM_DATA_ORDER_PLANE
    int             origin;		///< The origin of the image. Can only be CAM_ORIGIN_TL (top left).
    int             align;              ///< Alignment of image data : 4 (CAM_ALIGN_DWORD) or 8 (CAM_ALIGN_QWORD) byte align 
    int             width;		///< Width of the image in pixels
    int             height;		///< Height of the image in pixels
 
#ifdef SWIG
%mutable;
#endif
    /// Pointer to a ROI (Region of interest) structure.
    /** This argument can be NULL, which implies that a region of interest
     *  comprises all channels and the entire image area.
     */
    CamROI         *roi;
#ifdef SWIG
%immutable;
#endif

    /// Pointer to the header of another image that specifies the mask ROI.
    /** This argument can be NULL, which indicates that no mask ROI is used.
     *  A pixel is processed if the corresponding mask pixel is not 0, and is not
     *	processed if the mask pixel is 0. At the moment, only RLE encoded masks
     *	are supported (CamRLEImage structure)
     **/
    void           *mask;		///< Pointer to mask image
    
    /// The image ID
    /** Field reserved for the use of the application to identify the image.
     */
    void           *imageId;            ///< Free for user application
    
    void           *misc;               ///< Free for user application (information on tiling in IplImage structure)
    int             imageSize;          ///< Useful size in bytes
    unsigned char  *imageData;          ///< Pointer to aligned image
    int             widthStep;          ///< The size of aligned line in bytes
    int             borderMode[4];      ///< The top, bottom, left and right border mode
    int             borderConst[4];     ///< Constants for the top, bottom, left and right border
    unsigned char  *imageDataOrigin;    ///< Pointer to full, nonaligned image. A NULL value indicates that the image is not owned by the structure

    // C++ Wrapping

#ifdef __cplusplus
    CamImage() {nSize=sizeof(CamImage); imageData=NULL; roi=NULL; imageSize=0; mask=NULL; imageDataOrigin=NULL; depth=CAM_DEPTH_8U; nChannels=1; } ///< Default non-allocating constructor
    CamImage(int width, int height, int depth=CAM_DEPTH_8U, int channelseq=0); ///< Allocating constructor (1 to 16 bits, signed/unsigned, G/RGB/RGBA/YUV)
    CamImage(const CamImage& image);                    ///< Copy constructor 
    ~CamImage();                                        ///< Default destructor. Free any allocated memory.

#ifndef SWIG
    CamImage& operator=(const CamImage &image);         ///< Operator= redefinition
#endif
    CamImage* clone() const;                            ///< Clone image (using camClone() function)                              
    CamImage* copy() const;                             ///< Copy (not clone). Takes into account ROI and masks. C++ wrapping for camCopy() function
    bool copy(CamImage &dest) const;                    ///< Copy (not clone). Takes into account ROI and masks. C++ wrapping for camCopy() function
    bool allocated() const {return (imageData)?true:false;} ///< Check whether the image is allocated or not
    bool alloc(int width, int height, int depth=CAM_DEPTH_8U, int channelseq=0); ///< Image allocation (1 to 16 bits, signed/unsigned, G/RGB/RGBA/YUV/HLS)
    bool fill_header(int width, int height, int depth=CAM_DEPTH_8U, int channelseq=0); ///< Image header filling (1 to 16 bits, signed/unsigned, G/RGB/RGBA/YUV/HLS)
    bool deallocate();					///< Deallocate image (C++ wrapping for camDeallocateImage() function)
    bool free() {return deallocate();}
    bool alloc_rgb(int width, int height);              ///< RGB image allocation (8 bits, pixel oriented)
    bool alloc_rgba(int width, int height);             ///< RGBA image allocation (8 bits, pixel oriented, alpha channel)
    bool alloc_bgr(int width, int height);              ///< BGR image allocation (8 bits, pixel oriented)
    bool alloc_bgra(int width, int height);             ///< BGRA image allocation (8 bits, pixel oriented, alpha channel)
    bool alloc_yuv(int width, int height);              ///< YUV image allocation (8 bits, planar oriented)
    bool alloc_hls(int width, int height);		///< HLS image allocation (16 bits, planar oriented)
    bool load_pgm(const char *filename);                ///< C++ wrapping for camLoadPGM() function
    bool save_pgm(const char *filename) const;          ///< C++ wrapping for camSavePGM() function
    bool save_raw_pgm(const char *filename) const;	///< C++ wrapping for camSaveRawPGM() function
    bool load_bmp(const char *filename);                ///< C++ wrapping for camLoadPGM() function
    bool save_bmp(const char *filename) const;          ///< C++ wrapping for camSavePGM() function
    bool set_roi(const CamROI &roi);                    ///< C++ wrapping for camSetROI() function
    void get_pixels(char **result, int *len) const;     ///< Fills an char array allocated with C++ operator new. len is filled with the length of the array. Used for mapping Ruby to_s member.
    bool set_pixels(const char *pixels, int sz);        ///< Set pixels of the picture, from a byte array
    void inspect(char **result, int *len) const;        ///< Returns some textual information about the image
    bool view() const;                                  ///< View the picture
    bool set(int color);                                ///< C++ wrapping for camSet() function
    bool alpha_composite(const CamImage& source2, CamImage& dest) const; ///< C++ wrapping for camAlphaComposite() function 

    int erode_square3();                                ///< C++ wrapping for camErodeSquare3() function
    int erode_square3(CamImage &dest) const;            ///< C++ wrapping for camErodeSquare3() function
    int erode_circle5();                                ///< C++ wrapping for camErodeCircle5() function
    int erode_circle5(CamImage &dest) const;            ///< C++ wrapping for camErodeCircle5() function
    int erode_circle7();                                ///< C++ wrapping for camErodeCircle7() function
    int erode_circle7(CamImage &dest) const;            ///< C++ wrapping for camErodeCircle7() function
    int dilate_square3();                               ///< C++ wrapping for camDilateSquare3() function
    int dilate_square3(CamImage &dest) const;           ///< C++ wrapping for camDilateSquare3() function
    int dilate_circle5();                               ///< C++ wrapping for camDilateCircle5() function
    int dilate_circle5(CamImage &dest) const;           ///< C++ wrapping for camDilateCircle5() function
    int dilate_circle7();                               ///< C++ wrapping for camDilateCircle7() function
    int dilate_circle7(CamImage &dest) const;           ///< C++ wrapping for camDilateCircle7() function
    int morpho_gradient_square3();                      ///< C++ wrapping for camMorphoGradientSquare3() function
    int morpho_gradient_square3(CamImage &dest) const;  ///< C++ wrapping for camMorphoGradientSquare3() function
    int morpho_gradient_circle5();                      ///< C++ wrapping for camMorphoGradientCircle5() function
    int morpho_gradient_circle5(CamImage &dest) const;  ///< C++ wrapping for camMorphoGradientCircle5() function
    int morpho_gradient_circle7();                      ///< C++ wrapping for camMorphoGradientCircle7() function
    int morpho_gradient_circle7(CamImage &dest) const;  ///< C++ wrapping for camMorphoGradientCircle7() function

    int morpho_maths(const CamMorphoMathsKernel &ker);                                  ///< C++ wrapping for camMorphoMaths() function
    int morpho_maths(CamImage &dest, const CamMorphoMathsKernel &ker) const;            ///< C++ wrapping for camMorphoMaths() function
    int erode_3x3(const CamMorphoMathsKernel &ker);                                     ///< C++ wrapping for camErode3x3() function
    int erode_3x3(CamImage &dest, const CamMorphoMathsKernel &ker) const;               ///< C++ wrapping for camErode3x3() function
    int dilate_3x3(const CamMorphoMathsKernel &ker);                                    ///< C++ wrapping for camDilate3x3() function
    int dilate_3x3(CamImage &dest, const CamMorphoMathsKernel &ker) const;              ///< C++ wrapping for camDilate3x3() function
    int erode_5x5(const CamMorphoMathsKernel &ker);                                     ///< C++ wrapping for camErode5x5() function
    int erode_5x5(CamImage &dest, const CamMorphoMathsKernel &ker) const;               ///< C++ wrapping for camErode5x5() function
    int dilate_5x5(const CamMorphoMathsKernel &ker);                                    ///< C++ wrapping for camDilate5x5() function
    int dilate_5x5(CamImage &dest, const CamMorphoMathsKernel &ker) const;              ///< C++ wrapping for camDilate5x5() function
    int erode_7x7(const CamMorphoMathsKernel &ker);                                     ///< C++ wrapping for camErode7x7() function
    int erode_7x7(CamImage &dest, const CamMorphoMathsKernel &ker) const;               ///< C++ wrapping for camErode7x7() function
    int dilate_7x7(const CamMorphoMathsKernel &ker);                                    ///< C++ wrapping for camDilate7x7() function
    int dilate_7x7(CamImage &dest, const CamMorphoMathsKernel &ker) const;              ///< C++ wrapping for camDilate7x7() function

    int linear_filter_3x3(const CamLinearFilterKernel &ker);                            ///< C++ wrapping for camLinearFilter3x3() function
    int linear_filter_3x3(CamImage &dest, const CamLinearFilterKernel &k) const;        ///< C++ wrapping for camLinearFilter3x3() function
    int linear_filter_5x5(const CamLinearFilterKernel &ker);                            ///< C++ wrapping for camLinearFilter5x5() function
    int linear_filter_5x5(CamImage &dest, const CamLinearFilterKernel &k) const;        ///< C++ wrapping for camLinearFilter5x5() function
    int linear_filter_abs_3x3(const CamLinearFilterKernel &ker);                        ///< C++ wrapping for camLinearFilterAbs3x3() function
    int linear_filter_abs_3x3(CamImage &dest, const CamLinearFilterKernel &k) const;    ///< C++ wrapping for camLinearFilterAbs3x3() function
    int linear_filter_abs_5x5(const CamLinearFilterKernel &ker);                        ///< C++ wrapping for camLinearFilterAbs5x5() function
    int linear_filter_abs_5x5(CamImage &dest, const CamLinearFilterKernel &k) const;    ///< C++ wrapping for camLinearFilterAbs5x5() function
    bool sobel_v();                                                                     ///< C++ wrapping for camSobelV() function
    bool sobel_h();                                                                     ///< C++ wrapping for camSobelH() function
    bool sobel_v_abs();                                                                 ///< C++ wrapping for camSobelVAbs() function
    bool sobel_h_abs();                                                                 ///< C++ wrapping for camSobelHAbs() function
    bool sobel_v(CamImage &dest) const;                                                 ///< C++ wrapping for camSobelV() function
    bool sobel_h(CamImage &dest) const;                                                 ///< C++ wrapping for camSobelH() function
    bool sobel_v_abs(CamImage &dest) const;                                             ///< C++ wrapping for camSobelVAbs() function
    bool sobel_h_abs(CamImage &dest) const;                                             ///< C++ wrapping for camSobelHAbs() function
    int sep_filter_3x3(const CamSepFilterKernel &ker);                                  ///< C++ wrapping for camSepFilter3x3() function
    int sep_filter_3x3(CamImage &dest, const CamSepFilterKernel &k) const;              ///< C++ wrapping for camSepFilter3x3() function
    int sep_filter_5x5(const CamSepFilterKernel &ker);                                  ///< C++ wrapping for camSepFilter5x5() function
    int sep_filter_5x5(CamImage &dest, const CamSepFilterKernel &k) const;              ///< C++ wrapping for camSepFilter5x5() function
    int sep_filter_7x7(const CamSepFilterKernel &ker);                                  ///< C++ wrapping for camSepFilter7x7() function
    int sep_filter_7x7(CamImage &dest, const CamSepFilterKernel &k) const;              ///< C++ wrapping for camSepFilter7x7() function
    int sep_filter_abs_3x3(const CamSepFilterKernel &ker);                              ///< C++ wrapping for camSepFilterAbs3x3() function
    int sep_filter_abs_3x3(CamImage &dest, const CamSepFilterKernel &k) const;          ///< C++ wrapping for camSepFilterAbs3x3() function
    int sep_filter_abs_5x5(const CamSepFilterKernel &ker);                              ///< C++ wrapping for camSepFilterAbs5x5() function
    int sep_filter_abs_5x5(CamImage &dest, const CamSepFilterKernel &k) const;          ///< C++ wrapping for camSepFilterAbs5x5() function
    int sep_filter_abs_7x7(const CamSepFilterKernel &ker);                              ///< C++ wrapping for camSepFilterAbs7x7() function
    int sep_filter_abs_7x7(CamImage &dest, const CamSepFilterKernel &k) const;          ///< C++ wrapping for camSepFilterAbs7x7() function
    bool fixed_filter(CamImage &dest, int filter) const;                                ///< C++ wrapping for camFixedFilter() function
    bool fixed_filter(int filter);                                                      ///< C++ wrapping for camFixedFilter() function

    bool draw_line(int x1, int y1, int x2, int y2, int color);                          ///< C++ wrapping for camDrawLine() function
    bool accumulate_line(int x1, int y1, int x2, int y2, int acc);                      ///< C++ wrapping for camAccumulateLine() function
    bool draw_rectangle(int x1, int y1, int x2, int y2, int color);                     ///< C++ wrapping for camDrawRectangle() function
    bool draw_text_16s(const char *text, int x, int y, int cwidth, int cheight, int orientation, int color); ///< C++ wrapping for camDrawText16s() function
    bool draw_text_bitmap(const char *text, int x, int y, const CamBitmapFont &font);   ///< C++ wrapping for camDrawTextBitmap() function
    bool draw_circle(int x, int y, int r, int color);                                   ///< C++ wrapping for camDrawCircle() function
    bool draw_ellipse(int x, int y, int rx, int ry, int color);                         ///< C++ wrapping for camDrawEllipse() function
    bool plot(int x, int y, int color, int kind=CAM_POINT);                             ///< C++ wrapping for camPlot() function
    int fill_color(int x, int y, int fillcolor, int tolerance=-1);                      ///< C++ wrapping for camFillColor() function

    bool warping(CamImage &dest, int interpolation_method, bool perspective, const CamPoint &ul, const CamPoint &ur, const CamPoint &lr, const CamPoint &ll) const; ///< C++ wrapping for camWarping() function
    bool scale(CamImage &dest) const;                                                   ///< C++ wrapping for camScale() function
    bool set_mask(const CamRLEImage &mask);                                             ///< C++ wrapping for camSetRLEMask() function
    bool set_mask(const CamImage &mask);                                                ///< C++ wrapping for camSetMask() function
    bool apply_lut(const CamTable &lut);                                                ///< C++ wrapping for camApplyLUT() function
    bool apply_lut(CamImage &dest, const CamTable &lut) const;                          ///< C++ wrapping for camApplyLUT() function

    CamImage *to_yuv() const;                                                           ///< C++ wrapping for camRGB2YUV() function
    bool to_yuv(CamImage &dest) const;                                                  ///< C++ wrapping for camRGB2YUV() function
    CamImage *to_y() const;								///< C++ wrapping for camRGB2Y() function
    bool to_y(CamImage &dest) const;							///< C++ wrapping for camRGB2Y() function
    CamImage *to_rgb() const;                                                           ///< C++ wrapping for camYUV2RGB() function
    bool to_rgb(CamImage &dest) const;                                                  ///< C++ wrapping for camYUV2RGB() function
    CamImage *to_hls() const;                                                           ///< C++ wrapping for camRGB2HLS() function
    bool to_hls(CamImage &dest) const;                                                  ///< C++ wrapping for camRGB2HLS() function

    CamRLEImage* encode() const;                                                        ///< C++ wrapping for camRLEEncode() function
    CamRLEImage* encode_lut(const CamTable &LUT)  const;                                ///< C++ wrapping for camRLEEncodeLUT() function
    CamRLEImage* encode_threshold(int threshold) const;                                 ///< C++ wrapping for camRLEEncodeThreshold() function
    CamRLEImage* encode_threshold_inv(int threshold) const;                             ///< C++ wrapping for camRLEEncodeThresholdInv() function
    CamRLEImage* encode_color(const CamTable &clusters) const;                          ///< C++ wrapping for camRLEEncodeColor() function
    bool encode(CamRLEImage& dest) const;                                               ///< C++ wrapping for camRLEEncode() function
    bool encode_lut(CamRLEImage& dest, const CamTable &LUT)  const;                     ///< C++ wrapping for camRLEEncodeLUT() function
    bool encode_threshold(CamRLEImage& dest, int threshold) const;                      ///< C++ wrapping for camRLEEncodeThreshold() function
    bool encode_threshold_inv(CamRLEImage& dest, int threshold) const;                  ///< C++ wrapping for camRLEEncodeThresholdInv() function

    int threshold(CamImage &dest,int threshold) const;                                  ///< C++ wrapping for camThreshold() function
    int threshold_inv(CamImage &dest,int threshold) const;                              ///< C++ wrapping for camThresholdInv() function
    int abs(CamImage &dest) const;                                                      ///< C++ wrapping for camAbs() function
    int threshold(int threshold);                                                       ///< C++ wrapping for camThreshold() function
    int threshold_inv(int threshold);                                                   ///< C++ wrapping for camThresholdInv() function
    int abs();                                                                          ///< C++ wrapping for camAbs() function

    int arithm(int operation, int c1=0, int c2=0, int c3=0);                            ///< C++ wrapping for camMonadicArithm() function
    int arithm(CamImage& dest, int operation, int c1=0, int c2=0, int c3=0) const;      ///< C++ wrapping for camMonadicArithm() function
    int arithm(const CamImage& source2, CamImage& dest, int operation, int c1=0, int c2=0, int c3=0, int c4=0) const; ///< C++ wrapping for camDyadicArithm() function 

    CamMeasuresResults measures() const;                                                ///< C++ wrapping for camMeasures() function
    float average_deviation(int average=0) const;                                       ///< C++ wrapping for camMeasureAverageDeviation() function

    bool sum_hv(CamTable& hsum, CamTable &vsum) const;                                  ///< C++ wrapping for camSumHV() function
    bool sum_h(CamTable& sum) const;                                                    ///< C++ wrapping for camSumH() function
    bool sum_v(CamTable& sum) const;                                                    ///< C++ wrapping for camSumV() function

    int histogram(CamTable& histo) const;                                               ///< C++ wrapping for camHistogram() function
    bool histogram_equalization(CamImage &dest, const CamTable &src_histo, int option=0, CamImage *work=NULL); /// C++ wrapping for camHistogramEqualizatio() function
    bool histogram_2_channels(int ch1, int ch2, CamImage &result, int size=1) const;    ///< C++ wrapping for camHistogram2Channels() function
    int find_threshold(int percent) const;                                              ///< C++ wrapping for camFindThreshold() function

    int hough_circle(int percent, int rmin, int rmax, int &xc, int &yc, int &rc) const; ///< C++ wrapping for camHoughCircle() function

    int hierarchical_watershed(CamImage &watershed, CamTableOfBasins &tob) const;       ///< C++ wrapping for camHierarchicalWatershed() function
    int hierarchical_watershed_contours(CamImage &ws, CamTableOfBasins &tob) const;     ///< C++ wrapping for camHierarchicalWatershedContours() function
    int hierarchical_watershed_regions(const CamTableOfBasins &tob);                    ///< C++ wrapping for camHierarchicalWatershedRegions() function

    bool draw_keypoints(const CamKeypoints &points, int color = 255);		///< C++ wrapping for camDrawKeypoints() function
    bool draw_keypoint(const CamKeypoint &point, int color = 255);		///< C++ wrapping for camDrawKeypoint() function
    bool harris(CamKeypoints &points, int k = 41) const;				///< C++ wrapping for camHarris() function
    bool integral_image(CamImage &dest) const;						///< C++ wrapping for camIntegralImage() function	
    CamImage *integral_image() const;							///< C++ wrapping for camIntegralImage() function	
    bool fast_hessian_detector(CamKeypoints &points, int threshold, int options = 0) const; /// C++ wrapping for camFastHessianDetector() function
};

inline CamROI::CamROI(const CamImage &image, int _coi) { coi=_coi; xOffset=0; yOffset=0; width=image.width; height=image.height; }

#else
} CamImage;
#endif

/* Camellia C functions headers
 */

#ifndef SWIG

#define CAM_PIXEL_ACCESS(ptr,y,x) \
    ((CAM_PIXEL*)((char*)ptr+y*ptr##widthstep)+x)

#define CAM_MAX(a,b) (((a)>(b))?(a):(b))
#define CAM_MIN(a,b) (((a)<(b))?(a):(b))

/*! \def CAM_RGBA
 *	\brief 32 bits integer color representation, including an alpha channel.
 */
#define CAM_RGBA(r,g,b,a) ((r)|((g)<<8)|((b)<<16)|((a)<<24))

/*! \def CAM_RGB
 *	\brief 24 bits integer color representation. Stricly equivalent to the Windows RGB macro.
 *		   Please use this one in place of RGB for better portability of the code.
 */
#define CAM_RGB(r,g,b) ((r)|((g)<<8)|((b)<<16))

#endif // SWIG

/* General purpose structures
 */

/* Monadic and Dyadic Arithmetic Operators kernel
 */

#define CAM_ARITHM_ABS		    0
#define CAM_ARITHM_INVERSE	    1
#define CAM_ARITHM_SELECT	    2
#define CAM_ARITHM_THRESHOLD	    3 
#define CAM_ARITHM_DOUBLE_THRESHOLD 4

#define CAM_ARITHM_ADD		    0
#define CAM_ARITHM_SUM		    0
#define CAM_ARITHM_SUB		    1
#define CAM_ARITHM_MUL              2
#define CAM_ARITHM_ABSDIFF	    3
#define CAM_ARITHM_WEIGHTED_SUM	    4
#define CAM_ARITHM_INF		    5
#define CAM_ARITHM_SUP		    6
#define CAM_ARITHM_COMP_INF	    7
#define CAM_ARITHM_COMP_EQUAL	    8
#define CAM_ARITHM_COMP_SUP	    9
#define CAM_ARITHM_AND		    10
#define CAM_ARITHM_OR		    11

// Shorter constants
#define CAM_ABS		            0
#define CAM_INVERSE	            1
#define CAM_SELECT	            2
#define CAM_THRESHOLD	            3 
#define CAM_DOUBLE_THRESHOLD        4

#define CAM_ADD		            0
#define CAM_SUM		            0
#define CAM_SUB		            1
#define CAM_ABSDIFF	            2
#define CAM_WEIGHTED_SUM	    3
#define CAM_INF		            4
#define CAM_SUP		            5
#define CAM_COMP_INF	            6
#define CAM_COMP_EQUAL	            7
#define CAM_COMP_SUP	            8
#define CAM_AND		            9
#define CAM_OR		            10

#ifndef SWIG

/// This is the parameter structure for the arithmetic computation kernel
/** Basically, it includes the code of the operation to execute, and
 *  the different parameters for this operation.
 */
typedef struct {
    int operation;   ///< Operation to apply to the sources
    int c1; ///< Additional parameter (see camMonadicArithm() and camDyadicArithm() for details)
    int c2; ///< Additional parameter (see camMonadicArithm() and camDyadicArithm() for details)
    int c3; ///< Additional parameter (see camMonadicArithm() and camDyadicArithm() for details)
    int c4; ///< Additional parameter (see camMonadicArithm() and camDyadicArithm() for details)
} CamArithmParams;

#endif // SWIG

#ifdef __cplusplus
extern "C" {
#endif

#ifndef SWIG

/// This is the function that implements all the monadic arithmetic operators (i.e. operators with only one operand)
/** This function can be used to compute the absolute value of an image (<DFN>CAM_ARITHM_ABS</DFN>),
 *  to invert an image (<DFN>CAM_ARITHM_INVERSE</DFN>), to select all pixels with a given value in an image (<DFN>CAM_ARITHM_SELECT</DFN>),
 *  to threshold an image either with one threshold value (<DFN>CAM_ARITHM_THRESHOLD</DFN>), or two threshold values (<DFN>CAM_ARITHM_DOUBLE_THRESHOLD</DFN>)
 *
 *  \param source   The source ::CamImage to process. The source must be a grey-level image,
 *		    except when the operation is an image inversion (<DFN>CAM_ARITHM_INVERSE</DFN>).
 *  \param dest	    The destination ::CamImage. The destination image can be either a grey-level
 *		    image or a binary image. In the latter case, <DFN>CAM_ARITHM_ABS</DFN> is not a valid
 *		    operation
 *  \param params   A pointer to a ::CamArithmParams structure, defining all the parameters 
 *		    of the function to apply to the source image.
 *
 *  Here are the details on the operations and parameters to set in the ::CamArithmParams structure :
 *  - <DFN>CAM_ARITHM_ABS</DFN> : Absolute value computation. <I>pdest=abs(psource)</I>.
 *  - <DFN>CAM_ARITHM_INVERSE</DFN> : Inverse. <I>pdest=not(psource)</I>.
 *  - <DFN>CAM_ARITHM_SELECT</DFN> : Selection of a part of an image. <I>if (psource=c1) then pdest=c2 else pdest=c3</I>.
 *  - <DFN>CAM_ARITHM_THRESHOLD</DFN> : Comparison with a scalar (also called thresholding). <I>if (psource<c1) then pdest=c2 else pdest=c3</I>.
 *  - <DFN>CAM_ARITHM_DOUBLE_THRESHOLD</DFN> : Double thresholding. <I>if (psource>c1 and psource<c2) then pdest=c3 else pdest=c4</I>.
 *
 *  \return 0 (false) if an error occurs. An accumulator of all results pixel values otherwise.
 *
 *  Note that this kernel supports in-place processing (i.e. dest can be the same as source param).
 *
 */
int camMonadicArithm(CamImage *source, CamImage *dest, CamArithmParams *params);

/// This is the function that implements all the dyadic arithmetic operators (i.e. operators with two operands)
/** This function can be used to compute the sum of two images (<DFN>CAM_ARITHM_ADD</DFN>),
 *  to substract two images (<DFN>CAM_ARITHM_SUB</DFN>), to compute the absolute difference 
 *  between two images (<DFN>CAM_ARITHM_ABSDIFF</DFN>), to compute the weighted sum of two images (<DFN>CAM_ARITHM_WEIGHTED_SUM</DFN>),
 *  etc. (see below)
 *
 *  \param source1  The first source ::CamImage to process. Any source image can be either grey-level or
 *		    binary, depending on the operation to apply.
 *
 *  \param source2  The second source ::CamImage to process. Any source image can be either grey-level or
 *		    binary, depending on the operation to apply.
 *
 *  \param dest	    The destination ::CamImage. The destination image can be either a grey-level
 *		    image or a binary image, but source1, source2 and dest must be of the same kind.
 *
 *  \param params   A pointer to a ::CamArithmParams structure, defining all the parameters 
 *		    of the function to apply to the source images.
 *
 *  Here are the details on the operations and parameters to set in the ::CamArithmParams structure :
 *  - <DFN>CAM_ARITHM_ADD</DFN> : Addition of images, with saturation. <I>pdest=psource1+psource2; if (pdest<0) then pdest=0; if (pdest>valmax) then pdest=valmax</I>.
 *  - <DFN>CAM_ARITHM_SUB</DFN> : Substraction of images, with saturation. <I>pdest=psource1-psource2; if (pdest<valmin) then pdest=valmin; if (pdest>valmax) then pdest=valmax</I>.
 *  - <DFN>CAM_ARITHM_MUL</DFN> : Multiplication of images, with saturation. <I>pdest=(psource1*psource2)>>c1; if (pdest<valmin) then pdest=valmin; if (pdest>valmax) then pdest=valmax</I>.
 *  - <DFN>CAM_ARITHM_ABSDIFF</DFN> : Absolute difference, with saturation. <I>pdest=abs(psource1-psource2)</I>.
 *  - <DFN>CAM_ARITHM_WEIGHTED_SUM</DFN> : Weighted sum. <I>pdest=(c1*psource1+c2*psource2)>>c3</I>.
 *  - <DFN>CAM_ARITHM_INF</DFN> : Inf. <I>pdest=inf(psource1,psource2)</I>.
 *  - <DFN>CAM_ARITHM_SUP</DFN> : Sup. <I>pdest=sup(psource1,psource2)</I>.
 *  - <DFN>CAM_ARITHM_COMP_INF</DFN> : Comparison. <I>if (psource1<psource2) pdest=c1 else pdest=c2</I>.
 *  - <DFN>CAM_ARITHM_COMP_EQUAL</DFN> : Equality test. <I>if (psource1=psource2) pdest=c1 else pdest=c2</I>.
 *  - <DFN>CAM_ARITHM_COMP_SUP</DFN> : Comparison. <I>if (psource1>psource2) pdest=c1 else pdest=c2</I>.
 *  - <DFN>CAM_ARITHM_AND</DFN> : <I>pdest=psource1 & psource2</I> (arithmetic boolean "and" operator)
 *  - <DFN>CAM_ARITHM_OR</DFN> : <I>pdest=psource1 | psource2</I> (arithmetic boolean "or" operator)
 *
 *  \return 0 (false) if an error occurs. An accumulator of all results pixel values otherwise.
 *
 *  Note that this kernel supports in-place processing (i.e. dest can be the same as one of the sources).
 *
 *  Note also that this kernel supports signed images, and thus can produce signed results
 *  (see the member depth in ::CamImage structure).
 *
 */
int camDyadicArithm(CamImage *source1, CamImage *source2, CamImage *dest, CamArithmParams *params);

/// Image addition
/** Simple wrapper for camDyadicArithm() */
int camAdd(CamImage *source1, CamImage *source2, CamImage *dest);

/// Image multiplication 
/** Simple wrapper for camDyadicArithm() */
int camMul(CamImage *source1, CamImage *source2, CamImage *dest);

/// Image subtraction 
/** Simple wrapper for camDyadicArithm() */
int camSub(CamImage *source1, CamImage *source2, CamImage *dest);

/// Simple threshold function (wrapper for camMonadicArithm() function)
/** Turn all pixels below (<) the threshold value to 0, and all pixels above (>=) to 255
 */
int camThreshold(CamImage *source, CamImage *dest, int threshold);

/// Simple threshold function (wrapper for camMonadicArithm() function)
/** Turn all pixels below (<) the threshold value to 255, and all pixels above (>=) to 0
 */
int camThresholdInv(CamImage *source, CamImage *dest, int threshold);

/// Compute absolute value of image (for signed images) (wrapper for camMonadicArithm() function)
int camAbs(CamImage *source, CamImage *dest);

/* Apply-a-LUT-on-image Kernel
 */

// 12 bits maximum LUT management
#define CAM_TABLE_SIZE 4096

#define CamLUT CamTable
#define CamHisto CamTable

#ifdef __cplusplus
/// Pixel table (LUT (Look-Up Table) and histogram) structure
struct CamTable {
#else
/// LUT (Look-Up Table) structure
typedef struct {
#endif
    int t[CAM_TABLE_SIZE]; ///< Table containing the resulting pixel value for each input
    int size;              ///< Number of valid entries
#ifdef __cplusplus    
    CamTable(int s=0) {size=s;}
    int &operator[](int n);
    bool set(const int* const array, int sz) { if (sz<CAM_TABLE_SIZE) { size=sz; for (int i=0;i<sz;i++) t[i]=array[i]; return true;} return false;} 
};
#else
} CamTable;
#endif

/// Apply a Look-Up-Table on image function
int camApplyLUT(CamImage *source, CamImage *dest, CamTable *LUT);

#endif // SWIG

/* Fundamental Morphological Mathematics Algorithms' kernel
 */
#define CAM_MM_NEIGHB		7

#define CAM_MM_DILATED		0
#define CAM_MM_ERODED		1
#define CAM_MM_ORIGINAL		2

#define CAM_MM_SUBSTRACTION	0
#define CAM_MM_MULTIPLEX	1
#define CAM_MM_THINNING		2
#define CAM_MM_THICKENING	3

/// This is the parameter structure for the morpho maths kernel
/** Basically, it includes the structural elements for dilation and erosion
 *  as well as options for the full kernel.
 */
#ifdef __cplusplus
struct CamMorphoMathsKernel {
#else
typedef struct {
#endif
    // Structural elements
#ifndef SWIG
    int dilationStructElt[CAM_MM_NEIGHB][CAM_MM_NEIGHB]; ///< The structural element used for all dilation operations
    int erosionStructElt[CAM_MM_NEIGHB][CAM_MM_NEIGHB];  ///< The structural element used for all erosion operations
#endif
    int source1;	    ///< CAM_MM_DILATED | CAM_MM_ERODED | CAM_MM_ORIGINAL
    int source2;	    ///< CAM_MM_DILATED | CAM_MM_ERODED | CAM_MM_ORIGINAL
    int operation;	    ///< CAM_MM_SUBSTRACTION | CAM_MM_MULTIPLEX | CAM_MM_THINNING | CAM_MM_THICKENING
#ifdef __cplusplus
    /// Default constructor
    CamMorphoMathsKernel() {
        for (int i=0;i<CAM_MM_NEIGHB;i++) {
            for (int j=0;j<CAM_MM_NEIGHB;j++) {
                dilationStructElt[i][j]=0;
                erosionStructElt[i][j]=0;
            }
        }
        source1=CAM_MM_ORIGINAL;
        source2=CAM_MM_ORIGINAL;
        operation=CAM_MM_MULTIPLEX;
    }       
    /// Set an element of the dilation kernel
    bool set_dilate(int x, int y, int val) {
        if ((x>=0)&&(x<CAM_MM_NEIGHB)&&(y>=0)&&(y<CAM_MM_NEIGHB)) {
            dilationStructElt[x][y]=val; return true;
        } else return false;
    }
    /// Get an element from the dilation kernel
    int get_dilate(int x,int y) {
        if ((x>=0)&&(x<CAM_MM_NEIGHB)&&(y>=0)&&(y<CAM_MM_NEIGHB)) {
            return dilationStructElt[x][y];
        } else return 0;
    }
    /// Set an element of the erosion kernel
    bool set_erode(int x, int y, int val) {
        if ((x>=0)&&(x<CAM_MM_NEIGHB)&&(y>=0)&&(y<CAM_MM_NEIGHB)) {
            erosionStructElt[x][y]=val; return true;
        } else return false;
    }
    /// Get an element from the erosion kernel
    int get_erode(int x,int y) {
        if ((x>=0)&&(x<CAM_MM_NEIGHB)&&(y>=0)&&(y<CAM_MM_NEIGHB)) {
            return erosionStructElt[x][y];
        } else return 0;
    }
};
#else
} CamMorphoMathsKernel;
#endif

#ifndef SWIG

/** @name Morphomaths LLAs
 *  These functions share the same morpho maths kernel 
 */
//@{

/// This is the function that can compute erosion AND dilation in one scan (with 5x5 neighborhood).
/** It can be used to compute thinning, thickening, or morphological gradient in one scan.
 *  If only erosion or dilation is needed, it's more simple (and a bit faster) to use the
 *  dedicated functions.
 *
 *  \param source   The source ::CamImage to process
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamMorphoMathsKernel structure, defining the structural
 *		    elements, as well as the behaviour of the kernel.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this kernel supports in-place processing (i.e. dest can be the same as source param)
 */
int camMorphoMaths(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel);

// These are specific functions able to compute only erosion or dilation, on 3x3, 5x5 or 7x7 neighbourhood

/** \param source   The source ::CamImage to process
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamMorphoMathsKernel structure, defining the structural
 *		    element to use. Only the ErosionStructElt member is used by this function.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camErode3x3(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel); ///< 3x3 neighborhood erosion

/** \param source   The source ::CamImage to process
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamMorphoMathsKernel structure, defining the structural
 *		    element to use. Only the ErosionStructElt member is used by this function.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camErode5x5(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel); ///< 5x5 neighborhood erosion

/** \param source   The source ::CamImage to process
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamMorphoMathsKernel structure, defining the structural
 *		    element to use. Only the ErosionStructElt member is used by this function.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camErode7x7(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel); ///< 7x7 neighborhood erosion

/** \param source   The source ::CamImage to process
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamMorphoMathsKernel structure, defining the structural
 *		    element to use. Only the DilationStructElt member is used by this function.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camDilate3x3(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel); ///< 3x3 neighborhood dilation

/** \param source   The source ::CamImage to process
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamMorphoMathsKernel structure, defining the structural
 *		    element to use. Only the DilationStructElt member is used by this function.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camDilate5x5(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel); ///< 5x5 neighborhood dilation

/** \param source   The source ::CamImage to process
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamMorphoMathsKernel structure, defining the structural
 *		    element to use. Only the DilationStructElt member is used by this function.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camDilate7x7(CamImage *source, CamImage *dest, CamMorphoMathsKernel *kernel); ///< 7x7 neighborhood dilation

/** Computes the morphological gradient of an image.
 *  
 *  Uses a circular structural element of diameter 5.
 *  It is twice faster than the original morpho maths kernel.
 *
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camMorphoGradientCircle5(CamImage *source, CamImage *dest); ///< Morphological gradient computation (Diameter-5 circle structural element)

/** Computes the eroded image of a source image, using a circular structural
 *  element of diameter 5. Highly optimized code.
 *  
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camErodeCircle5(CamImage *source, CamImage *dest); ///< Erosion (Optimized for diameter-5 circle structural element)

/** Computes the dilated image of a source image, using a circular structural
 *  element of diameter 5. Highly optimized code.
 *  
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camDilateCircle5(CamImage *source, CamImage *dest);  ///< Dilation (Optimized for diameter-5 circle structural element)

/** Computes the morphological gradient of an image.
 *  
 *  Uses a circular structural element of diameter 7.
 *  It is twice faster than the original morpho maths kernel.
 *
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camMorphoGradientCircle7(CamImage *source, CamImage *dest); ///< Morphological gradient computation (Diameter-7 circle structural element)

/** Computes the eroded image of a source image, using a circular structural
 *  element of diameter 7. Highly optimized code.
 *  
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camErodeCircle7(CamImage *source, CamImage *dest); ///< Erosion (Optimized for diameter-7 circle structural element)

/** Computes the dilated image of a source image, using a circular structural
 *  element of diameter 7. Highly optimized code.
 *  
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camDilateCircle7(CamImage *source, CamImage *dest);  ///< Dilation (Optimized for diameter-7 circle structural element)

/** Computes the morphological gradient of an image.
 *  
 *  Uses a 3x3 square structural element (very classical).
 *  Highly optimized code.
 *
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camMorphoGradientSquare3(CamImage *source, CamImage *dest); ///< Morphological gradient computation (3x3 square structural element)

/** Computes the eroded image of a source image, using a classical 3x3 square structural
 *  element. Highly optimized code.
 *  
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camErodeSquare3(CamImage *source, CamImage *dest); ///< Erosion (3x3 square structural element)

/** Computes the dilated image of a source image, using a classical 3x3 square structural
 *  element. Highly optimized code.
 *  
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage. Must be a grey-level image.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camDilateSquare3(CamImage *source, CamImage *dest); ///< Dilation (3x3 square structural element)
//@}

/* Labeling kernel
 */
#define CamLabellingResults CamLabelingResults
#define camLabelling camLabeling
#define camLabelling2ndScan camLabeling2ndScan
#define camRLELabelling camRLELabeling

#define CAM_LABEL_MAX_BLOBS 1024
#define CAM_LABEL_PIXEL unsigned short

#define CAM_LABEL_PIXEL_ACCESS(ptr,y,x) \
    ((CAM_LABEL_PIXEL*)((char*)ptr+y*ptr##widthstep)+x)

/// Data structure containing the result of pixel-based labeling.
typedef struct {
    int nbLabels;		    ///< Number of labels found in the frame
    int equiv[CAM_LABEL_MAX_BLOBS]; ///< Labels equivalence table (see D3.1 for details)
} CamLabelingResults;

/// 4-connectedness labeling function
/** Computes the labeled image of a source image,
 *  i.e. finds the connected pixels (using 4-connectedness)
 *  
 *  \param source   The source ::CamImage to process. Must be a grey-level image.
 *  \param dest	    The destination ::CamImage..Must be 16-bits deep (its the label result image)
 *  \param results  The ::CamLabelingResults containing the label equivalence table, to provide to a blob analysis function.
 *  \return	    0 (false) if an error occurs
 *
 *  Note that this function is rather obsolete. This pixel-based labeling algorithm is
 *  outdated compared to RLE-based labeling. 
 *
 *  \sa camRLELabeling
 */
int camLabeling(CamImage *source, CamImage *dest, CamLabelingResults *results); 
										  
/** This algorithm is useless. Presented here for better understanding purpose only.
 *  Indeed, Blob analysis first scan (camBlobAnalysis1stScan()) integrates this second scan.
 *  In-place processing only.
 */
int camLabeling2ndScan(CamImage *image, CamLabelingResults *results); ///< Second scan for pixel-based labeling. Obsolete.

/* Blob analysis Kernel
 * C code */

#endif // SWIG

#define CAM_RLE_INT_TYPE unsigned short

/// The CamRun structure, basic element of a Run-Length Encoding (RLE) of an image.
/** sizeof(CamRun) is 8 (64 bits)
 */
typedef struct {
    CAM_RLE_INT_TYPE value;	///< Which color(s) this run represents
    CAM_RLE_INT_TYPE length;    ///< The length of the run (in pixels)
    CAM_RLE_INT_TYPE blob;	///< Run's parent in the connected components tree, which becomes the blob number after labeling
    CAM_RLE_INT_TYPE line;	///< The line to which the run belongs
} CamRun;

#ifndef SWIG

/// The Blob (block of bits) Information Structure. This is the most important result of Labeling + Blob analysis.
typedef struct {
    int id;
    int left;		///< Leftmost coordinate of the blob
    int top;		///< Topmost coordinate of the blob
    int width;		///< Width of the blob
    int height;		///< Height of the blob
    int surface;	///< Number of pixels covered by this blob
    int cx;		///< Center of gravity (x)
    int cy;		///< Center of gravity (y)
    int value;	        ///< Blob value, or average pixel value in the original image
    int min;		///< Minimum pixel value in the original image
    int max;		///< Maximum pixel value in the original image
    CamRun *first;	///< First run of the blob (only for RLE labeling)
    CamRun *last;	///< Last run of the blob (only for RLE labeling)
    void *misc;		///< Additional user-dependant blob information
} CamBlobInfo;

#define CamBlobAnalysisResults CamBlobs // For compatibility with previous versions

#ifdef __cplusplus
/// The result of any blob analysis. Essentially an array of ::CamBlobInfo
struct CamBlobs {
    int nbBlobs;	                        ///< Number of valid blobs
    CamBlobInfo blobInfo[CAM_LABEL_MAX_BLOBS];  ///< Array of information on the blobs
    CamBlobs() {nbBlobs=0;}                     ///< Default constructor
    CamBlobInfo& operator[](int index);
};
#else
/// The result of any blob analysis. Essentially an array of ::CamBlobInfo
typedef struct {
    int nbBlobs;	                        ///< Number of valid blobs
    CamBlobInfo blobInfo[CAM_LABEL_MAX_BLOBS];  ///< Array of information on the blobs
} CamBlobs;
#endif

/** Computes the most important blob information :
 *  <DFN>top</DFN>, <DFN>left</DFN>, <DFN>width</DFN>, <DFN>height</DFN>, <DFN>cx</DFN>, <DFN>cy</DFN>,
 *  and <DFN>surface</DFN>. <DFN>average</DFN>, <DFN>min</DFN> and <DFN>max</DFN> 
 *  are computed only if a pointer on the original image is provided (slower).
 *
 *  \param blobImage The result of a previous labeling operation.
 *  \param original  The original ::CamImage that was labelled previously. Can bet set to NULL.
 *  \param info	     The ::CamLabelingResults structure provided by the former call to camLabeling().(in data)
 *  \param results   The ::CamBlobs structure that is filled with the collected blob information.
 *		    
 *  \return	    0 (false) if an error occurs, especially if the number of blobs is too high.(more than the <DFN>CAM_LABEL_MAX_BLOBS</DFN> constant)
 *
 *  In-place processing (<DFN>blobImage</DFN> will be affected).
 */
int camBlobAnalysis1stScan(CamImage *blobImage, CamImage *original, CamLabelingResults *info, CamBlobs *results); ///< Blob analysis function.

/// Second pass, to get some more information if needed.
/** Computes the <DFN>average</DFN>, <DFN>min</DFN> and <DFN>max</DFN> blob information, if it was
 *  not computed before.
 *
 *  \param blobImage The result of a previous labeling operation.
 *  \param original  The original ::CamImage.that was labelled previously.
 *  \param results   The ::CamBlobs structure that is filled with the collected blob information.
 *		    
 *  \return	    0 (false) if an error occurs.
 */
int camBlobAnalysisRefinement(CamImage *blobImage, CamImage *original, CamBlobs *results);

/* RLE Labeling kernel
 * New : v1.4 of LLAs
 * Updated v1.6 and v1.9, v2.0 of LLAs
 */

/** @name RLE images processing functions and data structures
 */
//@{

#ifdef __cplusplus
/// The CamRLEImage structure : the RLE (Run Length Encoded) image structure.
struct CamRLEImage {
#else
/// The CamRLEImage structure : the RLE (Run Length Encoded) image structure.
typedef struct {
#endif
    int nSize;          ///< Size of CamImage struct
    int id;             ///< Frame id (user dependent)
    int height;		///< Image height
    int width;		///< Image width
    int nbRuns;		///< The number of runs
    int allocated;	///< Number of runs allocated
    CamRun *runs;	///< A pointer to the array of runs

#ifdef __cplusplus
    CamRLEImage() {nSize=sizeof(CamRLEImage); allocated=0; runs=NULL; nbRuns=0;} ///< Default constructor
    CamRLEImage(int nbruns);                            ///< Constructor with max number of runs parameter
    CamRLEImage(const CamRLEImage &image);              ///< Copy constructor
    ~CamRLEImage();                                     ///< Default destructor

    CamRLEImage& operator=(const CamRLEImage &image);   ///< Operator= redefinition
    CamRLEImage* clone() const;                         ///< Clone RLE image (using camRLEClone() function)
    bool alloc(int nbruns);                             ///< Allocator (C++ wrapping of camRLEAllocate() function)
    bool realloc(int nbruns);                           ///< Reallocator (C++ wrapping of camRLEReallocate() function)

    bool encode(const CamImage &image);                             ///< C++ wrapping for camRLEEncode() function
    bool encode_lut(const CamImage &image, const CamTable &LUT);    ///< C++ wrapping for camRLEEncodeLUT() function
    bool encode_threshold(const CamImage &image, int threshold);    ///< C++ wrapping for camRLEEncodeThreshold() function
    bool encode_threshold_inv(const CamImage &image, int threshold);///< C++ wrapping for camRLEEncodeThresholdInv() function
    bool encode_color(const CamImage &image, const CamTable &clusters); ///< C++ wrapping for camRLEEncodeColor() function
    CamBlobs* labeling();                                           ///< C++ wrapping for camRLELabeling() function
    bool labeling(CamBlobs &results);                               ///< C++ wrapping for camRLELabeling() function
    bool blob_analysis(CamBlobs &results) const;                    ///< C++ wrapping for camRLEBlobAnalysis() function
    bool apply_lut(const CamTable &LUT);                            ///< C++ wrapping for camRLEApplyLUT() function
    bool apply_lut(CamRLEImage &dest, const CamTable &LUT) const;   ///< C++ wrapping for camRLEApplyLUT() function
    bool decode(CamImage &dest) const;                              ///< C++ wrapping for camRLEDecode() function            
    bool decode(CamImage &dest, const CamTable &LUT) const;         ///< C++ wrapping for camRLEDecode() function
    bool decode_blobs(CamImage &dest) const;                        ///< C++ wrapping for camRLEDecodeBlobs() function            
    bool decode_blobs(CamImage &dest, const CamTable &LUT) const;   ///< C++ wrapping for camRLEDecodeBlos() function
    bool inverse();                                                 ///< C++ wrapping for camRLEInverse() function
    bool erode_cross(CamRLEImage &dest) const;                      ///< C++ wrapping for camRLEErodeCross() function 
    CamRLEImage *erode_cross() const;                               ///< C++ wrapping for camRLEErodeCross() function 
    bool erode_3x3(CamRLEImage &dest) const;                        ///< C++ wrapping for camRLEErode3x3() function 
    CamRLEImage *erode_3x3() const;                                 ///< C++ wrapping for camRLEErode3x3() function 
    bool erode_3x2(CamRLEImage &dest) const;                        ///< C++ wrapping for camRLEErode3x2() function 
    CamRLEImage *erode_3x2() const;                                 ///< C++ wrapping for camRLEErode3x2() function 
};
#else
} CamRLEImage;
#endif

/// RLE Image allocation.
/** Allocates a RLE image.
 *  
 *  \param rle	    The ::CamRLEImage to allocate. The number of runs should be chosen
 *		    so that it is enough to encode the image to process. A typical value
 *		    used for this parameter is <DFN>source.width*source.height/16</DFN>, assuming
 *		    a typical run will of length 16. Note that this is very realistic when
 *		    the original image was thresholded and filtered.
 *  \param max_runs The number of runs to allocate.
 *  \return	    0 (false) if an error occurs.
 *
 *  Note that this function uses the standard C <DFN>malloc()</DFN> function.
 *
 *  Note also that the amount of memory allocated is <DFN>8*max_runs</DFN> bytes. Thus, using 
 *  the typical <DFN>max_runs</DFN> value exposed above, a grey-level image will be compressed
 *  by a factor of 2, and a binary image expanded by a factor of 4 using RLE encoding.
 *
 *  The RLE image should be deallocated using the camRLEDeallocate() function,
 *  or by calling the standard C function <DFN>free()</DFN> on the
 *  <DFN>runs</DFN> member of the ::CamRLEImage structure.
 *
 */
int camRLEAllocate(CamRLEImage *rle, int max_runs);

/// RLE image deallocation.
/** Release a RLE image memory. Should be matched with a call to camRLEAllocate()
 *  
 *  \param rle	    The ::CamRLEImage to deallocate.
 *  \return	    0 (false) if an error occurs.
 *
 *  Note that this function uses the standard C <DFN>free()</DFN> function.
 *
 */
int camRLEDeallocate(CamRLEImage *rle);

/// Alias for camRLEDeallocate() function
int camRLEFree(CamRLEImage *rle);

/// RLE image reallocation
/** Reallocates a RLE image.
 *  
 *  \param rle	    The ::CamRLEImage to reallocate.
 *  \param new_max_runs The new number of allocated runs
 *  \return	    0 (false) if an error occurs.
 *
 *  Note that this function uses the standard C <DFN>realloc()</DFN> function.
 *
 */
int camRLEReallocate(CamRLEImage *rle, int new_max_runs);

/// RLE image cloning
/** Clone a RLE image.
 *  
 *  \param source   The ::CamRLEImage to clone.
 *  \param dest     The ::CamRLEImage to allocate (shouldn't be allocated)
 *  \return	    0 (false) if an error occurs.
 *
 *  Note that this function uses the standard C <DFN>malloc()</DFN> function.
 */
int camRLEClone(CamRLEImage *source, CamRLEImage *dest);

/// Run-Length encoding
/** 
 *  \param src	    The source ::CamImage to encode. This should be either a thresholded
 *		    image or a binary image. Note that this source image should be filtered
 *		    (using camErode3x3() for instance) before encoding,
 *		    in order to avoid a too high number of runs.
 *  \param dest	    The ::CamRLEImage resulting of the encoding.
 *  \return	    0 (false) if an error occurs.
 *
 * Note that binary images encoding was optimized.
 */
int camRLEEncode(CamImage *src, CamRLEImage *dest);

/// Run-Length encoding, with integrated LUT operations. 
/** Introduced v1.9.
 *  
 *  \param src	    The source ::CamImage to encode. This must be a grey-level image.
 *		    Note that this source image should be filtered
 *		    (using camErode3x3() for instance) before encoding,
 *		    in order to avoid a too high number of runs.
 *  \param dest	    The ::CamRLEImage resulting of the encoding.
 *  \param LUT	    An array of integer used to classify the source image pixels.
 *  \return	    0 (false) if an error occurs.
 */
int camRLEEncodeLUT(CamImage *src, CamRLEImage *dest, CamTable *LUT);

/// Run-Length encoding, with integrated thresholding. 
/** \param src	    The source ::CamImage to encode. This must be a grey-level image.
 *		    Note that this source image should be filtered
 *		    (using camErodeSquare3() for instance) before encoding,
 *		    in order to avoid a too high number of runs.
 *  \param dest	    The ::CamRLEImage resulting of the encoding.
 *  \param threshold The threshold used. All pixels with value higher (>=) than threshold will be labelled..
 *  \return	    0 (false) if an error occurs.
 */
int camRLEEncodeThreshold(CamImage *src, CamRLEImage *dest, int threshold);

/// Run-Length encoding, with integrated thresholding. 
/** \param src	    The source ::CamImage to encode. This must be a grey-level image.
 *		    Note that this source image should be filtered
 *		    (using camErodeSquare3() for instance) before encoding,
 *		    in order to avoid a too high number of runs.
 *  \param dest	    The ::CamRLEImage resulting of the encoding.
 *  \param threshold The threshold used. All pixels with value lower (>=) than threshold will be labelled..
 *  \return	    0 (false) if an error occurs.
 */
int camRLEEncodeThresholdInv(CamImage *src, CamRLEImage *dest, int threshold);

int camRLEEncodeColor(CamImage *source, CamRLEImage *dest, CamTable *clusters);

/// RLE image labeling + blob analysis. 4-connectedness labeling.
/** Very fast labeling algorithm originally introduced by the Carneggie Mellon University (see below).
 *  This function also performs a basic blob analysis
 *  
 *  \param src	    The source ::CamRLEImage to label. Note that the content of <DFN>src</DFN> is altered,
 *		    for RLE labeling is performed in-place. This is what makes it especially fast,
 *		    since it doesn't require neither an additional label image nor an equivalence
 *		    table, everything being stored in the ::CamRLEImage structure.
 *  \param results  The ::CamBlobs containing the results of the blob analysis.
 *  \return	    0 (false) if an error occurs. Generally when the number
 *		    of blobs exceeds the <DFN>CAM_LABEL_MAX_BLOBS</DFN> constant.
 *
 * This code is based on the ideas developped in the CMVision library by CMU.
 *
 * \verbatim

 -------------------------------------------------------------------------
  Copyright 1999, 2000         #### ### ### ## ## ## #### ##  ###  ##  ##
  James R. Bruce              ##    ####### ## ## ## ##   ## ## ## ######
  School of Computer Science  ##    ## # ## ## ## ##  ### ## ## ## ## ###
  Carnegie Mellon University   #### ##   ##  ###  ## #### ##  ###  ##  ##
 -------------------------------------------------------------------------

 * \endverbatim
 */
int camRLELabeling(CamRLEImage *src, CamBlobs *results);

/// The RLE Blob Analysis function
/** Automatically called by camRLELabeling().
 *
 *  \param src	    The source ::CamRLEImage, already labelled.
 *  \param results  The ::CamBlobs containing the results of the blob analysis.
 *  \return	    0 (false) if an error occurs. Generally when the number
 *		    of blobs exceeds the <DFN>CAM_LABEL_MAX_BLOBS</DFN> constant.
 */
int camRLEBlobAnalysis(CamRLEImage *src, CamBlobs *results);

/// The RLE "Apply a LUT" function
/** Very useful to post-process a RLE image. This function join runs when needed.
 *
 *  Applying a LUT (i.e. for instance thresholding) can be useful
 *  for many purposes (like "several thresholds at once" processing). For experts only.
 *
 *  \param src	    The source ::CamRLEImage.
 *  \param dest	    The destination ::CamRLEImage.
 *  \param LUT	    Look-up-table applied to the <DFN>value</DFN> member of each run.
 *  \return	    0 (false) if an error occurs.
 */
int camRLEApplyLUT(CamRLEImage *src, CamRLEImage *dest, CamTable *LUT);

/// The RLE decoding function
/** Reconstructs an ::CamImage from runs.
 *
 *  \param src	    The source ::CamRLEImage.
 *  \param dest	    The destination ::CamImage. Must be grey-level image
 *		    (binary image RLE decoding not yet implemented).
 *  \param LUT	    Look-up-table applied to the <DFN>value</DFN> member of each run, to produce
 *		    the pixel value in the <DFN>dest</DFN> image.
 *  \return	    0 (false) if an error occurs.
 *
 *  \todo   Implement binary image RLE decoding
 */
int camRLEDecode(CamRLEImage *src, CamImage *dest, CamTable *LUT);

/// Another RLE decoding function, used to retrieve some specific blobs.
/** Reconstructs an ::CamImage from a labelled RLE image. Very useful to see the result of labeling.
 *
 *  \param src	    The source ::CamRLEImage.
 *  \param dest	    The destination ::CamImage. Must be grey-level image
 *		    (binary image RLE decoding not yet implemented).
 *  \param LUT	    Look-up-table applied to the <DFN>blob</DFN> member of each run, to produce
 *		    the pixel value in the <DFN>dest</DFN> image.
 *  \return	    0 (false) if an error occurs.
 *
 *  \todo   Implement binary image RLE decoding
 */
int camRLEDecodeBlobs(CamRLEImage *src, CamImage *dest, CamTable *LUT);

/// RLE image inversion
/** Very useful function, for finding holes in blobs.
 *  \param image    The source and destination ::CamRLEImage. In-place processing only.
 *  \return	    0 (false) if an error occurs.
 */
int camRLEInverse(CamRLEImage *image);

/// RLE blob sides reconstruction
int camRLEBlobSides(CamBlobInfo *blob, int *left, int *top, int *right, int *bottom);

/// RLE blob intersection with a ROI
int camRLEBlobROIIntersect(CamBlobInfo *blob, CamROI *roi);

/// Retrieves the average, min and max values
int camRLEBlobMeasures(CamBlobInfo *blob, CamImage *original);

/// RLE image erosion (cross structural element)
/** 
 *  \param image    The source ::CamRLEImage.
 *  \param result   The destination ::CamRLEImage
 *  \return	    0 (false) if an error occurs.
 */
int camRLEErodeCross(CamRLEImage *image, CamRLEImage *result);

/// RLE image erosion (3x3 square structural element)
/** 
 *  \param image    The source ::CamRLEImage.
 *  \param result   The destination ::CamRLEImage
 *  \return	    0 (false) if an error occurs.
 */
int camRLEErode3x3(CamRLEImage *image, CamRLEImage *result);

/// RLE image erosion (3x2 square structural element)
/** 
 *  \param image    The source ::CamRLEImage.
 *  \param result   The destination ::CamRLEImage
 *  \return	    0 (false) if an error occurs.
 */
int camRLEErode3x2(CamRLEImage *image, CamRLEImage *result);

//@}

/** @name Histogram computation
 */
//@{

/// The classical image histogramming
/** Counts the number of occurence of each pixel value, on one channel only
 *  (grey-level image), for images up to 12-bits deep. It supports signed images
 *  by centering the histogram around the 0 value.
 *
 *  \param image    The ::CamImage to analyze. One channel only.
 *  \param histo    A pointer to a CamTable (array of integers) that will contain the results of histogramming.
 *  \return	    0 (false) if an error occurs. Accumulated values of scanned pixels otherwise.
 *
 *  camHistogram supports masking and ROIs.
 */
int camHistogram(CamImage *image, CamTable *histo);

#define CAM_EQUAL_PERFECT 0
#define CAM_EQUAL_FAST	  1

/// Histogram Equalization  
/** Performs a histogram equalization, with no holes. 
 * 
 *  \param src      The ::CamImage to equalize. One channel only.
 *  \param dest	    The resulting equalized image.
 *  \param src_histo The histogram of the source image, obtained by a previous call to camHistogram().
 *  \param option   <DFN>CAM_EQUAL_FAST</DFN> or <DFN>CAM_EQUAL_PERFECT</DFN> (default)
 *  \param work	    A ::CamImage used for internal computation. If set to NULL, allocation and deallocation will be done internally.   
 *  \return	    0 (false) if an error occurs. 
 *
 *  camHistogramEqualization supports in-place operation, masking and ROIs.
 */
int camHistogramEqualization(CamImage *src, CamImage *dest, CamTable *src_histo, int option, CamImage *work);

/// Two channels histogram computation
/** The result of this operation is 2D, and thus is stored in an image.
 *
 *  \param image    The ::CamImage for which to compute the histogram
 *  \param ch1	    The first channel number
 *  \param ch2	    The second channel number
 *  \param result   The ::CamImage containing the results of histogramming.
 *  \param size	    Subsampling factor. size=1 results in a 256x256 picture, size=2 results in a 128x128 picture, etc.
 *  \return	    0 (false) if an error occurs.
 */
int camHistogram2Channels(CamImage *image, int ch1, int ch2, CamImage *result, int size);

/// Compute the threshold for a given percentage of pixels 
int camFindThreshold(CamTable *histo, int percent);
//@}

/* Horizontal and vertical summing
 */

/// Horizontal and vertical summing function
/** Very useful to detect features in images, generally after applying
 *  a filter like sobel or morphological gradient.
 *
 *  \param image    The ::CamImage to process (or its ROI)
 *  \param hsum     The ::CamTable array of integers containing the horizontal sum.
 *  \param vsum     The ::CamTable array of integers containing the vertical sum.
 *  \return	    0 (false) if an error occurs.
 */
int camSumHV(CamImage *image, CamTable *hsum, CamTable *vsum);

/// Vertical summing function
/** Very useful to detect features in images, generally after applying
 *  a filter like sobel or morphological gradient.
 *
 *  \param image    The ::CamImage to process (or its ROI)
 *  \param results  The ::CamTable array of integers filled with results.
 *  \return	    0 (false) if an error occurs.
 */
int camSumV(CamImage *image, CamTable *results);

/// Horizontal summing function
/** Very useful to detect features in images, generally after applying
 *  a filter like sobel or morphological gradient.
 *
 *  \param image    The ::CamImage to process (or its ROI)
 *  \param results  The ::CamTable array of integers filled with results.
 *  \return	    0 (false) if an error occurs.
 */
int camSumH(CamImage *image, CamTable *results);

#endif // SWIG

/* Measures in an image : min, max, average computation
 */
#ifdef __cplusplus
/// The structure containing the result of measuring
struct CamMeasuresResults {
#else
/// The structure containing the result of measuring
typedef struct {
#endif
    int min, xmin, ymin;        ///< Minimum pixel value
    int max, xmax, ymax;	///< Maximum pixel value
    int average;		///< Average pixel value
    int sum;			///< Sum of all the pixels
#ifdef __cplusplus
    CamMeasuresResults() {min=0;xmin=0;ymin=0;max=0;xmax=0;ymax=0;average=0;sum=0;}
};
#else
} CamMeasuresResults;
#endif

#ifndef SWIG

/// Sum of pixels in an image
int camSumOfPixels(CamImage *image);

/// Measures in an image : min, max, average and sum
/** Average deviation is not measured. See camMeasureAverageDeviation().
 *
 *  \param image    The ::CamImage to process (or its ROI)
 *  \param results  The ::CamMeasuresResults structure to be filled.
 *  \return	    0 (false) if an error occurs.
 */
int camMeasures(CamImage *image, CamMeasuresResults *results);

/// Average deviation computation
/** This makes the second scan required for average deviation estimation.
 *  Uses the average field in ::CamMeasuresResults structure, so camMeasures()
 *  must have been called prior to this function (to do the 1st scan).
 *
 *  \param image    The ::CamImage to process (or its ROI)
 *  \param average  A former measure of average. If 0 or a negative number is provided, the average will be computed
 *  \return	    0 (false) if an error occurs.
 */
float camMeasureAverageDeviation(CamImage *image, int average);

/** @name Warping functions
 */
//@{

/// Warping using Volberg's algorithm
/** This is a forward separable mapping algorithm, i.e. the user
 *  must provide two functions that compute the destination points
 *  of any source point.
 *
 *  This function is the core function, making a single scanline mapping.
 *  It must be called twice to have a full mapping. Use camVolbergFwd()
 *  to do this.
 */
#ifdef CAM_VOLBERG_ORIGINAL
void volbergfvd(double f[], int in[] , int out[] , int inlen, int outlen);
#else
void camVolbergFwdScanline(CAM_PIXEL *in, int inlen, CAM_PIXEL *out, int outlen, double f[]);
#endif

/// The structure to provide to the Volberg's algorithm : two functions
typedef struct {
    void (*hfwd)(int x, int y, double *xp); ///< First scan, horizontal resampling
    void (*vfwd)(int x, int y, double *yp); ///< Second scan, vertical resampling
} CamVolbergFwdParams;

/// Helper function for using Volberg's warping algorithm
/** 
 *  \param source   The ::CamImage to warp
 *  \param dest	    The warped image
 *  \param params   The ::camVolbergFwdParams structure providing the mapping functions
 */
void camVolbergFwd(CamImage *source, CamImage *dest, CamVolbergFwdParams *params);

/* Backward warping
 */

#endif // SWIG

#define CAM_NN_INTERPOLATION 0
#define CAM_BILINEAR_INTERPOLATION 1

#ifndef SWIG

/// The parameters structure used by camWarping()
typedef struct {
    int interpolation;	///< Interpolation method (either <DFN>CAM_NN_INTERPOLATION</DFN> or <DFN>CAM_BILINEAR_INTERPOLATION</DFN>.
    int perspective;	///< 2D (0) or 3D (parallel to the ground)
    
    /// The points in the source image.
    /** Beware : These are 16-bits fixed-points (1 is (1<<16)).
     *
     * Indexes :
     * - 0 : top-left source point
     * - 1 : top-right source point
     * - 2 : bottom-right source point
     * - 3 : bottom-left source point
     */
    CamPoint p[4];
} CamWarpingParams;

/// Backward warping function
/** This function operates a backward mapping from the source image to a destination image.
 *  All the params refer to locations in the source image, whereas the ROI scanned
 *  by this warping function is set by the <DFN>roi</DFN> member of the dest image
 *
 *  \param source   The ::CamImage to warp
 *  \param dest	    The warped image
 *  \param params   The ::CamWarpingParams structure providing the ROI in the source image (not rectangular)
 *
 */
int camWarping(CamImage *source, CamImage *dest, CamWarpingParams *params);

/// Scaling function
/** This function operates a scaling from the source image to the destination image.
 *  It can upscale or downscale pictures, and uses bilinear interpolation technique.
 *
 *  \param source   The ::CamImage to warp
 *  \param dest	    The warped image
 */
int camScale(CamImage *source, CamImage *dest);

int camWarpingSuperSampling(CamImage *source, CamImage *dest, CamWarpingParams *params);

/// Simple helper function, used by camWarping()
/** 
 *  \param p	    An array defining the segments to intersect (AB and CD). These points
 *		    must be provided in 16-bits fixed-point arithmetic form (1 is (1<<16)).
 *  \param res	    The intersection point. 
 *  \return 0 if the segments are parallel. 1 otherwise
 */
int camIntersectionSegments(CamPoint p[4], CamPoint *res);
//@}

/** @name Linear filtering kernel
 */
//@{

// For compatibility with older versions
#define CamSobel3x3 camSobel
#define CamSobelAbs3x3 camSobelAbs

#define CAM_LINEAR_FILTER_KERNEL_MAX_SIZE 7

#endif //SWIG

#ifdef __cplusplus
/// The parameters structure for linear filtering
struct CamLinearFilterKernel {
#else
/// The parameters structure for linear filtering
typedef struct {
#endif
#ifndef SWIG
    int kernel[CAM_LINEAR_FILTER_KERNEL_MAX_SIZE][CAM_LINEAR_FILTER_KERNEL_MAX_SIZE]; ///< The NxN coefficients matrix (aka the kernel)
#endif
    int coeff1;         ///< Multiplicative coefficient
    int coeff2;	        ///< Final value is (result*coeff1)>>coeff2. This is to simulate division.
#ifdef __cplusplus
    /// Default constructor
    CamLinearFilterKernel() {
        for (int i=0;i<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE;i++) {
            for (int j=0;j<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE;j++) {
                kernel[i][j]=0;
            }
        }
        coeff1=1;
        coeff2=0;
    }       
    /// Set an element of the linear filter kernel
    bool set(int x, int y, int val) {
        if ((x>=0)&&(x<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE)&&(y>=0)&&(y<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE)) {
            kernel[x][y]=val; return true;
        } else return false;
    }
    /// Get an element from the linear filter kernel
    int get(int x,int y) {
        if ((x>=0)&&(x<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE)&&(y>=0)&&(y<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE)) {
            return kernel[x][y];
        } else return 0;
    }
};
#else
} CamLinearFilterKernel;
#endif

#ifndef SWIG

/// 3x3 Linear Filtering function
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param params   A pointer to a ::CamLinearFilterKernel structure, providing
 *		    the linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camLinearFilter3x3(CamImage *source, CamImage *dest, CamLinearFilterKernel *params);

/// 5x5 Linear Filtering function
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param params   A pointer to a ::CamLinearFilterKernel structure, providing
 *		    the linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camLinearFilter5x5(CamImage *source, CamImage *dest, CamLinearFilterKernel *params);

/// 3x3 Linear Filtering function (absolute)
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param params   A pointer to a ::CamLinearFilterKernel structure, providing
 *		    the linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camLinearFilterAbs3x3(CamImage *source, CamImage *dest, CamLinearFilterKernel *params);

/// 5x5 Linear Filtering function (absolute)
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param params   A pointer to a ::CamLinearFilterKernel structure, providing
 *		    the linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camLinearFilterAbs5x5(CamImage *source, CamImage *dest, CamLinearFilterKernel *params);

/* Obsolete
 * int camSobel(CamImage *source, CamImage *dest, int vert_edges);
 * int camSobelAbs(CamImage *source, CamImage *dest, int vert_edges);
 */

#endif //SWIG

#ifdef __cplusplus
/// The parameters structure for linear filtering
struct CamSepFilterKernel {
#else
/// The parameters structure for linear filtering
typedef struct {
#endif
#ifndef SWIG
    int x[CAM_LINEAR_FILTER_KERNEL_MAX_SIZE]; ///< The horizontal array of coefficients 
    int y[CAM_LINEAR_FILTER_KERNEL_MAX_SIZE]; ///< The vertical array of coefficients
#endif
    int coeff1;         ///< Multiplicative coefficient
    int coeff2;	        ///< Final value is (result*coeff1)>>coeff2. This is to simulate division.
#ifdef __cplusplus
    /// Default constructor
    CamSepFilterKernel() {
        for (int i=0;i<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE;i++) {
            x[i]=0; y[i]=0;
        }
        coeff1=1;
        coeff2=0;
    }       
    /// Set an element of the linear separable filter kernel
    bool set_x(int y, int val) {
        if ((y>=0)&&(y<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE)) {
            x[y]=val; return true;
        } else return false;
    }
    bool set_y(int x, int val) {
        if ((x>=0)&&(x<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE)) {
            y[x]=val; return true;
        } else return false;
    }
    /// Get an element from the linear separable filter kernel
    int get_x(int y) {
        if ((y>=0)&&(y<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE)) {
            return x[y];
        } else return 0;
    }
    int get_y(int x) {
        if ((x>=0)&&(x<CAM_LINEAR_FILTER_KERNEL_MAX_SIZE)) {
            return y[x];
        } else return 0;
    }
};
#else
} CamSepFilterKernel;
#endif

#define CAM_SOBEL_H         1
#define CAM_SOBEL_V         2
#define CAM_GAUSSIAN_3x3    3
#define CAM_GAUSSIAN_5x5    4
#define CAM_GAUSSIAN_7x7    5
#define CAM_SCHARR_H        6
#define CAM_SCHARR_V        7

#ifndef SWIG

/// 3x3 Linear Filtering function with a separable kernel
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamSepFilterKernel structure, specifying the separable linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSepFilter3x3(CamImage *source, CamImage *dest, CamSepFilterKernel *kernel);

/// 3x3 Linear Filtering function with a separable kernel (absolute value)
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamSepFilterKernel structure, specifying the separable linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSepFilterAbs3x3(CamImage *source, CamImage *dest, CamSepFilterKernel *kernel);

/// 5x5 Linear Filtering function with a separable kernel
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamSepFilterKernel structure, specifying the separable linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSepFilter5x5(CamImage *source, CamImage *dest, CamSepFilterKernel *kernel);

/// 5x5 Linear Filtering function with a separable kernel (absolute value)
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamSepFilterKernel structure, specifying the separable linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSepFilterAbs5x5(CamImage *source, CamImage *dest, CamSepFilterKernel *kernel);

/// 7x7 Linear Filtering function with a separable kernel
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamSepFilterKernel structure, specifying the separable linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSepFilter7x7(CamImage *source, CamImage *dest, CamSepFilterKernel *kernel);

/// 7x7 Linear Filtering function with a separable kernel (absolute value)
/** \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \param kernel   A pointer to a ::CamSepFilterKernel structure, specifying the separable linear kernel to use.
 *  \return Sum (Accumulator) of all computed pixels
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSepFilterAbs7x7(CamImage *source, CamImage *dest, CamSepFilterKernel *kernel);

/// 3x3 Horizontal Sobel Filter. Detects horizontal edges.
/** \param source       The source ::CamImage to process. Grey scale only.
 *  \param dest	        The destination ::CamImage
 *  \return 1 if the function succeeds. 0 otherwise.
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSobelH(CamImage *source, CamImage *dest);

/// 3x3 Horizontal Sobel Filter (absolute). Detects horizontal edges.
/** \param source       The source ::CamImage to process. Grey scale only.
 *  \param dest	        The destination ::CamImage
 *  \return 1 if the function succeeds. 0 otherwise.
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSobelHAbs(CamImage *source, CamImage *dest);

/// 3x3 Vertical Sobel Filter. Detects Vertical edges.
/** \param source       The source ::CamImage to process. Grey scale only.
 *  \param dest	        The destination ::CamImage
 *  \return 1 if the function succeeds. 0 otherwise.
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSobelV(CamImage *source, CamImage *dest);

/// 3x3 Vertical Sobel Filter (absolute). Detects Vertical edges.
/** \param source       The source ::CamImage to process. Grey scale only.
 *  \param dest	        The destination ::CamImage
 *  \return 1 if the function succeeds. 0 otherwise.
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camSobelVAbs(CamImage *source, CamImage *dest);

/// Linear convolution with a predefined kernel
/** The function camFixedFilter() is used to convolve the input image with a predefined filter kernel
 *  specified in argument.
 *
 *  \param source       The source ::CamImage to process. Grey scale only.
 *  \param dest	        The destination ::CamImage
 *  \param filter       Constant defining the kernel to be used. The filter kernel can be one of the following :
 *      - <DFN>CAM_SOBEL_H</DFN> performs a horizontal edges detection (3x3 sobel)
 *      - <DFN>CAM_SOBEL_V</DFN> performs a vertical edges detection (3x3 sobel)
 *      - <DFN>CAM_GAUSSIAN_3x3</DFN> performs a gaussian filtering
 *      - <DFN>CAM_GAUSSIAN_5x5</DFN> performs a gaussian filtering
 *      - <DFN>CAM_GAUSSIAN_7x7</DFN> performs a gaussian filtering
 *      - <DFN>CAM_SCHARR_H</DFN> performs a horizontal edges detection (3x3 scharr filter. Better preserves gradient direction)
 *      - <DFN>CAM_SCHARR_V</DFN> performs a vertical edges detection (3x3 scharr filter. Better preserves gradient direction)
 *
 *  \return 1 if the function succeeds. 0 otherwise.
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 *
 *  Note also that signed saturation is to (-127;+127) instead of the traditional signed char (-128;+127),
 *  so that results of filtering are unbiased.
 */
int camFixedFilter(CamImage *source, CamImage *dest, int filter);

int camScharrH(CamImage *source, CamImage *dest);
int camScharrV(CamImage *source, CamImage *dest);
int camScharrHAbs(CamImage *source, CamImage *dest);
int camScharrVAbs(CamImage *source, CamImage *dest);

//@}

/** @name Median filtering kernel
 */
//@{

#define camMedianFiltering3x3 camMedianFilter3x3
#define camMedianFiltering5x5 camMedianFilter5x5

/// 3x3 Median Filtering function
/** For each 3x3 set of pixels, keep the median value. Considered as being a
 *  good filter, less sensitive to noise.than linear filtering.
 *
 *  \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \return 0 (false) if an error occurs
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camMedianFilter3x3(CamImage *source, CamImage *dest);

/// 5x5 Median Filtering function
/** For each 5x5 set of pixels, keep the median value. Considered as being a
 *  good filter, less sensitive to noise.than linear filtering.
 *
 *  \param source   The source ::CamImage to process. Grey scale only.
 *  \param dest	    The destination ::CamImage
 *  \return 0 (false) if an error occurs
 *
 *  Note that this function supports in-place processing (i.e. dest can be the same as source param)
 */
int camMedianFilter5x5(CamImage *source, CamImage *dest);
//@}

/** @name Watersheding kernel
 */
//@{

/// 1D watershed computation
/** Retrieves all the watershed points, along with their depth (i.e. the minimum depth of
 *  the 2 catchment bassins contributing to the watershed point).
 *
 *  1D watersheding operation is a very smart way to analyze the results of an histogram
 *  (generally before thresholding), yielding better results than a simple n% threshold.
 *
 *  \param input    The 1D integer data for which to compute the watershed
 *  \param size	    The size of the 1D integer array (<DFN>t</DFN>). Generally 256 when <DFN>t</DFN> is the
 *		    results of an histogramming operation.
 *  \param results  The 1D integer array contaning the results of the watershed computation.
 *		    A value different from 0 indicates a watershed point, the value indicating the depth
 *		    (and thus the importance) associated to this watershed point.
 *		    The memory for this array is not allocated by this function, and thus an array
 *		    of appropriate size must be allocated before calling this function.
 *
 *  \return 0 (false) if an error occurs
 */
int camWatershed1D(int *input, int size, int *results);
#else
%immutable;
#endif // SWIG

/// The structure defining a basin
typedef struct 
{
   int dynamics;	///< Its dynamics (importance)
   int minimum;		///< Minimum of the basin
   int flooded;		///< The catchment basin which has flooded it
   int surface;         ///< Surface of the region
   int accsurface;      ///< Accumulated surface of all children
   unsigned short x,y;	///< Position of the minimum
} CamBasin;

#define CAM_NOT_COMPUTED 65536

#ifndef SWIG
#ifdef __cplusplus
/// The table of basins structure. Simply a dynamic size array of <DFN>CamBasin</DFN>.
struct CamTableOfBasins {
    int sizeMax;
    int nbBasins;
    CamBasin *tab;
    CamBasin& operator[](int index) {return tab[index];}
    void get_rid_of(CamBasin &basin) {basin.surface=0;}
    CamTableOfBasins() {tab=NULL;sizeMax=0;nbBasins=0;}
    ~CamTableOfBasins();
};
#else
typedef struct
{
   int sizeMax;
   int nbBasins;
   CamBasin *tab;
} CamTableOfBasins;
#endif

/// Free a table of basins after use.
void camFreeTableOfBasins(CamTableOfBasins *t);

/// 2D Hierarchical watershed computation
/** Retrieves all the basins (regional segmentation)
 *
 *  Watersheding computes a hierarchichal segmentation of an image.
 *  Watersheding operation is a very smart way to analyze the results of a gradient image.
 *  
 *  \param source   The ::CamImage for which to compute the watershed
 *  \param dest	    A 16-bits deep ::CamImage where the results of the watersheding operation are stored.
 *		    The index of the basin is given by the pixel value minus 1 (so starting at 1).
 *                  Border pixels are set to -32768.
 *		    <DFN>dest</DFN> must be allocated by the caller, and must have the same size as
 *		    the <DFN>source</DFN> image. It must also be a signed 16 bits deep image (<dfn>CAM_DEPTH_16S</dfn>).
 *  \param tob	    The table of basins, containing their dynamics, their surface and their minimum value.
 *		    This table is allocated by the function.
 *
 *  \return 0 (false) if an error occurs
 */
int camHierarchicalWatershed(CamImage *source, CamImage *dest, CamTableOfBasins *tob);

/// 2D Hierarchical watershed computation (with watershed/contours markers)
/** Retrieves all the watershed points, along with all the basins (regional segmentation)
 *
 *  Watersheding computes a hierarchichal segmentation of an image.
 *  Watersheding operation is a very smart way to analyze the results of a gradient image.
 *  
 *  \param source   The ::CamImage for which to compute the watershed
 *  \param dest	    A 16-bits deep ::CamImage where the results of the watersheding operation are stored.
 *		    A negative pixel value indicates a watershed point.
 *		    A positive indicates that this pixel belongs to a basin.
 *		    The index of the basin is given by the pixel value minus 1. Border pixels are set to -32768.
 *		    <DFN>dest</DFN> must be allocated by the caller, and must have the same size as
 *		    the <DFN>source</DFN> image. It must also be a signed 16 bits deep image (<dfn>CAM_DEPTH_16S</dfn>).
 *  \param tob	    The table of basins, containing their dynamics, their surface and their minimum value.
 *		    This table is allocated by the function.
 *
 *  \return 0 (false) if an error occurs
 */
int camHierarchicalWatershedContours(CamImage *source, CamImage *dest, CamTableOfBasins *tob);

/// Retrieves regions from a watershed image
/** Retrieves all the regions of basins with surface different from 0. The user can set the surface of a basin in
 *  the table of basins to deselect it. Thus, the user can remove from the watershed image all the basins with
 *  low dynamics, and the pixels of this basin will be affected to a flooding basin.
 *
 *  \return 0 (false) if an error occurs
 */
int camHierarchicalWatershedRegions(CamImage *watershed, CamTableOfBasins *tob);
//@}

/** @name Hough functions
 */
//@{

/// Circle Hough. Find a circle in a picture.
/** Find a circle in the input picture. Returns the circle with the bigger hough accumulation.
 *
 *  \param image    The ::CamImage to process
 *  \param percent  The percentage of pixels to consider (from the gradient image). Directly proportionnal to the speed of exection. Typically 10.
 *  \param rmin     The minimum radius to look for
 *  \param rmax     The maximum radius to look for. This determines the size of the (x,y,r) Hough cube
 *  \param xc       A pointer to the circle center (return value)
 *  \param yc       A pointer to the circle center (return value)
 *  \param rc       A pointer to the circle radius (return value)
 *
 *  \return The confidence in the circle found (hough accumulator)
 */
int camHoughCircle(CamImage *image, int percent, int rmin, int rmax, int *xc, int *yc, int *rc);

struct _CamKeypoints;

#endif // SWIG 

/** @name Keypoints and Local descriptors 
 */
//@{

#ifdef __cplusplus
#ifdef SWIG
    %mutable;
#endif

/// The Keypoint structure
struct CamKeypoint {
#else
typedef struct {
#endif // __cplusplus
    int descriptor[128];    ///< The descriptor table itself
    int x;		    ///< x coordinate of keypoints in image
    int y;		    ///< y coordinate of keypoints in image
    int scale;		    ///< Scale in pixels
    int angle;		    ///< Angle in degrees
    int value;		    ///< Hessian value
#ifdef SWIG
    %immutable;
#endif
    int size;		    ///< Size of descriptor
    void *internal;	    ///< Internal use only
#ifdef __cplusplus
    CamKeypoints *set;
    bool draw(CamImage &image, int color = 255) const;	 ///< C++ wrapping for camDrawKeypoint function
    bool set_descriptor(const int* const array, int sz); ///< Set the descriptor
    CamKeypoint() { x = 0; y = 0; scale = 0; angle = 0; value = 0; size = 0; internal = NULL; set = NULL; }
};
#else
    struct _CamKeypoints *set;
} CamKeypoint;
#endif

typedef struct {
    CamKeypoint *p1;
    CamKeypoint *p2;
    int mark;
    int error;
} CamKeypointsMatch;

#define CAM_MAX_NB_MATCHES 2048

#ifdef __cplusplus
/// Affine Transform parameters
struct CamAffineTransform {
    double m[6];
};

/// A match between 2 CamKeypoints
struct CamKeypointsMatches {
#else

typedef struct {
    double m[6];
} CamAffineTransform;

typedef struct {
#endif //__cplusplus
    int nbMatches;
    int nbOutliers;
    int allocated;
    CamKeypointsMatch *pairs;
#ifdef __cplusplus
    CamKeypointsMatches(int size = CAM_MAX_NB_MATCHES);		    ///< Default constructor
    ~CamKeypointsMatches();					    ///< Default destructor
    CamAffineTransform find_affine_transform(int *error) const;	    ///< C++ wrapping for camFindAffindTransform function
    CamAffineTransform find_affine_transform2(int *error) const;    ///< C++ wrapping for camFindAffindTransform function
};
#else
} CamKeypointsMatches;
#endif

#ifdef __cplusplus
struct CamKeypointsKdTree;

#ifdef SWIG
    %mutable;
#endif
/// The CamKeypoints structure
struct CamKeypoints {
#else
/// The CamKeypoints structure
typedef struct _CamKeypoints {
#endif
    int width, height;		    ///< Original size of object (or picture of object)
    int cx, cy;			    ///< Reference coordinate of object (center)
    int id;			    ///< Id of reference object
#ifdef SWIG
    %immutable;
#endif
    int allocated;
    int nbPoints;	            ///< Number of valid points
    CamKeypoint **keypoint; ///< Array of keypoints
    CamKeypoint *bag;	    ///< Bag of keypoints
#ifdef __cplusplus
    CamKeypoints() {allocated = 0; nbPoints = 0; keypoint = NULL; bag = NULL; cx = 0; cy = 0;} ///< Default constructor
    CamKeypoints(int nbPoints);			///< Constructor with max number of points
#ifndef SWIG
    CamKeypoint& operator[](int index);
#endif
    ~CamKeypoints();                                ///< Default destructor

    bool add(CamKeypoint &p);
    CamKeypoints& operator<<(CamKeypoint &p) { add(p); return *this; }
    bool draw(CamImage &image, int color = 255) const;	///< C++ wrapping for camDrawKeypoints function
    int matching(const CamKeypoints **models, int nbModels, CamKeypointsMatches &matches) const;  ///< C++ wrapping for camKeypointsMatching() function
    int matching2(const CamKeypoints &points, CamKeypointsMatches &matches) const;		  ///< C++ wrapping for camKeypointsMatching2() function
    int matchingKdTree(const CamKeypointsKdTree &kdTree, CamKeypointsMatches &matches, int explore = 100) const; ///< C++ wrapping for camKeypointsMatchingKdTree() function 

    bool alloc(int nbPoints);                           ///< Allocator (C++ wrapping of camKeypointsAllocate() function)
    bool realloc(int nbPoints);                         ///< Reallocator (C++ wrapping of camKeypointsReallocate() function)
};
#else
} CamKeypoints;
#endif

#define CAM_UPRIGHT 1

int camKeypointsSetParameters(int patchSize, int sigma, int threshGradient);

#ifndef SWIG

/// Keypoints allocation
int camAllocateKeypoints(CamKeypoints *fpoints, int nbPoints);

/// Keypoints reallocation
int camReallocateKeypoints(CamKeypoints *fpoints, int nbPoints);

/// Keypoints release 
int camFreeKeypoints(CamKeypoints *fpoints);

/// Draw kepoints on screen
int camDrawKeypoints(CamKeypoints *points, CamImage *dest, int color);

/// Draw one keypoint on screen
int camDrawKeypoint(CamKeypoint *point, CamImage *dest, int color);

/// Harris corner detector.
/** \param k Harris' corner detection parameter (default value is 41, matching k=0.04 in Harris' article)
 */
int camHarris(CamImage *source, CamKeypoints *points, int k);

/// Local maxima finder (Circle7 neighborhood)
int camFindLocalMaximaCircle7(CamImage *source, CamKeypoints *points, int threshold);

/// Local maxima finder (Circle5 neighborhood)
int camFindLocalMaximaCircle5(CamImage *source, CamKeypoints *points, int threshold);

/// Local maxima finder (Square3 neighborhood)
int camFindLocalMaximaCircle3(CamImage *source, CamKeypoints *points, int threshold);

/// Integral image computation
int camIntegralImage(CamImage *src, CamImage *dest);

/// Fast Hessian Detection (for one scale only)
int camFastHessianDetectorFixedScale(CamImage *integral, CamImage *dest, int scale);

/// Fast Hessian Detection (for all scales)
int camFastHessianDetector(CamImage *source, CamKeypoints *points, int threshold, int options);

/// Find a keypoint in a set of keypoints
CamKeypoint* camFindKeypoint(CamKeypoint *point, CamKeypoints *points, int *dist1, int *dist2);

/// Allocate keypoints matches
int camAllocateKeypointsMatches(CamKeypointsMatches *matches, int nbpairs);

/// Free keypoints matches
void camFreeKeypointsMatches(CamKeypointsMatches *matches);

/// Find an object in a database (brute force matching)
int camKeypointsMatching(CamKeypoints *target, CamKeypoints **models, int nbModels, CamKeypointsMatches *matches);
int camKeypointsMatching2(CamKeypoints *points1, CamKeypoints *points2, CamKeypointsMatches *matches);

/// Find the affine transform from one object to the next
int camFindAffineTransform(CamKeypointsMatches *matches, CamAffineTransform *t, int *error);
int camFindAffineTransform2(CamKeypointsMatches *matches, CamAffineTransform *t, int *error);

/// Apply an affine transform on a point (xy) to find the target point (uv)
void camApplyAffineTransform(CamPoint *xy, CamPoint *uv, CamAffineTransform *t);

typedef struct _CamFPKdTreeNode {
    int i;
    int m;
    struct _CamFPKdTreeNode *right;
} CamFPKdTreeNode;

int camKeypointsMatchingKdTree(CamKeypoints *target, CamFPKdTreeNode *kdTreeRoot, CamKeypointsMatches *matches, int explore);

CamFPKdTreeNode *camKeypointsCompileKdTree(CamKeypoints **models, int nbModels);

CamKeypoint *camFindKeypointKdTree(CamKeypoint *point, CamFPKdTreeNode *kdTreeRoot, int explore, int *dist1, int *dist2);

#endif // SWIG

#ifdef __cplusplus
/// The CamKeypointsKdTree class 
struct CamKeypointsKdTree {
    CamFPKdTreeNode *root;

    void compile(const CamKeypoints **models, int nbModels) {root = camKeypointsCompileKdTree((CamKeypoints**)models, nbModels);}
    CamKeypoint *find(const CamKeypoint *point, int explore = 100, int *dist1 = NULL, int *dist2 = NULL) const; // C++ Wrapper for camFindKeypointKdTree function

    CamKeypointsKdTree(const CamKeypoints **models = NULL, int nbModels = 0) { if (nbModels != 0) compile(models, nbModels); else root = NULL;};
    ~CamKeypointsKdTree() {if (root) free(root);}	///< Default destructor
};
#endif // __cplusplus

//@}

/** @name Utility functions
 */
//@{

/* Image allocation utility routines
 */

#ifndef SWIG

/// Grey scale image allocation
int camAllocateImage(CamImage *image, int width, int height, int depth);
/// Any kind of image allocation
int camAllocateImageEx(CamImage *image, int width, int height, int depth, int color_seq);
/// Fill image header (no memory initialization)
int camFillImageHeader(CamImage *image, int width, int height, int depth, int channelseq);
/// YUV image allocation
int camAllocateYUVImage(CamImage *image, int width, int height);
/// HLS image allocation
int camAllocateHLSImage(CamImage *image, int width, int height);
/// RGB image allocation
int camAllocateRGBImage(CamImage *image, int width, int height);
/// RGB image allocation with alpha channel
int camAllocateRGBAImage(CamImage *image, int width, int height);
/// BGR image allocation
int camAllocateBGRImage(CamImage *image, int width, int height);
/// BGR image allocation with alpha channel
int camAllocateBGRAImage(CamImage *image, int width, int height);
/// Image memory release 
int camDeallocateImage(CamImage *image);
/// Image memory release
int camFreeImage(CamImage *image);

/* Other useful functions
 */
/// Set the ROI utility function
int camSetROI(CamROI *roi, int coi, int xOffset, int yOffset, int width, int height);
/// Set the ROI to the maximum size of the image
int camSetMaxROI(CamROI *roi, CamImage *image);
/// Reduce the roi by a given number of pixels on each side
int camReduceROI(CamROI *roi, int pixels);
/// Enlarge the roi by a given number of pixels on each side
int camEnlargeROI(CamROI *roi, int pixels);
/// Simple 2x Zoom functin (by pixels replication)
int camZoom2x(CamImage *src, CamImage *dst);
/// Simple nearest-neighbour decimation
int camDecimateNN(CamImage *src, CamImage *dest, int factor);
/// Set the image mask (Run-Length encoded mask)
int camSetRLEMask(CamImage *image, CamRLEImage *mask);
/// Set the image mask (::CamImage mask)
int camSetMask(CamImage *image, CamImage *mask);

#define camDownScaling2x2 camDownscaling2x2
/// 2x2 linear interpolation downscaling
int camDownScaling2x2(CamImage *src, CamImage *dest);
/// Copy function, without any color space conversion, but able to deal with planar/pixel oriented conversion, ROIs and even masking
/** Supports grey scale to color conversion, as well as RGB to RGBA, and RGBA to RGB copying (adding and removing alpha channel)
 *
 *  \return 0 (false) if an error occurs
 */
int camCopy(CamImage *source, CamImage *dest);
/// Simple cloning function
/** This function allocates the memory for the dest image. dest actual content is not considered by this function.
 *  Beware not to have allocated an image in dest before, otherwise this will result in a memory leak.
 */
int camClone(CamImage *source, CamImage *dest);
/// Reference copy
/** This function doesn't copy the pixels, only the structure. Destination image will reference the same pixels as the source image.
 *  The image will be freed when source will be deallocated through camDeallocateImage() function.
 */
int camRefCopy(CamImage *source, CamImage *dest);
/// Set all the pixel values to <DFN>fillValue</DFN>
int camSet(CamImage *image, int fillValue);
/// Alpha channel compositing
/** \param source1 must be a RGB image
 *  \param source2 must be a RGBA image (with an alpha channel)
 *  \param dest destination image
 */
int camAlphaComposite(CamImage *source1, CamImage *source2, CamImage *dest);

/// Set the border value of an image
int camSetBorder(CamImage *image, int borderValue);
/// Clip the roi of an image
int camClipROI(CamImage *image);
/// Clip a ROI with respect to an image
int camClip(CamROI *roi, CamImage *image);
/// Compute intersection between ROIs
int camROIIntersect(CamROI *roi1, CamROI *roi2, CamROI *dest);

#endif //SWIG
int camSetImageViewer(char *s);
#ifndef SWIG

/// View an image
int camView(CamImage *image);
/// Returns the version of the library
const char *camVersion();

/* Drawing functions
 */
/// Draw a line
int camDrawLine(CamImage *image, int x1, int y1, int x2, int y2, int color);
/// Accumulate a line in a frame (very useful for Hough transforms)
int camAccumulateLine(CamImage *image, int x1, int y1, int x2, int y2, int acc);
/// Draw a rectangle
int camDrawRectangle(CamImage *image, int x1, int y1, int x2, int y2, int color);
/// Draw some text using 16 segments font (looks like an alarm clock...)
int camDrawText16s(CamImage *image, char *text, int x, int y, int cwidth, int cheight, int orientation, int color);
/// Draw a circle
int camDrawCircle(CamImage *image, int x, int y, int r, int color);
/// Draw an ellipse
int camDrawEllipse(CamImage *image, int x, int y, int rx, int ry, int color);
/// Plot
/** Plots a point (\ref CAM_POINT), a cross (\ref CAM_CROSS), a circle (\ref CAM_CIRCLE) or a combination of them (<DFN>CAM_CROSS|CAM_CIRCLE</DFN> for instance)
 */
int camPlot(CamImage *image, int x, int y, int color, int kind);
/// Fill a region with a color
/** Try to fill the image with pixels as much possible pixels with the same color as the original pixel color in (x,y)
 *
 *  \param x horizontal coordinate where to start the filling
 *  \param y vertical coordinate where to start the filling
 *  \param fillcolor filling color (use CAM_RGB or CAM_RGBA macros to set this color)
 *  \param tolerance this sets where the filling should stop. The reference is the color of the original pixel color in (x,y), and all the pixels nearby that are within the
 *      tolerance parameters get filled. If tolerance is set to -1, the filling stops with pixels having the same color as the filling color
 *      (this mode is very useful for filling circles, rectangles, etc.)
 *
 *  \return The number of colored pixels (at least 1 is function succeeds)
 */
int camFillColor(CamImage *image, int x, int y, int fillcolor, int tolerance);

#endif // SWIG

#ifdef __cplusplus
/// The bitmap font structure
struct CamBitmapFont {
#else
typedef struct {
#endif
    int first_char;
    int nb_chars;
    int height;
    CamRLEImage *masks;
    CamImage *letters;
#ifdef __cplusplus
    CamBitmapFont() {first_char=33; nb_chars=0; masks=NULL; letters=NULL;}
    CamBitmapFont(const char *filename);
    ~CamBitmapFont();
    bool load(const char *filename);
};
#else
} CamBitmapFont;
#endif

#ifndef SWIG
/// Load a bitmap font
int camLoadBitmapFont(CamBitmapFont *font, char *filename);
/// Deallocate a bitmap font
int camFreeBitmapFont(CamBitmapFont *font);
/// Draw some text using a bitmap font
int camDrawTextBitmap(CamImage *image, char *text, int x, int y, CamBitmapFont *font);
#endif // SWIG

/// 24 bits integer color representation. Stricly equivalent to the Windows RGB macro.
/** Please use this one in place of RGB for better portability of the code.
 */
int camRGB(int r, int g, int b);

/// 32 bits integer color representation, including an alpha channel
int camRGBA(int r, int g, int b, int a);

#ifndef SWIG

/* Load and save PGM images
 */
/// Load a PGM image
int camLoadPGM(CamImage *image, char *fn);
/// Save a PGM image
int camSavePGM(CamImage *image, char *filename);
/// Save a raw PGM image (8-bits only)
int camSaveRawPGM(CamImage *image, char *filename);

/* Load and save BMP images
 */
/// Load a BMP image
int camLoadBMP(CamImage *image, char *fn);
/// Save a BMP image
int camSaveBMP(CamImage *image, char *filename);

/* Load config files
 */
#define CAM_CONFIG_MAX_ENTRIES 256
typedef struct {
    int nbEntries;
    char parameter[CAM_CONFIG_MAX_ENTRIES][128];
    char value[CAM_CONFIG_MAX_ENTRIES][128];
} CamConfig;

int camLoadConfig(const char *filename, CamConfig *config);
int camConfigInt(const CamConfig *config, const char *entry);
float camConfigFloat(const CamConfig *config, const char *entry);
const char *camConfigString(const CamConfig *config, const char *entry);

/* Image capture functions
 */
void* camCaptureInit(int options);
int camCapture(void *handle, CamImage *image);
int camCaptureOver(void *handle);

#endif

#define CAM_CAPTURE_AUTO_SOURCE 1
#define CAM_CAPTURE_DISPLAY     2
#define CAM_CAPTURE_USE_READ    4

#ifdef __cplusplus
class CamCapture {
    void *handle;
public:
    bool capture(CamImage &capture);
    bool ready() {return (handle)?true:false;}
    CamCapture(int options=0);
    ~CamCapture();
};
#endif

#ifndef SWIG

/// Error management function
void camError(char *module, char *error);
typedef void (*camErrorFunct)(char *,char*);
void camSetErrorFunct(camErrorFunct funct);
//@}

/** @name Color conversion functions
 */
//@{
int camYUV2RGB(CamImage* source, CamImage *dest); ///< Converts a YUV image to a RGB image 
int camRGB2YUV(CamImage* source, CamImage *dest); ///< Converts a RGB image to a YUV image
int camRGB2Y(CamImage *source, CamImage *dest);   ///< Converts a RGB image to a gray scale image
int camRGB2HLS(CamImage* source, CamImage *dest); ///< Converts a RGB image to a HLS image
//@}

/** @name Motion estimation functions
 */
//@{
/*
 * Computes the Sum of Absolute Values between two 8x8 blocks
 *
 * This function takes into account the blocks lying partially outside the image.
 * MMX optimized if <DFN>CAM_OPT_MMX</DFN> compilation option is set (requires Intel C++ compiler).
 *
 *
 * \param image1    Current image
 * \param image2    Previous image
 * \param bleft     The x coordinate of the current block
 * \param btop      The y coordinate of the current block
 * \param dx        The x offset to reach the candidate block
 * \param dy        The y offset to reach the candidate block
 *
 * \return The SAD value between the given two blocks
 */
int camSAD8x8(CamImage *image1, CamImage *image2, int bleft, int btop, int dx, int dy);

/*
 * Computes the Sum of Absolute Values between two 16x16 blocks
 *
 * This function takes into account the blocks lying partially outside the image.
 * MMX optimized if <DFN>CAM_OPT_MMX</DFN> compilation option is set (requires Intel C++ compiler).
 *
 *
 * \param image1    Current image
 * \param image2    Previous image
 * \param bleft     The x coordinate of the current block
 * \param btop      The y coordinate of the current block
 * \param dx        The x offset to reach the candidate block
 * \param dy        The y offset to reach the candidate block
 *
 * \return The SAD value between the given two blocks
 */
int camSAD16x16(CamImage *image1, CamImage *image2, int bleft, int btop, int dx, int dy);

typedef struct {
    int niter;          ///< Number of iterations (should be set to 0)
    int seed;           ///< Seed for randomization
    int lsearch;        ///< Number of pixels for searching around the blocks (small search)
    int rsearch;        ///< Full random search parameter
    int blockSize;      ///< Block size (8 or 16)
    int scans;          ///< Number of scans
    int candidates;     ///< Number of candidates (per scan)
    int test0;          ///< Systematically test (0,0) motion vector
} CamMotionEstimation3DRSParams;

typedef struct {
    int vx[CAM_MAX_SCANLINE/8][CAM_MAX_FRAME_HEIGHT/8];   ///< Horizontal motion vectors
    int vy[CAM_MAX_SCANLINE/8][CAM_MAX_FRAME_HEIGHT/8];   ///< Vertical motion vectors
    int SAD[CAM_MAX_SCANLINE/8][CAM_MAX_FRAME_HEIGHT/8];  ///< SAD results
} CamMotionEstimation3DRSResults;

int camMotionEstimation3DRSInit(CamMotionEstimation3DRSParams *params, int seed, int lsearch, int rsearch, int bs, int scans, int candidates, int test0);
int camMotionEstimation3DRS(CamImage *current, CamImage *previous, CamMotionEstimation3DRSParams *params, CamMotionEstimation3DRSResults *results);
//@}

/** @name 3D Projection/Retroprojection functions
 */
//@{

void camProject(const double extr[4][4], const double fc[2], const double cc[2], double x, double y, double z, int *xp, int *yp);

void camBackproject(const double extr[4][4], const double fc[2], const double cc[2], int xp, int yp, double z, double *x, double *y);

int camUndistort(CamImage *source, CamImage *dest,
                 const float* intrinsic_matrix,
                 const float* dist_coeffs);
int camUndistortFixed(CamImage *source, CamImage *dest,
                      const CAM_FIXED_POINT* intrinsic_matrix,
                      const CAM_FIXED_POINT* dist_coeffs);
int camUndistortBuildLUT(CamImage *source,
                         const float* intrinsic_matrix,
                         const float* dist_coeffs,
                         CamImage *LUTX, CamImage *LUTY);
int camUndistortLUT(CamImage *source, CamImage *dest,
                    CamImage *LUTX, CamImage *LUTY);

//@}

#endif // SWIG

#ifdef __cplusplus
}
#endif

#endif


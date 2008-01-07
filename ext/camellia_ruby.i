/* File : camellia_ruby.i */
%module camellia

%{
#include "../inc/camellia.h"
extern "C" void Init_camcapture();
%}

%include "cstring.i"

// Ruby object tracking
%trackobjects;
%feature("trackobjects","0") CamBasin;

// Bash
%rename("dealloc!") CamImage::dealloc();

%rename("erode_square3!") CamImage::erode_square3();
%rename("erode_circle5!") CamImage::erode_circle5();
%rename("erode_circle7!") CamImage::erode_circle7();
%rename("dilate_square3!") CamImage::dilate_square3();
%rename("dilate_circle5!") CamImage::dilate_circle5();
%rename("dilate_circle7!") CamImage::dilate_circle7();
%rename("morpho_gradient_square3!") CamImage::morpho_gradient_square3();
%rename("morpho_gradient_circle5!") CamImage::morpho_gradient_circle5();
%rename("morpho_gradient_circle7!") CamImage::morpho_gradient_circle7();
%rename("morpho_maths!") CamImage::morpho_maths(const CamMorphoMathsKernel &ker);
%rename("erode_3x3!") CamImage::erode_3x3(const CamMorphoMathsKernel &ker);
%rename("dilate_3x3!") CamImage::dilate_3x3(const CamMorphoMathsKernel &ker);
%rename("erode_5x5!") CamImage::erode_5x5(const CamMorphoMathsKernel &ker);
%rename("dilate_5x5!") CamImage::dilate_5x5(const CamMorphoMathsKernel &ker);
%rename("erode_7x7!") CamImage::erode_7x7(const CamMorphoMathsKernel &ker);
%rename("dilate_7x7!") CamImage::dilate_7x7(const CamMorphoMathsKernel &ker);
%rename("threshold!") CamImage::threshold(int threshold);
%rename("threshold_inv!") CamImage::threshold_inv(int threshold);
%rename("abs!") CamImage::abs();
%rename("set!") CamImage::set(int color);

%rename("linear_filter_3x3!") CamImage::linear_filter_3x3(const CamLinearFilterKernel &ker);
%rename("linear_filter_5x5!") CamImage::linear_filter_5x5(const CamLinearFilterKernel &ker);
%rename("linear_filter_abs_3x3!") CamImage::linear_filter_abs_3x3(const CamLinearFilterKernel &ker);
%rename("linear_filter_abs_5x5!") CamImage::linear_filter_abs_5x5(const CamLinearFilterKernel &ker);
%rename("sep_filter_3x3!") CamImage::sep_filter_3x3(const CamSepFilterKernel &ker);
%rename("sep_filter_5x5!") CamImage::sep_filter_5x5(const CamSepFilterKernel &ker);
%rename("sep_filter_7x7!") CamImage::sep_filter_7x7(const CamSepFilterKernel &ker);
%rename("sep_filter_abs_3x3!") CamImage::sep_filter_abs_3x3(const CamSepFilterKernel &ker);
%rename("sep_filter_abs_5x5!") CamImage::sep_filter_abs_5x5(const CamSepFilterKernel &ker);
%rename("sep_filter_abs_7x7!") CamImage::sep_filter_abs_7x7(const CamSepFilterKernel &ker);
%rename("sobel_v!") CamImage::sobel_v();
%rename("sobel_h!") CamImage::sobel_h();
%rename("sobel_v_abs!") CamImage::sobel_v_abs();
%rename("sobel_h_abs!") CamImage::sobel_h_abs();
%rename("fixed_filter!") CamImage::fixed_filter(int filter);
%rename("hierarchical_watershed_regions!") CamImage::hierarchical_watershed_regions(const CamTableOfBasins &tob);

%rename("arithm!") CamImage::arithm(int operation, int c1=0, int c2=0, int c3=0);
%rename("draw_keypoints!") CamImage::draw_keypoints(const CamKeypoints &points, int color = 255);
%rename("draw_keypoint!") CamImage::draw_keypoint(const CamKeypoints &points, int color = 255);

%rename("apply_lut!") CamImage::apply_lut(const CamTable &LUT);
%rename("apply_lut!") CamRLEImage::apply_lut(const CamTable &LUT);

%rename("inverse!") CamRLEImage::inverse();
%rename("labeling!") CamRLEImage::labeling(); 
%rename("labeling!") CamRLEImage::labeling(CamBlobs &results);

// Predicates
%rename("allocated?") CamImage::allocated() const;

// Other renaming
%rename("cam_rgb") camRGB(int r, int g, int b);
%rename("cam_rgba") camRGBA(int r, int g, int b);
%rename("cam_set_viewer") camSetImageViewer;
%rename("nb_runs") CamRLEImage::nbRuns;
%rename("nb_blobs") CamBlobs::nbBlobs;
%rename("nb_matches") CamKeypointsMatches::nbMatches;
%rename("nb_outliers") CamKeypointsMatches::nbOutliers;
%rename("nb_points") CamKeypoints::nbPoints;

// Aliases
%alias CamRLEImage::labeling "labelling!";
%alias CamImage::set_mask "mask=";
%alias CamImage::copy "copy_to";
%alias CamImage::set "fill_with";
%alias CamImage::to_y "to_grey";
%alias CamKeypoint::set_descriptor "descriptor=";

// Ruby idioms
%alias CamImage::get_pixels "to_s";
%alias CamImage::clone "dup";

%newobject CamImage::clone;
%newobject CamImage::encode;
%newobject CamImage::encode_threshold;
%newobject CamImage::encode_threshold_inv;
%newobject CamImage::encode_threshold_lut;
%newobject CamImage::labeling;
%newobject CamImage::to_yuv;
%newobject CamImage::to_y;
%newobject CamImage::to_rgb;
%newobject CamImage::to_hls;
%newobject CamImage::integral_image;
%newobject CamRLEImage::erode_cross;
%newobject CamRLEImage::erode_3x3;
%newobject CamRLEImage::erode_3x2;

// Garbage collection
%markfunc CamImage "mark_CamImage";
%markfunc CamKeypointsMatch "mark_CamKeypointsMatch";
%markfunc CamKeypointsMatches "mark_CamKeypointsMatches";
%markfunc CamKeypoints "mark_CamKeypoints";
%markfunc CamKeypoint "mark_CamKeypoint";

// Typemaps

// Correct string allocation for to_s
%cstring_output_allocate_size(char **result, int *len, delete[](*$1));

%include "typemaps.i"
%apply int *OUTPUT { int &xc };
%apply int *OUTPUT { int &yc };
%apply int *OUTPUT { int &rc };
%apply int *OUTPUT { int *dist1 };
%apply int *OUTPUT { int *dist2 };
%apply int *OUTPUT { int *error };

%typemap(in) (const char *pixels, int sz)
{
    $1=STR2CSTR($input);
    $2=(int)RSTRING($input)->len;
}

%typemap(in) (const int* const array, int sz)
{
    Check_Type($input, T_ARRAY);
    $2=RARRAY($input)->len;
    if ($2) {
        $1=new int[$2];
        for (int i=0;i!=$2;i++) {
            VALUE inst = rb_ary_entry($input, i);
            $1[i]=NUM2INT(inst);
        }
    } else {
        $1=NULL;
    }
}

%typemap(freearg) (const int* const array, int sz)
{
    if ($1) delete[] $1;
}

%typemap(in) (const CamKeypoints **models, int nbModels)
{
    Check_Type($input, T_ARRAY);
    $2=RARRAY($input)->len;
    if ($2) {
        $1=new CamKeypoints*[$2];
        for (int i=0;i!=$2;i++) {
            VALUE inst = rb_ary_entry($input, i);
            void *argp;
            int res = SWIG_ConvertPtr(inst, &argp, SWIGTYPE_p_CamKeypoints,  0 );
            if (!SWIG_IsOK(res)) {
                SWIG_exception_fail(SWIG_ArgError(res), "Array of CamKeypoints expected"); 
            }
            $1[i]=reinterpret_cast< CamKeypoints * >(argp);
        }
    } else {
        $1=NULL;
    }
}

%typemap(freearg) (const CamKeypoints **models, int nbModels)
{
    if ($1) delete[] $1;
}

%typemap(out) double [ANY] 
{
    VALUE arr = rb_ary_new2($1_dim0);
    for (int i = 0; i < $1_dim0; i++) {
        rb_ary_push(arr, rb_float_new($1[i]));
    }
    $result = arr;
}

%typemap(out) int [ANY] 
{
    VALUE arr = rb_ary_new2($1_dim0);
    for (int i = 0; i < $1_dim0; i++) {
        rb_ary_push(arr, INT2FIX($1[i]));
    }
    $result = arr;
}

// Include header

%rename(CamCapture2) CamCapture;
%include "../inc/camellia.h"

// Additional definitions

#define CAM_COLORMODEL_GREY 0
#define CAM_COLORMODEL_RGB  1
#define CAM_COLORMODEL_RGBA 2
#define CAM_COLORMODEL_YUV  3
#define CAM_CHANNELSEQ_GREY 0
#define CAM_CHANNELSEQ_RGB  1
#define CAM_CHANNELSEQ_RGBA 2
#define CAM_CHANNELSEQ_YUV  3
#define CAM_CHANNELSEQ_BGR  4
#define CAM_CHANNELSEQ_BGRA 5

%mixin CamTable "Enumerable";

struct CamTable {
    int size;              ///< Number of valid entries
    CamTable(int s=0);
    bool set(const int* const array, int sz);
};

%extend CamTable {
    int __getitem__(int n) {
        if ((n>=0)&&(n<self->size)&&(n<CAM_TABLE_SIZE)) return self->t[n];
        else camError("CamTable","Index out of range");
        return self->t[0];
    };
    void __setitem__(int n, int val) {
        if ((n>=0)&&(n<self->size)&&(n<CAM_TABLE_SIZE)) self->t[n]=val;
        else camError("CamTable","Index out of range");
    };
    void each() {
        for (int i=0;i<self->size;i++) {
            VALUE obj = INT2FIX(self->t[i]);
            if (obj != Qnil) {
                rb_yield(obj);
            }
        }
    };
};

%mutable;
struct CamRLEImage {
    int id;             ///< Frame id (user dependent)
%immutable;
    int height;                 ///< Image height
    int width;                  ///< Image width
    int nbRuns;                 ///< The number of runs
    int allocated;              ///< Number of runs allocated
%mutable;

    CamRLEImage();                                      ///< Default constructor
    CamRLEImage(int nbruns);                            ///< Constructor with max number of runs parameter
    CamRLEImage(const CamRLEImage &image);              ///< Copy constructor
    ~CamRLEImage();                                     ///< Default destructor

    CamRLEImage* clone() const;                         ///< Clone RLE image (using CamRLEClone() function)
    bool alloc(int nbruns);                             ///< Allocator (C++ wrapping of CamRLEAllocate() function)
    bool realloc(int nbruns);                           ///< Reallocator (C++ wrapping of CamRLEReallocate() function)

    bool encode(const CamImage &image);                             ///< C++ wrapping for CamRLEEncode() function
    bool encode_lut(const CamImage &image, const CamTable &LUT);    ///< C++ wrapping for CamRLEEncodeLUT() function
    bool encode_threshold(const CamImage &image, int threshold);    ///< C++ wrapping for CamRLEEncodeThreshold() function
    bool encode_threshold_inv(const CamImage &image, int threshold);///< C++ wrapping for CamRLEEncodeThresholdInv() function
    bool encode_color(const CamImage &image, const CamTable &clusters); ///< C++ wrapping for camCamRLEEncodeColor() function
    CamBlobs* labeling();                                           ///< C++ wrapping for CamRLELabeling() function
    bool labeling(CamBlobs &results);                               ///< C++ wrapping for CamRLELabeling() function
    bool blob_analysis(CamBlobs &results) const;                    ///< C++ wrapping for CamRLEBlobAnalysis() function
    bool apply_lut(const CamTable &LUT);                            ///< C++ wrapping for CamRLEApplyLUT() function
    bool apply_lut(CamRLEImage &dest, const CamTable &LUT) const;   ///< C++ wrapping for CamRLEApplyLUT() function
    bool decode(CamImage &dest) const;                              ///< C++ wrapping for CamRLEDecode() function            
    bool decode(CamImage &dest, const CamTable &LUT) const;         ///< C++ wrapping for CamRLEDecode() function
    bool decode_blobs(CamImage &dest) const;                        ///< C++ wrapping for CamRLEDecodeBlobs() function            
    bool decode_blobs(CamImage &dest, const CamTable &LUT) const;   ///< C++ wrapping for CamRLEDecodeBlos() function
    bool inverse();                                                 ///< C++ wrapping for CamRLEInverse() function
    bool erode_cross(CamRLEImage &dest) const;                      ///< C++ wrapping for CamRLEErodeCross() function 
    CamRLEImage *erode_cross() const;                               ///< C++ wrapping for CamRLEErodeCross() function 
    bool erode_3x3(CamRLEImage &dest) const;                        ///< C++ wrapping for CamRLEErode3x3() function 
    CamRLEImage *erode_3x3() const;                                 ///< C++ wrapping for CamRLEErode3x3() function 
    bool erode_3x2(CamRLEImage &dest) const;                        ///< C++ wrapping for CamRLEErode3x2() function 
    CamRLEImage *erode_3x2() const;                                 ///< C++ wrapping for CamRLEErode3x2() function 
};

%immutable;
typedef struct {
    int id;
    int left;           ///< Leftmost coordinate of the blob
    int top;            ///< Topmost coordinate of the blob
    int width;          ///< Width of the blob
    int height;         ///< Height of the blob
    int surface;        ///< Number of pixels covered by this blob
    int cx;             ///< Center of gravity (x)
    int cy;             ///< Center of gravity (y)
    int value;          ///< Blob value, or average pixel value in the original image
    int min;            ///< Minimum pixel value in the original image
    int max;            ///< Maximum pixel value in the original image
    void *misc;         ///< Additional user-dependant blob information
} CamBlobInfo;

%mixin CamBlobs "Enumerable";

struct CamBlobs {
    int nbBlobs;            ///< Number of valid blobs
    CamBlobs() {nbBlobs=0;} ///< Default constructor
};

%extend CamBlobs {
    CamBlobInfo __getitem__(int n) {
        if ((n>=0)&&(n<self->nbBlobs)) return self->blobInfo[n];
        else camError("CamBlobs","Index out of range");
        return self->blobInfo[0];
    };
    void each() {
        for (int i=0;i<self->nbBlobs;i++) {
            VALUE obj = SWIG_NewPointerObj((new CamBlobInfo(self->blobInfo[i])), SWIGTYPE_p_CamBlobInfo, SWIG_POINTER_OWN);
            if (obj != Qnil) {
                rb_yield(obj);
            }
        }
    };
};

%mixin CamTableOfBasins "Enumerable";

struct CamTableOfBasins
{
    int nbBasins;
    void get_rid_of(CamBasin &basin) {basin.surface=-1;}
};

%extend CamTableOfBasins {
    CamBasin* __getitem__(int n) {
        if ((n >= 0) && (n < self->nbBasins)) return &self->tab[n];
        else camError("CamTableOfBasins","Index out of range");
        return &self->tab[0];
    };
    void each() {
        for (int i = 0;i < self->nbBasins; i++) {
            VALUE obj = SWIG_NewPointerObj(&self->tab[i], SWIGTYPE_p_CamBasin, 0 );
            if (obj != Qnil) {
                rb_yield(obj);
            }
        }
    };
};

%mixin CamKeypoints "Enumerable";

%extend CamKeypoints {
    CamKeypoint* __getitem__(int n) {
        if (n >= 0 && n < self->nbPoints) return self->keypoint[n];
        else camError("CamKeypoints","Index out of range");
        return NULL;
    };
    void each() {
        for (int i=0; i < self->nbPoints; i++) {
            VALUE obj = SWIG_NewPointerObj(self->keypoint[i], SWIGTYPE_p_CamKeypoint, 0 );
            if (obj != Qnil) {
                rb_yield(obj);
            }
        }
    };
};

%mixin CamKeypointsMatches "Enumerable";

%extend CamKeypointsMatches {
    CamKeypointsMatch* __getitem__(int n) {
        if (n >= 0 && n < self->nbMatches) return &(self->pairs[n]);
        else camError("CamKeypointsMatches","Index out of range");
        return NULL;
    };
    void each() {
        for (int i=0; i < self->nbMatches; i++) {
            VALUE obj = SWIG_NewPointerObj(&(self->pairs[i]), SWIGTYPE_p_CamKeypointsMatch, 0 );
            if (obj != Qnil) {
                rb_yield(obj);
            }
        }
    };
};

%mutable;

// Garbage collection implementation (mark & sweep)
%header %{
static void mark_CamImage(void *ptr) {
    CamImage *image=(CamImage*)ptr;
    //printf("Marking CamImage %x\n", ptr); 
    if (image->roi) {
        VALUE object = SWIG_RubyInstanceFor(image->roi);
        if (object != Qnil) {
            rb_gc_mark(object);
        }
    }
    if (image->mask) {
        VALUE object = SWIG_RubyInstanceFor(image->mask);
        if (object != Qnil) {
            rb_gc_mark(object);
        }
    }
    if (image->imageData !=NULL && image->imageDataOrigin == NULL) {
        // Image references a Ruby string
        rb_gc_mark((VALUE)image->imageId);
    }
}

static void mark_CamKeypoints(void *ptr) {
    int i;
    //printf("Marking keypoints %x\n", ptr); 
    CamKeypoints *points = (CamKeypoints*)ptr;
    for (i = 0; i < points->nbPoints; i++) {
        VALUE object = SWIG_RubyInstanceFor(points->keypoint[i]);
        if (object != Qnil) {
            rb_gc_mark(object);
        }
    }
}

static void mark_CamKeypoint(void *ptr) {
    //printf("Marking keypoint %x\n", ptr); 
    CamKeypoint *point = (CamKeypoint*)ptr;
    if (point->set) {
        VALUE object = SWIG_RubyInstanceFor(point->set);
        if (object != Qnil) {
            rb_gc_mark(object);
        }
    }
}

static void mark_CamKeypointsMatch(void *ptr) {
    CamKeypointsMatch *match = (CamKeypointsMatch*)ptr;
    //printf("Marking match %x\n", ptr); 
    VALUE object = SWIG_RubyInstanceFor(match->p1);
    if (object != Qnil) {
        rb_gc_mark(object);
    }
    object = SWIG_RubyInstanceFor(match->p2);
    if (object != Qnil) {
        rb_gc_mark(object);
    }
}

static void mark_CamKeypointsMatches(void *ptr) {
    int i;
    CamKeypointsMatches *matches = (CamKeypointsMatches*)ptr;
    //printf("Marking matches %x\n", ptr); 
    for (i = 0; i < matches->nbMatches; i++) {
        VALUE object = SWIG_RubyInstanceFor(matches->pairs[i].p1);
        if (object != Qnil) {
            rb_gc_mark(object);
        }
        object = SWIG_RubyInstanceFor(matches->pairs[i].p2);
        if (object != Qnil) {
            rb_gc_mark(object);
        }
    }   
}

%}

// Exception handling
%header %{
void camErrorRuby(char *module, char *error)
{
    rb_raise(rb_eRuntimeError,"Error in %s : %s",module,error);
}

static VALUE camellia_set_image_data(VALUE self, VALUE str)
{
    VALUE strx;
    CamImage *image;
    void *argp1 = 0 ;

    SWIG_ConvertPtr(self, &argp1, SWIGTYPE_p_CamImage, 0 |  0 );
    image = reinterpret_cast< CamImage * >(argp1);
    strx = StringValue(str);
    if (image->imageData) {
        camDeallocateImage(image);
    }
    image->imageId = (void*)strx; // Reference to the Ruby string kept for Mark & Sweep
    image->imageData = (unsigned char*)RSTRING(strx)->ptr;
    return self;
}
%}

// Set ruby error function
%init %{
    camSetErrorFunct(camErrorRuby);
    rb_define_method(cCamImage.klass, "set_image_data", VALUEFUNC(camellia_set_image_data), 1);
%}



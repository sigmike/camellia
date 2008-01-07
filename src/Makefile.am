AUTOMAKE_OPTIONS = gnu
lib_LTLIBRARIES = libCamellia.la
libCamellia_la_SOURCES = \
		camellia.cpp \
		cam_3d.c \
		cam_arithmetics.c \
		cam_blob_analysis.c \
		cam_capture.cpp \
		cam_draw.c \
		cam_keypoints.c \
		cam_keypoints_matching.c \
		cam_harris.c \
		cam_histogram.c \
		cam_hls.c \
		cam_hough.c \
		cam_hvsumming.c \
		cam_integralimage.c \
		cam_io.c \
		cam_labelling.c \
		cam_linear_filtering.c \
		cam_LUT.c \
		cam_ME.c \
		cam_measures.c \
		cam_median_filtering.c \
		cam_morphomaths.c \
		cam_RLE_labelling.c \
		cam_RLE_morpho.c \
		cam_RLE_utils.c \
		cam_SAD.c \
		cam_utils.c \
		cam_volberg.c \
		cam_warping.c \
		cam_watershed.c \
		cam_yuv.c \
		cam_error.c

noinst_HEADERS = ../inc/camellia_internals.h ../inc/cam_capture.h
include_HEADERS = ../inc/camellia.h
libCamellia_la_LDFLAGS = -no-undefined -version-info 0:0:0

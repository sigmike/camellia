#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "camellia.h"
#include "cv.h"
#ifdef USE_LIBMORPHO
#include "libmorpho.h"
#endif

#define MAX_NEIGHB 7
#define MAX_RUNS 10

typedef struct {
    int line;
    int start;
    int length;
} StructEltRun;

typedef struct {
    int nblines;
    int nbruns;
    StructEltRun runs[MAX_RUNS];
} StructElement;

int sortStructEltRuns(const void *p1, const void *p2)
{
    StructEltRun *r1 = (StructEltRun*)p1;
    StructEltRun *r2 = (StructEltRun*)p2;
    if (r1->length > r2->length) return -1;
    if (r1->length < r2->length) return 1;
    return 0;
}

#define max_using_if(val)	if (val[1] > val[0]) val[0] = val[1]
#define max_using_boolean(val)	val[0] = val[val[1] > val[0]]
#define max_using_arithm(val)	val[0] = val[(((unsigned int)val[0] - val[1])) >> 31]

#define GENERATE_DILATION_NAIVE(suffix, PIXEL, max_method) \
void camDilationNaive##suffix(CamImage *source, CamImage *dest, StructElement *elt)		\
{												\
    PIXEL *srcptr, *dstptr;									\
    int i, j, x, y, val[2];									\
    int offset[MAX_NEIGHB * MAX_NEIGHB], n;							\
												\
    /* Algorithm initialization */								\
    for (n = 0, i = 0; i < elt->nbruns; i++) {							\
	for (j = 0; j < elt->runs[i].length; j++) {							\
	    offset[n++] = elt->runs[i].start + j + source->widthStep * elt->runs[i].line / sizeof(PIXEL);	\
	}											\
    }												\
												\
    /* Frame scan */										\
    for (y = 0; y < source->height - elt->nblines + 1; y++) {					\
	srcptr = (PIXEL*)(source->imageData + y * source->widthStep);				\
	dstptr = (PIXEL*)(dest->imageData + y * dest->widthStep);				\
												\
	for (x = 0; x < source->width - elt->nblines + 1; x++, dstptr++, srcptr++) {		\
	    val[0] = *(srcptr + offset[0]);							\
	    for (i = 1; i != n; i++) {								\
		val[1] = *(srcptr + offset[i]);							\
		max_method(val);								\
	    }											\
	    *dstptr = val[0];									\
	}											\
    }												\
}

GENERATE_DILATION_NAIVE(8, unsigned char, max_using_if);
GENERATE_DILATION_NAIVE(8V2, unsigned char, max_using_boolean);
GENERATE_DILATION_NAIVE(8V3, unsigned char, max_using_arithm);
GENERATE_DILATION_NAIVE(16, signed short, max_using_if);
GENERATE_DILATION_NAIVE(16V2, signed short, max_using_boolean);
GENERATE_DILATION_NAIVE(16V3, signed short, max_using_arithm);

#define MAXLIST_SIZE 4
#define MAXLIST_MASK ((1 << MAXLIST_SIZE) - 1)
#define MAXLIST_LENGTH (1 << MAXLIST_SIZE)
#define MAXLIST_BIGNUM 65536

#define max_method max_using_if
#define PIXEL unsigned char
void camDilationMaxlist2DV2(CamImage *source, CamImage *dest, StructElement *elt)
{
    PIXEL *srcptr, *dstptr;
    int i, j, k, l, x, y, start, end, val[2], maxlist[MAXLIST_LENGTH];
    int offset[MAX_RUNS], maxlength;    

    /* Algorithm initialization */
    maxlength = 0;
    for (i = 0; i != elt->nbruns; i++) {							
	offset[i] = elt->runs[i].start + elt->runs[i].length - 1 + source->widthStep * elt->runs[i].line / sizeof(PIXEL);
	if (elt->runs[i].length > maxlength) maxlength = elt->runs[i].length;
    }												
    for (i = 0; i != MAXLIST_LENGTH; i++) maxlist[i] = 0;

    /* Frame scan */										
    for (y = 0; y < source->height - elt->nblines + 1; y++) {					
	srcptr = (PIXEL*)(source->imageData + y * source->widthStep);				
	dstptr = (PIXEL*)(dest->imageData + y * dest->widthStep);				
	start = 0;
	end = maxlength - 1;
	maxlist[MAXLIST_LENGTH - 1] = MAXLIST_BIGNUM;

	/* Initialize maxlist */
	for (i = 0; i != end; i++) maxlist[i] = 0;
	for (i = 0; i != elt->nbruns; i++) {
	    for (j = 0; j < elt->runs[i].length - 1; j++) {
		val[0] = *((PIXEL*)(srcptr + source->widthStep * elt->runs[i].line) + elt->runs[i].start + j);
		k = 0;
		while (val[0] > maxlist[(j - k) & MAXLIST_MASK]) {
		    maxlist[(j - k) & MAXLIST_MASK] = val[0];
		    k++;
		}
	    }
	}

	/* Line scan */
	for (x = 0; x < source->width - elt->nblines + 1; x++, dstptr++, srcptr++) {		
	    val[0] = *(srcptr + offset[0]);
	    l = elt->runs[0].length;
	    for (i = 1; elt->runs[i].length == l; i++) {
		val[1] = *(srcptr + offset[i]);
		max_method(val);
	    }
	    maxlist[end] = val[0];
	    for (j = 1; maxlist[(end - j) & MAXLIST_MASK] < val[0]; j++)
		maxlist[(end - j) & MAXLIST_MASK] = val[0];
	    while (i != elt->nbruns) {	
		val[0] = *(srcptr + offset[i]);
		l = elt->runs[i].length;
		for (; i != elt->nbruns && elt->runs[i].length == l; i++) {
		    val[1] = *(srcptr + offset[i]);
		    max_method(val);
		}
		for (j = l - 1; maxlist[(start + j) & MAXLIST_MASK] < val[0]; j--)
		    maxlist[(start + j) & MAXLIST_MASK] = val[0];
	    }
	    *dstptr = maxlist[start];		
	    maxlist[start] = MAXLIST_BIGNUM;	    
	    start = (start + 1) & MAXLIST_MASK;
	    end = (end + 1) & MAXLIST_MASK;
	}											
    }												
}

typedef struct _CamMaxlistElt{
    struct _CamMaxlistElt *prev;
    struct _CamMaxlistElt *next;
    int value;
    int position;
} CamMaxlistElt;

void camDilationMaxlist2D(CamImage *source, CamImage *dest, StructElement *elt)
{
    PIXEL *srcptr, *dstptr;
    int i, j, l, x, y, nbFreeElts, val, valmax;
    CamMaxlistElt maxlist[MAX_NEIGHB], *freeElts[MAX_NEIGHB], *start, *end, *ptr;
    int offset[MAX_RUNS], maxlength, v[MAX_NEIGHB - 1];    

    // Algorithm initialization
    maxlength = 0;
    for (i = 0; i != elt->nbruns; i++) {							
	offset[i] = elt->runs[i].start + elt->runs[i].length - 1 + source->widthStep * elt->runs[i].line / sizeof(PIXEL);
	if (elt->runs[i].length > maxlength) maxlength = elt->runs[i].length;
    }												

    // Frame scan									
    for (y = 0; y < source->height - elt->nblines + 1; y++) {					
	srcptr = (PIXEL*)(source->imageData + y * source->widthStep);				
	dstptr = (PIXEL*)(dest->imageData + y * dest->widthStep);				

	// Initialize maxlist
	for (i = 0; i != maxlength - 1; i++) v[i] = 0;
	for (i = 0; i != elt->nbruns; i++) {
	    for (j = 0; j != elt->runs[i].length - 1; j++) {
		val = *((PIXEL*)(srcptr + source->widthStep * elt->runs[i].line) + elt->runs[i].start + j);
		if (val > v[j]) v[j] = val;
	    }
	}
	// Insert first element
	maxlist[0].prev = NULL;
	maxlist[0].next = NULL;
	maxlist[0].value = v[0];
	maxlist[0].position = 0;
	start = &maxlist[0];
	end = &maxlist[0];
	for (i = 1; i != maxlength; i++) freeElts[i - 1] = &maxlist[i];
	nbFreeElts = maxlength - 1;
	// Insert next elements
	for (i = 1; i != maxlength - 1; i++) {
	    if (v[i] >= end->value) {
		// Update the last value
		end->value = v[i];
		end->position = i;
		ptr = end->prev;
		if (ptr != NULL) {
		    while (v[i] >= ptr->value) {
			// Remove that element
			freeElts[nbFreeElts++] = ptr;
			if (ptr->prev == NULL) {
			    // It was the first in queue
			    ptr->next->prev = NULL;
			    start = ptr->next;
			    break;
			} else {
			    ptr->prev->next = ptr->next;
			    ptr->next->prev = ptr->prev;
			    ptr = ptr->prev;
			}
		    }
		}	
	    } else {
		// Should be appended to the queue
		freeElts[--nbFreeElts]->prev = end;
		end->next = freeElts[nbFreeElts];
		end = end->next;
		end->next = NULL;
		end->value = v[i];
		end->position = i;
	    }
	}

	/* Line scan */
	for (x = 0; x < source->width - elt->nblines + 1; x++, dstptr++, srcptr++) {		
	    valmax = *(srcptr + offset[0]);
	    l = elt->runs[0].length;
	    for (i = 1; elt->runs[i].length == l; i++) {
		val = *(srcptr + offset[i]);
		if (val > valmax) valmax = val;
	    }
	    // Update the queue
	    if (valmax >= end->value) {
		// Update the last value
		end->value = valmax;
		end->position = x + maxlength - 1;
		ptr = end->prev;
		if (ptr != NULL) {
		    while (valmax >= ptr->value) {
			// Remove that element
			freeElts[nbFreeElts++] = ptr;
			if (ptr->prev == NULL) {
			    // It was the first in queue
			    ptr->next->prev = NULL;
			    start = ptr->next;
			    break;
			} else {
			    ptr->prev->next = ptr->next;
			    ptr->next->prev = ptr->prev;
			    ptr = ptr->prev;
			}
		    }
		}	
	    } else {
		// Should be appended to the queue
		freeElts[--nbFreeElts]->prev = end;
		end->next = freeElts[nbFreeElts];
		end = end->next;
		end->next = NULL;
		end->value = valmax;
		end->position = x + maxlength - 1;
	    }

	    while (i != elt->nbruns) {	
		valmax = *(srcptr + offset[i]);
		l = elt->runs[i].length;
		for (; i != elt->nbruns && elt->runs[i].length == l; i++) {
		    val = *(srcptr + offset[i]);
		    if (val > valmax) valmax = val;
		}
		// Update the queue
		if (valmax > end->value) {
		    // Find the right place in the queue
		    ptr = end->prev;
		    if (ptr != NULL) {
			do {
			    if (x + l - 1 > ptr->position) {
				// We have reached a position left to this entry
    				// Insert it in the list
				freeElts[--nbFreeElts]->next = ptr->next;
				ptr->next = ptr->next->prev = freeElts[nbFreeElts];
				freeElts[nbFreeElts]->prev = ptr;
				freeElts[nbFreeElts]->value = valmax;
				freeElts[nbFreeElts]->position = x + l - 1;
				while (valmax >= ptr->value) {
				    // Remove that element
				    freeElts[nbFreeElts++] = ptr;
				    if (ptr->prev == NULL) {
					// It was the first in queue
					ptr->next->prev = NULL;
					start = ptr->next;
					break;
				    } else {
					ptr->prev->next = ptr->next;
					ptr->next->prev = ptr->prev;
					ptr = ptr->prev;
				    }
				}
				break;
			    } else {
				if (valmax > ptr->value) {
				    if (x + l - 1 == ptr->position) {
					// Update that element
					ptr->value = valmax;
					// Remove all elements before
					ptr = ptr->prev;
					if (ptr != NULL) {
					    while (valmax >= ptr->value) {
						// Remove that element
						freeElts[nbFreeElts++] = ptr;
						if (ptr->prev == NULL) {
						    // It was the first in queue
						    ptr->next->prev = NULL;
						    start = ptr->next;
						    break;
						} else {
						    ptr->prev->next = ptr->next;
						    ptr->next->prev = ptr->prev;
						    ptr = ptr->prev;
						}
					    }
					}
					break;
				    } else {
					ptr = ptr->prev;
					if (ptr == NULL) {
					    // It should be put first place in the queue
					    freeElts[--nbFreeElts]->next = start;
					    start = freeElts[nbFreeElts];
					    start->next->prev = start;
					    start->prev = NULL;
					    start->value = valmax;
					    start->position = x + l - 1;
					    break;
					}
				    }
				} else break; // do nothing with this value
			    }
			} while (1);
		    } else {
			// It should be put first place in the queue
			freeElts[--nbFreeElts]->next = start;
			start = freeElts[nbFreeElts];
			start->next->prev = start;
			start->prev = NULL;
			start->value = valmax;
			start->position = x + l - 1;		       
		    }
		} // else do nothing with this value
	    }

	    *dstptr = start->value;
	    if (start->position == x) {
		freeElts[nbFreeElts++] = start;
		start = start->next;
		start->prev = NULL;
	    }		
	}											
    }												
}

void camDilationHistogram(CamImage *source, CamImage *dest, StructElement *elt)
{
    PIXEL *srcptr, *dstptr;
    int i, j, x, y, val[2];
    int maximum, offset_left[MAX_RUNS], offset_right[MAX_RUNS];    
    int histogram[1 << (sizeof(PIXEL) * 8)];

    /* Algorithm initialization */
    for (i = 0; i != elt->nbruns; i++) {							
	offset_right[i] = elt->runs[i].start + elt->runs[i].length - 1 + source->widthStep * elt->runs[i].line / sizeof(PIXEL);
	offset_left[i] = elt->runs[i].start - 1 + source->widthStep * elt->runs[i].line / sizeof(PIXEL);
    }												

    /* Frame scan */										
    for (y = 0; y < source->height - elt->nblines + 1; y++) {					
	srcptr = (PIXEL*)(source->imageData + y * source->widthStep);				
	dstptr = (PIXEL*)(dest->imageData + y * dest->widthStep);				

	//for (i = 0; i != (1 << (sizeof(PIXEL) * 8)); i++) histogram[i] = 0;
	memset(histogram, 0, (1 << (sizeof(PIXEL) * 8)) * sizeof(int));
	val[0] = 0;
	for (i = 0; i != elt->nbruns; i++) {
	    for (j = 0; j != elt->runs[i].length; j++) {
		val[1] = *((PIXEL*)(srcptr + source->widthStep * elt->runs[i].line) + elt->runs[i].start + j);
		histogram[val[1]]++;
		max_method(val);
	    }
	}
	*dstptr++ = maximum = val[0];
	srcptr++;

	/* Line scan */
	for (x = 1; x < source->width - elt->nblines + 1; x++, dstptr++, srcptr++) {		
	    val[0] = maximum;
	    for (i = 0; i != elt->nbruns; i++) {
		val[1] = *(srcptr + offset_right[i]);
		histogram[val[1]]++;
		max_method(val);
		histogram[*(srcptr + offset_left[i])]--;
	    }
	    maximum = val[0];
	    if (histogram[maximum] == 0) {
		/* We have to look for the maximum */
		do maximum--; while (histogram[maximum] == 0); 
	    }
	    *dstptr = maximum;
	}	
    }												
}

void camDilationSmartNaive(CamImage *source, CamImage *dest, StructElement *elt)
{
    PIXEL *srcptr, *dstptr;
    int i, j, x, y, val[2];
    int maxlength, slw[MAX_NEIGHB], offset[MAX_RUNS], *off2[MAX_RUNS];    

    /* Algorithm initialization */
    maxlength = 0;
    for (i = 0; i != elt->nbruns; i++) {							
	offset[i] = elt->runs[i].start + elt->runs[i].length - 1 + source->widthStep * elt->runs[i].line / sizeof(PIXEL);
	off2[i] = &slw[elt->runs[i].length - 1];
	if (elt->runs[i].length > maxlength) maxlength = elt->runs[i].length;
    }												

    /* Frame scan */										
    for (y = 0; y < source->height - elt->nblines + 1; y++) {					
	srcptr = (PIXEL*)(source->imageData + y * source->widthStep);				
	dstptr = (PIXEL*)(dest->imageData + y * dest->widthStep);				

	for (i = 0; i != maxlength - 1; i++) slw[i] = 0;
	for (i = 0; i != elt->nbruns; i++) {
	    for (j = 0; j != elt->runs[i].length - 1; j++) {
		val[0] = slw[j];
		val[1] = *((PIXEL*)(srcptr + source->widthStep * elt->runs[i].line) + elt->runs[i].start + j);
		max_method(val);
		slw[j] = val[0];
	    }
	}

	/* Line scan */
	for (x = 0; x < source->width - elt->nblines + 1; x++, dstptr++, srcptr++) {		
	    /* Update sliding window */
	    slw[maxlength - 1] = *(srcptr + offset[0]);
	    for (i = 1; i!= elt->nbruns; i++) {
		// if (*(srcptr + offset[i]) > *off2[i]) *off2[i] = *(srcptr + offset[i]);
		val[0] = *off2[i];	
		val[1] = *(srcptr + offset[i]);
		max_method(val);
		*off2[i] = val[0];
	    }
	    /* Sliding window scan */
	    val[0] = slw[0];
	    for (i = 1; i != maxlength; i++) {
		val[1] = slw[i];
		max_method(val);
	    }	
	    for (i = 0; i != maxlength - 1; i++) slw[i] = slw[i + 1];
	    *dstptr = val[0];
	}	
    }												
}

#if defined(__SSE2__)
#include <emmintrin.h>

#ifdef __GNUC__
/* SSE2 implementation of Smart Naive with Horizontal Scan */
void camDilationSmartNaive8_SSE2_H(CamImage *source, CamImage *dest, StructElement *elt)
{
    PIXEL *srcptr, *dstptr, *tmpdstptr;
    int i, j, k, x, y;
    int maxlength, offset[MAX_RUNS][16];    
    __m128i val, slw[MAX_NEIGHB], *off2[MAX_RUNS]; 
    unsigned char pix[16];
    unsigned char out_sse[16] __attribute__((aligned(16)));

    /* Algorithm initialization */
    maxlength = 0;
    for (i = 0; i != elt->nbruns; i++) {							
	for (j = 0; j < 16; j++) {
	    offset[i][j] = elt->runs[i].start + elt->runs[i].length - 1 + source->widthStep * (elt->runs[i].line + j) / sizeof(PIXEL);
	}
	off2[i] = &slw[elt->runs[i].length - 1];
	if (elt->runs[i].length > maxlength) maxlength = elt->runs[i].length;
    }												

    /* Frame scan */										
    for (y = 0; y < source->height - elt->nblines + 1 - 16; y += 16) {					
	srcptr = (PIXEL*)(source->imageData + y * source->widthStep);				
	dstptr = (PIXEL*)(dest->imageData + y * dest->widthStep);				

	for (i = 0; i != maxlength; i++) slw[i] = _mm_setzero_si128();
	for (i = 0; i != elt->nbruns; i++) {
	    for (j = 0; j != elt->runs[i].length - 1; j++) {
		for (k = 0; k != 16; k++) {
		    pix[k] = *((PIXEL*)(srcptr + source->widthStep * (elt->runs[i].line + k)) + elt->runs[i].start + j);
		}
		val = _mm_setr_epi8((char)pix[0], (char)pix[1], (char)pix[2], (char)pix[3],
			     (char)pix[4], (char)pix[5], (char)pix[6], (char)pix[7],
			     (char)pix[8], (char)pix[9], (char)pix[10], (char)pix[11],
			     (char)pix[12], (char)pix[13], (char)pix[14], (char)pix[15]);
		slw[j] = _mm_max_epu8(slw[j], val);
	    }
	}

	/* Line scan */
	for (x = 0; x < source->width - elt->nblines + 1; x++, dstptr++, srcptr++) {		
	    /* Update sliding window */
	    for (i = 0; i!= elt->nbruns; i++) {
		for (k = 0; k != 16; k++) pix[k] = *(srcptr + offset[i][k]);
		val = _mm_setr_epi8((char)pix[0], (char)pix[1], (char)pix[2], (char)pix[3],
				   (char)pix[4], (char)pix[5], (char)pix[6], (char)pix[7],
				   (char)pix[8], (char)pix[9], (char)pix[10], (char)pix[11],
				   (char)pix[12], (char)pix[13], (char)pix[14], (char)pix[15]);
		*off2[i] = _mm_max_epu8(*off2[i], val);
	    }
	    /* Sliding window scan */
	    val = slw[0];
	    for (i = 1; i != maxlength; i++) {
		val = _mm_max_epu8(slw[i], val);
	    }	
	    _mm_store_si128((__m128i*)out_sse, val);
	    tmpdstptr = dstptr;
	    for (i = 0; i != 16; i++, dstptr += dest->widthStep)
		*dstptr = out_sse[i];
	    dstptr = tmpdstptr;
	    for (i = 0; i != maxlength - 1; i++) slw[i] = slw[i + 1];
	    slw[maxlength - 1] = _mm_setzero_si128();	
	}	
    }												
}
#endif

/* SSE2 implementation of Smart Naive with Vertical Scan */
void camDilationSmartNaive8_SSE2(CamImage *source, CamImage *dest, StructElement *elt)
{
    PIXEL *srcptr, *dstptr;
    int i, j, x, y;
    int maxlength, offset[MAX_RUNS];    
    __m128i val, slw[MAX_NEIGHB], *off2[MAX_RUNS]; 

    /* Algorithm initialization */
    maxlength = 0;
    for (i = 0; i != elt->nbruns; i++) {							
	offset[i] = elt->runs[i].line + source->widthStep * (elt->runs[i].start + elt->runs[i].length - 1);
	off2[i] = &slw[elt->runs[i].length - 1];
	if (elt->runs[i].length > maxlength) maxlength = elt->runs[i].length;
    }												

    /* Frame scan */										
    for (x = 0; x < source->width - elt->nblines + 1 - 16; x+= 16) {		
	srcptr = (PIXEL*)(source->imageData + x);				
	dstptr = (PIXEL*)(dest->imageData + x);				

	for (i = 0; i != maxlength; i++) slw[i] = _mm_setzero_si128();
	for (i = 0; i != elt->nbruns; i++) {
	    for (j = 0; j != elt->runs[i].length - 1; j++)
		slw[j] = _mm_max_epu8(slw[j], _mm_loadu_si128((__m128i*)((PIXEL*)(srcptr + source->widthStep * (elt->runs[i].start + j)) + elt->runs[i].line)));
	}

	/* vertical scan */
	for (y = 0; y < source->height - elt->nblines + 1; y++) {					
	    /* Update sliding window */
	    for (i = 0; i!= elt->nbruns; i++)
		*off2[i] = _mm_max_epu8(*off2[i], _mm_loadu_si128((__m128i*)(srcptr + offset[i])));
	    /* Sliding window scan */
	    val = slw[0];
	    for (i = 1; i != maxlength; i++) {
		val = _mm_max_epu8(slw[i], val);
	    }	
	    _mm_storeu_si128((__m128i*)dstptr, val);
	    dstptr += dest->widthStep;
	    srcptr += source->widthStep;
	    for (i = 0; i != maxlength - 1; i++) slw[i] = slw[i + 1];
	    slw[maxlength - 1] = _mm_setzero_si128();	
	}	
    }												
}
#endif

// Benchmarking functions
#ifdef _WIN32
#include <windows.h>
#include <mmsystem.h>
void camInitBenchmark() {timeBeginPeriod(1);}
int camGetTimeMs() {return timeGetTime();}
#else
#include <sys/time.h>
#include <unistd.h>
void camInitBenchmark() {}
int camGetTimeMs()
{
    int t;
    struct timeval tv;
    gettimeofday(&tv,NULL);
    t=tv.tv_sec*1000+tv.tv_usec/1000;
    return t;
}
#endif

const int Circle5StructElt_start[5] = {1, 0, 0, 0, 1};
const int Circle5StructElt_length[5] = {3, 5, 5, 5, 3};

const int Circle7StructElt_start[7] = {2, 1, 0, 0, 0, 1, 2};
const int Circle7StructElt_length[7] = {3, 5, 7, 7, 7, 5, 3};

const char *test_elt_names[]= {
    "Circle5", "Circle7"
};

typedef void (*CamDilationFunc)(CamImage *source, CamImage *dest, StructElement *elt);
typedef struct {
    CamDilationFunc func;
    const char *name;
} TestFunction;
#define FUNC(func) { func, #func }

#define NB_TESTS 6
TestFunction test_funcs[NB_TESTS] = {
    FUNC(camDilationMaxlist2D), 
    FUNC(camDilationMaxlist2DV2), 
    FUNC(camDilationSmartNaive), FUNC(camDilationSmartNaive8_SSE2),
    FUNC(camDilationNaive8), //FUNC(camDilationNaive8V2), FUNC(camDilationNaive8V3),
    FUNC(camDilationHistogram)
};

unsigned char se_Circle5[25] = {
    0,1,1,1,0,
    1,1,1,1,1,
    1,1,1,1,1,
    1,1,1,1,1,
    0,1,1,1,0
};

unsigned char se_Circle7[49] = {
    0,0,1,1,1,0,0,
    0,1,1,1,1,1,0,
    1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,
    0,1,1,1,1,1,0,
    0,0,1,1,1,0,0
};

int main()
{
    CamImage source, y, dest;
    int i, j, n, start, stop;
    StructElement circle5, circle7;
    char filename[256];
    StructElement *test_elts[] = {&circle5, &circle7};
    IplConvKernel *element;

    // Structural elements initialization
    circle5.nbruns = 5;
    circle5.nblines = 5;
    for (i = 0; i < 5; i++) {
	circle5.runs[i].line = i;
	circle5.runs[i].start = Circle5StructElt_start[i];
	circle5.runs[i].length = Circle5StructElt_length[i];
    }
    qsort(circle5.runs, circle5.nbruns, sizeof(StructEltRun), sortStructEltRuns);
    circle7.nbruns = 7;
    circle7.nblines = 7;
    for (i = 0; i < 7; i++) {
	circle7.runs[i].line = i;
	circle7.runs[i].start = Circle7StructElt_start[i];
	circle7.runs[i].length = Circle7StructElt_length[i];
    }
    qsort(circle7.runs, circle7.nbruns, sizeof(StructEltRun), sortStructEltRuns);

    camInitBenchmark();
    camLoadBMP(&source, "../../resources/photos/mrpotato4.bmp");
    camAllocateImage(&y, source.width, source.height, CAM_DEPTH_8U);
    camRGB2Y(&source, &y);
    
    for (n = 0; n < 2; n++) {
	camAllocateImage(&dest, source.width - test_elts[n]->nblines + 1, source.height - test_elts[n]->nblines + 1, CAM_DEPTH_8U);
	for (j = 0; j < NB_TESTS; j++) {
	    camSet(&dest, 0);
	    start = camGetTimeMs();
	    for (i = 0; i < 10; i++) {
		(*test_funcs[j].func)(&y, &dest, test_elts[n]);
	    }
	    stop = camGetTimeMs();
	    printf("%s on %s = %dus\n", test_funcs[j].name, test_elt_names[n], (stop - start) * 100);
	    sprintf(filename, "../../output/mrpotato_%s_%s.pgm", test_funcs[j].name, test_elt_names[n]);
	    camSavePGM(&dest, filename);
	}
    }

    // Tests with OpenCV
    camDeallocateImage(&dest);
    camAllocateImage(&dest, source.width, source.height, CAM_DEPTH_8U);

    element=cvCreateStructuringElementEx(5,5,3,3,CV_SHAPE_ELLIPSE,NULL);
    start = camGetTimeMs();
    for (i = 0; i < 10; i++) {
        cvDilate(&y, &dest, element, 1);
    }
    stop = camGetTimeMs();
    printf("cvDilate on Circle5 = %dus\n", (stop - start) * 100); 
    camSavePGM(&dest,"../../output/mrpotato_cvDilate_Circle5.pgm");
    cvReleaseStructuringElement(&element);
    
    element=cvCreateStructuringElementEx(7,7,3,3,CV_SHAPE_ELLIPSE,NULL);
    start = camGetTimeMs();
    for (i = 0; i < 10; i++) {
        cvDilate(&y, &dest, element, 1);
    }
    stop = camGetTimeMs();
    printf("cvDilate on Circle7 = %dus\n", (stop - start) * 100); 
    camSavePGM(&dest,"../../output/mrpotato_cvDilate_Circle7.pgm");
    cvReleaseStructuringElement(&element);
   
#ifdef USE_LIBMORPHO
    // Tests with libmorpho
    start = camGetTimeMs();
    for (i = 0; i < 10; i++) {
	dilation_arbitrary_SE(y.imageData, dest.imageData, source.width, source.height, se_Circle5, 5, 5, 2, 2);
    }
    stop = camGetTimeMs();
    printf("libmorpho on Circle5 = %dus\n", (stop - start) * 100); 
    camSavePGM(&dest,"../../output/mrpotato_libmorpho_Circle5.pgm");
    
    start = camGetTimeMs();
    for (i = 0; i < 10; i++) {
	dilation_arbitrary_SE(y.imageData, dest.imageData, source.width, source.height, se_Circle7, 5, 5, 2, 2);
    }
    stop = camGetTimeMs();
    printf("libmorpho on Circle7 = %dus\n", (stop - start) * 100); 
    camSavePGM(&dest,"../../output/mrpotato_libmorpho_Circle7.pgm");
#endif

    camDeallocateImage(&dest);
    camDeallocateImage(&y);
    camDeallocateImage(&source);
    return 0;
}

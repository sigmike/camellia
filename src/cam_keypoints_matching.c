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

    Redistribution and use in integral and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of integral code must retain the above copyright
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

#include "camellia.h"
#include "camellia_internals.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

#ifdef CAM_EUCLIDIAN_DISTANCE
int camCompareKeypoints(CamKeypoint *point1, CamKeypoint *point2)
{
    int i;
    long long distance = 0, x;
    for (i = 0; i < point1->size; i++) {
	x = point1->descriptor[i] - point2->descriptor[i];
	x *= x;
	distance += x;
    }
    return (int)(distance >> 24);
}
#else
#ifdef __SSE2__
#include <emmintrin.h>

int camCompareDescriptors(const int *desc1, const int *desc2, const int s)
{
    int i, j, distance = 0;
    __m128i sum, d1, d2, md, d, cmp;
    __m128i *p1 = (__m128i*)desc1, *p2 = (__m128i*)desc2;
    int out_sse[4] __attribute__((aligned(16)));
	
    /* Looks like a good idea... But this deteriorates performance...
    // Software prefetch
    d1 = _mm_load_si128(p1);
    d2 = _mm_load_si128(p2);
    for (i = 0; i != s; i += 32) {
	_mm_prefetch(&desc1[i], _MM_HINT_NTA);
	_mm_prefetch(&desc2[i], _MM_HINT_NTA);
    }
    */

    sum = _mm_setzero_si128();
    for (i = 0; i != s >> 4; i++) {
	// 32-bits SAD for 4 integers in parallel
	d1 = _mm_load_si128(p1++);
	d2 = _mm_load_si128(p2++);
	d = _mm_sub_epi32(d1, d2);
	md = _mm_sub_epi32(d2, d1);
	cmp = _mm_cmplt_epi32(d, _mm_setzero_si128());
	md = _mm_and_si128(cmp, md);
	d = _mm_andnot_si128(cmp, d);
	sum = _mm_add_epi32(sum, md);
	sum = _mm_add_epi32(sum, d);   
	
	// 32-bits SAD for 4 integers in parallel
	d1 = _mm_load_si128(p1++);
	d2 = _mm_load_si128(p2++);
	d = _mm_sub_epi32(d1, d2);
	md = _mm_sub_epi32(d2, d1);
	cmp = _mm_cmplt_epi32(d, _mm_setzero_si128());
	md = _mm_and_si128(cmp, md);
	d = _mm_andnot_si128(cmp, d);
	sum = _mm_add_epi32(sum, md);
	sum = _mm_add_epi32(sum, d);   
	
	// 32-bits SAD for 4 integers in parallel
	d1 = _mm_load_si128(p1++);
	d2 = _mm_load_si128(p2++);
	d = _mm_sub_epi32(d1, d2);
	md = _mm_sub_epi32(d2, d1);
	cmp = _mm_cmplt_epi32(d, _mm_setzero_si128());
	md = _mm_and_si128(cmp, md);
	d = _mm_andnot_si128(cmp, d);
	sum = _mm_add_epi32(sum, md);
	sum = _mm_add_epi32(sum, d);   
	
	// 32-bits SAD for 4 integers in parallel
	d1 = _mm_load_si128(p1++);
	d2 = _mm_load_si128(p2++);
	d = _mm_sub_epi32(d1, d2);
	md = _mm_sub_epi32(d2, d1);
	cmp = _mm_cmplt_epi32(d, _mm_setzero_si128());
	md = _mm_and_si128(cmp, md);
	d = _mm_andnot_si128(cmp, d);
	sum = _mm_add_epi32(sum, md);
	sum = _mm_add_epi32(sum, d);   
    }
    _mm_store_si128((__m128i*)out_sse, sum);
    return out_sse[0] + out_sse[1] + out_sse[2] + out_sse[3];
}

int camCompareKeypoints(CamKeypoint *point1, CamKeypoint *point2)
{
    return camCompareDescriptors(point1->descriptor, point2->descriptor, point1->size);
}

#else
int camCompareDescriptors(int *d1, int *d2, int s)
{
    int i, distance = 0;
    for (i = 0; i < s; i++) {
	distance += abs(d1[i] - d2[i]);
    }
    return distance;
}

int camCompareKeypoints(CamKeypoint *point1, CamKeypoint *point2)
{
    return camCompareDescriptors(point1->descriptor, point2->descriptor, point1->size);
}
#endif
#endif

int camAllocateKeypointsMatches(CamKeypointsMatches *matches, int nbpairs)
{
    matches->pairs = (CamKeypointsMatch*)malloc(nbpairs * sizeof(CamKeypointsMatch));
    matches->nbMatches = 0;
    matches->nbOutliers = 0;
    if (matches->pairs == NULL) return 0;
    matches->allocated = nbpairs;
    return 1;
}

void camFreeKeypointsMatches(CamKeypointsMatches *matches)
{
    if (matches->pairs) {
	free(matches->pairs);
    }
    matches->pairs = NULL;
    matches->nbMatches = 0;
    matches->nbOutliers = 0;
    matches->allocated = 0;
}

CamKeypoint* camFindKeypoint(CamKeypoint *point, CamKeypoints *points, int *dist1, int *dist2)
{
    int i, best = 0;
    int distance, bestDistance, secondBestDistance;
    bestDistance = camCompareKeypoints(points->keypoint[0], point);
    secondBestDistance = camCompareKeypoints(points->keypoint[1], point);
    if (secondBestDistance < bestDistance) {
	i = bestDistance;
	bestDistance = secondBestDistance;
	secondBestDistance = i;
	best = 1;
    }
    for (i = 2; i < points->nbPoints; i++) {
	distance = camCompareKeypoints(points->keypoint[i], point);
	if (distance <= bestDistance) {
	    secondBestDistance = bestDistance;
	    bestDistance = distance;
	    best = i;
	} else if (distance <= secondBestDistance) {
	    secondBestDistance = distance; 
	}
    }
    *dist1 = bestDistance;
    *dist2 = secondBestDistance;
    return points->keypoint[best];
}

// Spatial clustering of features
typedef struct {
    int *index;
    CamKeypoint **points;
} CamKeypointsCluster;

void camKeypointsClusterFree(CamKeypointsCluster *cluster)
{
    free(cluster->index);
    free(cluster->points);
}

// Cluster size is 64 pixels (right shift of 6 bits)
#define CAM_CLUSTER_SIZE 6

int camKeypointsClustering(CamKeypoints *object, CamKeypointsCluster *cluster)
{
    int i, c, nbClusters, wc;
    int *count;
    
    // Memory allocation
    wc = (((object->width - 1) >> CAM_CLUSTER_SIZE) + 1); 
    nbClusters = wc * (((object->height - 1) >> CAM_CLUSTER_SIZE) + 1);
    cluster->index = (int*)malloc(nbClusters * sizeof(int));
    cluster->points = (CamKeypoint**)malloc(object->nbPoints * sizeof(CamKeypoint*));

    count = (int*)malloc(nbClusters * sizeof(int));
    for (i = 0; i < nbClusters; i++) count[i] = 0;
	
    // First pass : count the number of points per cluster
    for (i = 0; i < object->nbPoints; i++) {
	c = (object->keypoint[i]->x >> CAM_CLUSTER_SIZE) + wc * (object->keypoint[i]->y >> CAM_CLUSTER_SIZE);
	count[c]++;
    }
    // Initialize indexes
    cluster->index[0] = 0;
    for (i = 1; i < object->nbPoints; i++) {
	cluster->index[i] = cluster->index[i-1] + count[i-1];
    }
    for (i = 0; i < nbClusters; i++) count[i] = 0;
    // Second pass : fill the pointers
    for (i = 0; i < object->nbPoints; i++) {
	c = (object->keypoint[i]->x >> CAM_CLUSTER_SIZE) + wc * (object->keypoint[i]->y >> CAM_CLUSTER_SIZE);
	cluster->points[cluster->index[c] + count[c]] = object->keypoint[i];
	count[c]++;
    }
    free(count);
    return 1;
}

int camKeypointsSmartMatching(CamKeypoints *model, CamKeypoints *target)
{
    CamKeypointsCluster model_cluster, target_cluster;

    // Spatial clustering
    camKeypointsClustering(model, &model_cluster); 
    camKeypointsClustering(target, &target_cluster); 
       
    // A* Algorithm
    // ...

    // Free memory 
    camKeypointsClusterFree(&model_cluster);
    camKeypointsClusterFree(&target_cluster);
    return 0;
}

int camKeypointsMatching(CamKeypoints *target, CamKeypoints **models, int nbM, CamKeypointsMatches *matches)
{
    int i, c, model, best, nbModels = 0;
    int distance, bestDistance, secondBestDistance, bestModel, secondBestModel;
    CamKeypoint *point, *bestMatch;
#define MAX_NB_MODELS 256
    int results[MAX_NB_MODELS], best4Target[2048];

    CAM_CHECK_ARGS(camKeypointsMatching, nbModels <= MAX_NB_MODELS);
    CAM_CHECK_ARGS(camKeypointsMatching, target->nbPoints <= 2048);
    CAM_CHECK_ARGS(camKeypointsMatching, matches->allocated != 0);

    for (model = 0; model < MAX_NB_MODELS; model++) results[model] = 0;
    matches->nbMatches = 0;
    matches->nbOutliers = 0;

    for (c = 0; c < target->nbPoints; c++) {
	point = target->keypoint[c];
	bestDistance = -1; bestModel = 0;

	for (model = 0; model < nbM; model++) {
	    for (i = 0; i < models[model]->nbPoints; i++) {
		distance = camCompareKeypoints(models[model]->keypoint[i], point);
		if (bestDistance == -1 || distance <= bestDistance) {
		    bestMatch = models[model]->keypoint[i];
		    secondBestDistance = bestDistance;
		    secondBestModel = bestModel;
		    bestDistance = distance;
		    bestModel = model;
		} else if (secondBestDistance == -1 || distance <= secondBestDistance) {
		    secondBestDistance = distance; 
		    secondBestModel = model;
		}
	    }
	}
	
	//printf("%d %d %d\n", bestMatch->set->id, bestDistance, secondBestDistance);
	
	/*
	if (models[bestModel]->id == models[secondBestModel]->id) {
	    // Accept the point whatever it is
	    secondBestDistance = 2 * bestDistance;
	}
	*/

	/*
	    secondBestDistance = -1; secondBestModel = 0;

	    // We have to scan again for the second best
	    for (model = 0; model < nbM; model++) {
		if (models[model]->id == models[bestModel]->id) continue; // Skip the best model
		for (i = 0; i < models[model]->nbPoints; i++) {
		    distance = camCompareKeypoints(models[model]->keypoint[i], point);
		    if (secondBestDistance == -1 || distance <= secondBestDistance) {
			secondBestDistance = distance; 
			secondBestModel = model;
		    }
		}
	    }
	}
	*/	 

	// Final test...
	if (bestDistance < 0.8 * secondBestDistance) {
	    if (bestMatch->set->id >= 0 && bestMatch->set->id < MAX_NB_MODELS) {
		results[bestMatch->set->id]++;
		if (bestMatch->set->id + 1 > nbModels) nbModels = bestMatch->set->id + 1;
	    }
    	    matches->pairs[matches->nbMatches].p1 = bestMatch;
    	    matches->pairs[matches->nbMatches].p2 = point;
    	    matches->pairs[matches->nbMatches].mark = bestDistance;
	    best4Target[matches->nbMatches] = bestMatch->set->id;
	    matches->nbMatches++;
	    if (matches->nbMatches == matches->allocated) break;
	}	    
    }

    best = 0;
    for (i = 1; i < nbModels; i++) {
	if (results[i] > results[best]) best = i;
    }
    
    c = 0;
    for (i = 0; i < matches->nbMatches; i++) {
	if (best4Target[i] == best) {
	    matches->pairs[c++] = matches->pairs[i];
	}
    }
    matches->nbMatches = results[best];
    return best;
}

int camKeypointsMatching2(CamKeypoints *points1, CamKeypoints *points2, CamKeypointsMatches *matches)
{
    int i, dist1, dist2;
    CamKeypoint *best;
    matches->nbMatches = 0;
    matches->nbOutliers = 0;
    
    for (i = 0; i < points1->nbPoints; i++) {
	best = camFindKeypoint(points1->keypoint[i], points2, &dist1, &dist2);
	if (dist1 < 0.8 * dist2) {
    	    matches->pairs[matches->nbMatches].p1 = points1->keypoint[i];
    	    matches->pairs[matches->nbMatches].p2 = best;
    	    matches->pairs[matches->nbMatches].mark = dist1;
	    matches->nbMatches++;
	    if (matches->nbMatches == matches->allocated) break;
	}
    }
    return matches->nbMatches;
}

// Arithmetic routines
double *camAllocateVector(int nl, int nh)
{
    double *v;
    v=(double *)malloc((unsigned)(nh - nl + 1) * sizeof(double));
    if (!v) {
	camError("camAllocateVector", "Memory allocation failure");
	return NULL;
    }
    return v-nl;
}

double **camAllocateMatrix(int nrl, int nrh, int ncl, int nch)
{
    int i;
    double **m;

    m=(double **)malloc((unsigned)(nrh - nrl + 1)*sizeof(double*));
    if (!m) {
	camError("camAllocateMatrix", "Memory allocation failure");
	return NULL;
    }
    m -= nrl;

    for(i = nrl; i <= nrh; i++) {
	m[i]=(double *)malloc((unsigned)(nch - ncl + 1)*sizeof(double));
	if (!m[i]) {
	    camError("camAllocateMatrix", "Memory allocation failure");
	    return NULL;
	}
	m[i] -= ncl;
    }
    return m;
}

int camFreeVector(double *v, int nl, int nh)
{
    free((void*)(v + nl));
    return 1;
}

int camFreeMatrix(double **m, int nrl, int nrh, int ncl, int nch)
{
    int i;
    for(i = nrh; i >= nrl; i--) free((void*)(m[i] + ncl));
    free((void*)(m + nrl));
    return 1;
}

#define CAM_SQR(a) ((camSqrTmp=(a)) == 0.0 ? 0.0 : camSqrTmp*camSqrTmp)

#define CAM_IMIN(a,b) (camMinTmp1=(a),camMinTmp2=(b),(camMinTmp1) < (camMinTmp2) ?\
        (camMinTmp1) : (camMinTmp2))

#define CAM_SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

#define CAM_DMAX(a,b) (camMaxTmp1=(a),camMaxTmp2=(b),(camMaxTmp1) > (camMaxTmp2) ?\
        (camMaxTmp1) : (camMaxTmp2))

double camPythag(double a, double b)
{
    double absa, absb;
    double camSqrTmp;
    absa=fabs(a);
    absb=fabs(b);
    if (absa > absb) return absa*sqrt(1.0+CAM_SQR(absb/absa));
    else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+CAM_SQR(absa/absb)));
}

int camSVD(double **a, int m, int n, double w[], double **v)
{
    int flag, i, its, j, jj, k, l, nm;
    double anorm, c, f, g, h, s, scale, x, y, z, *rv1;
    int camMinTmp1, camMinTmp2;
    double camMaxTmp1, camMaxTmp2;

    rv1=camAllocateVector(1,n);
    g=scale=anorm=0.0;
    for (i=1;i<=n;i++) {
	l=i+1;
	rv1[i]=scale*g;
	g=s=scale=0.0;
	if (i <= m) {
	    for (k=i;k<=m;k++) scale += fabs(a[k][i]);
	    if (scale) {
		for (k=i;k<=m;k++) {
		    a[k][i] /= scale;
		    s += a[k][i]*a[k][i];
		}
		f=a[i][i];
		g = -CAM_SIGN(sqrt(s),f);
		h=f*g-s;
		a[i][i]=f-g;
		for (j=l;j<=n;j++) {
		    for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
		    f=s/h;
		    for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
		}
		for (k=i;k<=m;k++) a[k][i] *= scale;
	    }
	}
	w[i]=scale*g;
	g=s=scale=0.0;
	if (i <= m && i != n) {
	    for (k=l;k<=n;k++) scale += fabs(a[i][k]);
	    if (scale) {
		for (k=l;k<=n;k++) {
		    a[i][k] /= scale;
		    s += a[i][k]*a[i][k];
		}
		f=a[i][l];
		g = -CAM_SIGN(sqrt(s),f);
		h=f*g-s;
		a[i][l]=f-g;
		for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
		for (j=l;j<=m;j++) {
		    for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
		    for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
		}
		for (k=l;k<=n;k++) a[i][k] *= scale;
	    }
	}
	anorm=CAM_DMAX(anorm,(fabs(w[i])+fabs(rv1[i])));
    }
    for (i=n;i>=1;i--) {
	if (i < n) {
	    if (g) {
		for (j=l;j<=n;j++) v[j][i]=(a[i][j]/a[i][l])/g;
		for (j=l;j<=n;j++) {
		    for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
		    for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
		}
	    }
	    for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
	}
	v[i][i]=1.0;
	g=rv1[i];
	l=i;
    }
    for (i=CAM_IMIN(m,n);i>=1;i--) {
	l=i+1;
	g=w[i];
	for (j=l;j<=n;j++) a[i][j]=0.0;
	if (g) {
	    g=1.0/g;
	    for (j=l;j<=n;j++) {
		for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
		f=(s/a[i][i])*g;
		for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
	    }
	    for (j=i;j<=m;j++) a[j][i] *= g;
	} else for (j=i;j<=m;j++) a[j][i]=0.0;
	++a[i][i];
    }
    for (k=n;k>=1;k--) {
	for (its=1;its<=30;its++) {
	    flag=1;
	    for (l=k;l>=1;l--) {
		nm=l-1;
		if ((double)(fabs(rv1[l])+anorm) == anorm) {
		    flag=0;
		    break;
		}
		if ((double)(fabs(w[nm])+anorm) == anorm) break;
	    }
	    if (flag) {
		c=0.0;
		s=1.0;
		for (i=l;i<=k;i++) {
		    f=s*rv1[i];
		    rv1[i]=c*rv1[i];
		    if ((double)(fabs(f)+anorm) == anorm) break;
		    g=w[i];
		    h=camPythag(f,g);
		    w[i]=h;
		    h=1.0/h;
		    c=g*h;
		    s = -f*h;
		    for (j=1;j<=m;j++) {
			y=a[j][nm];
			z=a[j][i];
			a[j][nm]=y*c+z*s;
			a[j][i]=z*c-y*s;
		    }
		}
	    }
	    z=w[k];
	    if (l == k) {
		if (z < 0.0) {
		    w[k] = -z;
		    for (j=1;j<=n;j++) v[j][k] = -v[j][k];
		}
		break;
	    }
	    if (its == 100) {
		camError("camSVD", "No convergence after 100 iterations");
		return 0;
	    }
	    x=w[l];
	    nm=k-1;
	    y=w[nm];
	    g=rv1[nm];
	    h=rv1[k];
	    f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
	    g=camPythag(f,1.0);
	    f=((x-z)*(x+z)+h*((y/(f+CAM_SIGN(g,f)))-h))/x;
	    c=s=1.0;
	    for (j=l;j<=nm;j++) {
		i=j+1;
		g=rv1[i];
		y=w[i];
		h=s*g;
		g=c*g;
		z=camPythag(f,h);
		rv1[j]=z;
		c=f/z;
		s=h/z;
		f=x*c+g*s;
		g = g*c-x*s;
		h=y*s;
		y *= c;
		for (jj=1;jj<=n;jj++) {
		    x=v[jj][j];
		    z=v[jj][i];
		    v[jj][j]=x*c+z*s;
		    v[jj][i]=z*c-x*s;
		}
		z=camPythag(f,h);
		w[j]=z;
		if (z) {
		    z=1.0/z;
		    c=f*z;
		    s=h*z;
		}
		f=c*g+s*y;
		x=c*y-s*g;
		for (jj=1;jj<=m;jj++) {
		    y=a[jj][j];
		    z=a[jj][i];
		    a[jj][j]=y*c+z*s;
		    a[jj][i]=z*c-y*s;
		}
	    }
	    rv1[l]=0.0;
	    rv1[k]=f;
	    w[k]=x;
	}
    }
    camFreeVector(rv1,1,n);
    return 1;
}

int camSVDSolve(double **u, double w[], double **v, int m, int n, double b[], double x[])
{
    int jj,j,i;
    double s,*tmp;

    tmp=camAllocateVector(1,n);
    for (j=1;j<=n;j++) {
	s=0.0;
	if (w[j]) {
	    for (i=1;i<=m;i++) s += u[i][j]*b[i];
	    s /= w[j];
	}
	tmp[j]=s;
    }
    for (j=1;j<=n;j++) {
	s=0.0;
	for (jj=1;jj<=n;jj++) s += v[j][jj]*tmp[jj];
	x[j]=s;
    }
    camFreeVector(tmp,1,n);
    return 1;
}

int camFindAffineTransform(CamKeypointsMatches *matches, CamAffineTransform *t, int *error)
{
    double **A, *b, *w, **v, *x;
    CamPoint xy, uv;
    int d1, d2; 
    int i, c;
    
    // Fill matrix A and vector b
    A = camAllocateMatrix(1, matches->nbMatches * 2, 1, 6);
    b = camAllocateVector(1, matches->nbMatches * 2);
    for (i = 0, c = 0; i < matches->nbMatches; i++) {
	if (matches->pairs[i].mark != -1) {
	    A[c * 2 + 1][1] = matches->pairs[i].p1->x;
	    A[c * 2 + 1][2] = matches->pairs[i].p1->y;
	    A[c * 2 + 1][3] = 0; A[c * 2 + 1][4] = 0;
	    A[c * 2 + 1][5] = 1; A[c * 2 + 1][6] = 0;
	    A[c * 2 + 2][3] = matches->pairs[i].p1->x;
	    A[c * 2 + 2][4] = matches->pairs[i].p1->y;
	    A[c * 2 + 2][1] = 0; A[c * 2 + 2][2] = 0;
	    A[c * 2 + 2][5] = 0; A[c * 2 + 2][6] = 1;
	    b[c * 2 + 1] = matches->pairs[i].p2->x;
	    b[c * 2 + 2] = matches->pairs[i].p2->y;
	    c++;
	}
    }
    
    // Use SVD to find the result
    w = camAllocateVector(1, 6);
    v = camAllocateMatrix(1, 6, 1, 6);
    camSVD(A, c * 2, 6, w, v);
    x = (double*)t->m - 1;
    camSVDSolve(A, w, v, c * 2, 6, b, x);

    // Compute the error
    *error = 0;
    for (i = 0; i < matches->nbMatches; i++) {
	if (matches->pairs[i].mark != -1) {
	    // Compute the theoretical location of the match
	    xy.x = matches->pairs[i].p1->x;
	    xy.y = matches->pairs[i].p1->y;
	    camApplyAffineTransform(&xy, &uv, t);
	    d1 = uv.x - matches->pairs[i].p2->x;
	    d2 = uv.y - matches->pairs[i].p2->y;
	    matches->pairs[i].error = d1*d1 + d2*d2;
	    *error += matches->pairs[i].error;
	}	
    }

    camFreeMatrix(A, 1, matches->nbMatches * 2, 1, 6);
    camFreeVector(b, 1, matches->nbMatches * 2);
    camFreeVector(w, 1, 6);
    camFreeMatrix(v, 1, 6, 1, 6);
    return 1;
}

void camApplyAffineTransform(CamPoint *p1, CamPoint *p2, CamAffineTransform *t)
{
    p2->x = (int)(t->m[0] * p1->x + t->m[1] * p1->y + t->m[4] + 0.5);
    p2->y = (int)(t->m[2] * p1->x + t->m[3] * p1->y + t->m[5] + 0.5);
}

int camCompareMatches(const void *m1x, const void *m2x)
{
    CamKeypointsMatch *m1 = (CamKeypointsMatch*)m1x;
    CamKeypointsMatch *m2 = (CamKeypointsMatch*)m2x;
    if (m1->error < m2->error) return 1;
    if (m1->error == m2->error) return 0;
    return -1;
}

int camFindAffineTransform2(CamKeypointsMatches *matches, CamAffineTransform *t, int *error)
{
    int cerror, i, c;
    for (c = 0; c < 2; c++) {
	if (matches->nbMatches - matches->nbOutliers < 3) return 0;
	camFindAffineTransform(matches, t, error);
	cerror = *error;
	qsort(matches->pairs, matches->nbMatches, sizeof(CamKeypointsMatch), camCompareMatches);
	i = 0;
	while (cerror >= *error / 3) {
	    if (matches->pairs[i].mark != -1) {
		matches->pairs[i].mark = -1;
		matches->nbOutliers++;
		cerror -= matches->pairs[i].error;
		// printf("Removing pairs of error %d\n", matches->pairs[i].error);
	    }
	    i++;
	}
    }
    return camFindAffineTransform(matches, t, error);
}

typedef struct {
    int distance;
    CamFPKdTreeNode *node;
    int *descriptor;
} CamFPKdTreeBranch;

#define EXCH(a, b) tmp = pqueue[a]; pqueue[a] = pqueue[b]; pqueue[b] = tmp;

void camFPKdTreeFixUp(CamFPKdTreeBranch *pqueue, int k)
{
    CamFPKdTreeBranch tmp;
    while (k > 0 && pqueue[(k-1)/2].distance > pqueue[k].distance) {
	EXCH((k-1)/2, k);
	k = (k-1)/2;
    }
}

void camFPKdTreeFixDown(CamFPKdTreeBranch *pqueue, int k, int N)
{
    int j;
    CamFPKdTreeBranch tmp;
    while (2*k+1 < N) {
	j = 2*k+1;
	if (j < N-1 && pqueue[j].distance > pqueue[j+1].distance) j++;
	if (!(pqueue[k].distance > pqueue[j].distance)) break;
	EXCH(k, j);
	k = j;
    }
}

void camFPKdTreeInsert(CamFPKdTreeBranch *pqueue, CamFPKdTreeBranch *item, int *N)
{
    pqueue[*N] = *item;
    camFPKdTreeFixUp(pqueue, *N);
    (*N)++;
}

CamFPKdTreeBranch *camFPKdTreeGetmin(CamFPKdTreeBranch *pqueue, int *N)
{
    CamFPKdTreeBranch tmp;
    if (*N == 0) return NULL; 
    (*N)--;
    EXCH(0, *N);
    camFPKdTreeFixDown(pqueue, 0, *N);
    return &pqueue[*N];
}

CamKeypoint *camFindKeypointKdTree(CamKeypoint *point, CamFPKdTreeNode *kdTreeRoot, int explore, int *dist1, int *dist2)
{
    CamFPKdTreeNode *node;
    CamKeypoint *best;
    int i, distance, bestDistance = -1, firstDistance, secondBestDistance = -1;
    int *descriptor_heap, descriptor_heap_pos = 0, descriptor_heap_size;
    CamFPKdTreeBranch *branches_pqueue, branch, cbr, *br;
    int nbBranches = 0, explored = 0;
#define MAX_NB_BRANCHES 1000

    const int nbBytes = point->size * sizeof(int);

    // Let's start by finding a first candidate by directly digging into the kdTree
    node = kdTreeRoot;
    while (node->i != -1) {
	if (point->descriptor[node->i] < node->m)
	    node = node + 1;
	else node = node->right;
    }
    best = (CamKeypoint*)node->right;
    firstDistance = camCompareKeypoints(best, point);

    // OK. Now we can start again from the root
    descriptor_heap_size = point->size * MAX_NB_BRANCHES * 10;
    descriptor_heap = (int*)malloc(sizeof(int) * descriptor_heap_size);
    branches_pqueue = (CamFPKdTreeBranch*)malloc(sizeof(CamFPKdTreeBranch) * MAX_NB_BRANCHES);
    for (i = 0; i < point->size; i++) descriptor_heap[i] = point->descriptor[i];    
    branch.distance = 0;
    branch.descriptor = descriptor_heap;
    branch.node = kdTreeRoot;
    descriptor_heap_pos += point->size;
    camFPKdTreeInsert(branches_pqueue, &branch, &nbBranches);

    // We are now in the main process loop
    do {
	// Let's take the best branch first
	br = camFPKdTreeGetmin(branches_pqueue, &nbBranches);
	if (br) {
	    // Is this a leaf ?
	    if (br->node->i == -1) {
		// Yes. Let's see whether it is better or not...
		distance = camCompareKeypoints((CamKeypoint*)br->node->right, point);
		if (bestDistance == -1 || distance <= bestDistance) {
		    secondBestDistance = bestDistance;
		    bestDistance = distance;
		    best = (CamKeypoint*)br->node->right;
		} else if (secondBestDistance == -1 || distance <= secondBestDistance) {
		    secondBestDistance = distance; 
		}
		explored++;
		if (explored == explore) break;
	    } else {
		// No. We have to explore the children of this node
		// First of all, let's check that it is still a valid hypothesis...
		if (br->distance < bestDistance || br->distance <= firstDistance) {
		    // OK. It is valid.
		    branch = cbr = *br;
		    // Let's compute the distance to the left and right children
		    if (point->descriptor[br->node->i] < br->node->m) {
			// The left child is closer
			// We can insert it for sure in the priority queue
			branch.node = br->node + 1; // This is the left child
			if (nbBranches == MAX_NB_BRANCHES) { best = NULL; break; }
			camFPKdTreeInsert(branches_pqueue, &branch, &nbBranches); // This destroys br pointer
			branch.distance = cbr.distance + cbr.node->m - cbr.descriptor[cbr.node->i]; // Compute this in advance while br is this valid
			// And what about the right child ?
			if (branch.distance < bestDistance || branch.distance < firstDistance) {
			    // OK. It is valid
			    branch.node = cbr.node->right;
			    if (descriptor_heap_pos == descriptor_heap_size) { best = NULL; break; }
			    //for (i = 0; i < point->size; i++) *(descriptor_heap + descriptor_heap_pos + i) = cbr.descriptor[i];    
			    memcpy(descriptor_heap + descriptor_heap_pos, cbr.descriptor, nbBytes);
			    branch.descriptor = descriptor_heap + descriptor_heap_pos;
			    branch.descriptor[cbr.node->i] = cbr.node->m;
			    descriptor_heap_pos += point->size;
			    if (nbBranches == MAX_NB_BRANCHES) { best = NULL; break; }
			    camFPKdTreeInsert(branches_pqueue, &branch, &nbBranches);
			}	
		    } else {
			// The right child is closer
			// We can insert it for sure in the priority queue
			branch.node = br->node->right; // This is the right child
			if (nbBranches == MAX_NB_BRANCHES) { best = NULL; break; }
			camFPKdTreeInsert(branches_pqueue, &branch, &nbBranches);
			branch.distance = cbr.distance + cbr.descriptor[cbr.node->i] - cbr.node->m;
			// And what about the left child ?
			if (branch.distance < bestDistance || branch.distance < firstDistance) {
			    // OK. It is valid
			    branch.node = cbr.node + 1;
			    if (descriptor_heap_pos == descriptor_heap_size) { best = NULL; break; }
			    //for (i = 0; i < point->size; i++) *(descriptor_heap + descriptor_heap_pos + i) = cbr.descriptor[i];    
			    memcpy(descriptor_heap + descriptor_heap_pos, cbr.descriptor, nbBytes);
			    branch.descriptor = descriptor_heap + descriptor_heap_pos;
			    branch.descriptor[cbr.node->i] = cbr.node->m;
			    descriptor_heap_pos += point->size;
			    if (nbBranches == MAX_NB_BRANCHES) { best = NULL; break; }
			    camFPKdTreeInsert(branches_pqueue, &branch, &nbBranches);
			}	
		    }
		} else 
		    // No. No point in this node can be better than the one we have
		    // It is the end of the exploration
		    break;
	    }
	}
    } while (br);
    
    *dist1 = bestDistance;
    *dist2 = secondBestDistance;
    free(descriptor_heap);
    free(branches_pqueue);
    return best;
}

int camKeypointsMatchingKdTree(CamKeypoints *target, CamFPKdTreeNode *kdTreeRoot, CamKeypointsMatches *matches, int explore)
{
    int i, c, model, best, nbModels = 0;
    int bestDistance, secondBestDistance;
    CamKeypoint *point, *bestMatch;
    int results[MAX_NB_MODELS], best4Target[2048];

    CAM_CHECK_ARGS(camKeypointsMatchingKdTree, target->nbPoints <= 2048);
    CAM_CHECK_ARGS(camKeypointsMatchingKdTree, matches->allocated != 0);

    for (model = 0; model < MAX_NB_MODELS; model++) results[model] = 0;
    matches->nbMatches = 0;
    matches->nbOutliers = 0;

    for (c = 0; c < target->nbPoints; c++) {
	point = target->keypoint[c];
	bestMatch = camFindKeypointKdTree(point, kdTreeRoot, explore, &bestDistance, &secondBestDistance);
	//printf("%d %d %d\n", bestMatch->set->id, bestDistance, secondBestDistance);
	// Final test...
	if (secondBestDistance == -1 || bestDistance < 0.8 * secondBestDistance) {
	    if (bestMatch->set->id >= 0 && bestMatch->set->id < MAX_NB_MODELS) {
		results[bestMatch->set->id]++;
		if (bestMatch->set->id + 1 > nbModels) nbModels = bestMatch->set->id + 1;
	    }
    	    matches->pairs[matches->nbMatches].p1 = bestMatch;
    	    matches->pairs[matches->nbMatches].p2 = point;
    	    matches->pairs[matches->nbMatches].mark = bestDistance;
	    best4Target[matches->nbMatches] = bestMatch->set->id;
	    matches->nbMatches++;
	    if (matches->nbMatches == matches->allocated) break;
	}	    
    }

    best = 0;
    for (i = 1; i < nbModels; i++) {
	if (results[i] > results[best]) best = i;
    }
    
    c = 0;
    for (i = 0; i < matches->nbMatches; i++) {
	if (best4Target[i] == best) {
	    matches->pairs[c++] = matches->pairs[i];
	}
    }
    matches->nbMatches = results[best];
    return best;
}

int camFPKdTreeCompare(const void *fp1x, const void *fp2x)
{
    CamKeypoint **fp1 = (CamKeypoint**)fp1x;
    CamKeypoint **fp2 = (CamKeypoint**)fp2x;
    if ((*fp1)->descriptor[(int)(*fp1)->internal] > (*fp2)->descriptor[(int)(*fp2)->internal]) return 1;
    if ((*fp1)->descriptor[(int)(*fp1)->internal] == (*fp2)->descriptor[(int)(*fp2)->internal]) return 0;
    return -1;
}

CamFPKdTreeNode *camFPKdTreeRecurs(CamFPKdTreeNode *node, double deviation[128], int index[128], CamKeypoint **points, int nbPoints)
{
    double dev[128], tmpdev;
    int idx[128], i, j, k, tmpidx;
    double sum, avg, devx, diff;
    
    if (nbPoints == 1) {
	node->i = -1;
	node->right = (CamFPKdTreeNode*)points[0];
	return node + 1;
    }
    // First of all, find out the index of the maximum deviation
    node->i = index[0]; // This is quite simple
    // Fixdown the heap
    deviation[0] = deviation[points[0]->size - 1];
    index[0] = index[points[0]->size - 1];
    k = 0;
    while (2*k+1 < points[0]->size - 1) {
	j = 2*k+1;
	if (j < points[0]->size - 2 && deviation[j] < deviation[j+1]) j++;
	if (!(deviation[k] < deviation[j])) break;
	tmpdev = deviation[k];
	deviation[k] = deviation[j];
	deviation[j] = tmpdev;
	tmpidx = index[k];
	index[k] = index[j];
	index[j] = tmpidx;	
	k = j;
    }
   
    // We have the maximum deviation
    // Let's find the median value on this index
    for (i = 0; i < nbPoints; i++) {
	points[i]->internal = (void*)node->i;
    }
    qsort(points, nbPoints, sizeof(CamKeypoint*), camFPKdTreeCompare);
    node->m = (points[nbPoints/2 - 1]->descriptor[node->i] + points[nbPoints/2]->descriptor[node->i]) / 2;
    
    // Now, we can compute the deviation on the left part
    for (i = 0; i < points[0]->size - 1; i++) {
	dev[i] = deviation[i];
	idx[i] = index[i];
    }
    sum = 0;
    for (j = 0; j < nbPoints/2; j++) {
	sum += points[j]->descriptor[node->i];
    }
    avg = sum / (nbPoints/2);
    devx = 0;
    for (j = 0; j < nbPoints/2; j++) {
	diff = points[j]->descriptor[node->i] - avg;
	devx += diff * diff;
    }
    devx /= nbPoints/2;
    // Insert this into the priority queue
    deviation[points[0]->size - 1] = devx;
    index[points[0]->size - 1] = node->i;
    k = points[0]->size - 1;
    while (k > 0 && deviation[(k-1)/2] < deviation[k]) {
	// Swap (k-1)/2 and k
	tmpdev = deviation[(k-1)/2];
	deviation[(k-1)/2] = deviation[k];
	deviation[k] = tmpdev;
	tmpidx = index[(k-1)/2];
	index[(k-1)/2] = index[k];
	index[k] = tmpidx;
	k = (k-1)/2;
    }
    // OK. We can now call recursively camKdTreeRecurs with the left part
    node->right = camFPKdTreeRecurs(node + 1, deviation, index, points, nbPoints / 2);

    // Now, we can compute the deviation on the right part
    sum = 0;
    for (j = nbPoints/2; j < nbPoints; j++) {
	sum += points[j]->descriptor[node->i];
    }
    avg = sum / (nbPoints - nbPoints/2);
    devx = 0;
    for (j = nbPoints/2; j < nbPoints; j++) {
	diff = points[j]->descriptor[node->i] - avg;
	devx += diff * diff;
    }
    devx /= nbPoints - nbPoints/2;
    // Insert this into the priority queue
    dev[points[0]->size - 1] = devx;
    idx[points[0]->size - 1] = node->i;
    k = points[0]->size - 1;
    while (k > 0 && dev[(k-1)/2] < dev[k]) {
	// Swap (k-1)/2 and k
	tmpdev = dev[(k-1)/2];
	dev[(k-1)/2] = dev[k];
	dev[k] = tmpdev;
	tmpidx = idx[(k-1)/2];
	idx[(k-1)/2] = idx[k];
	idx[k] = tmpidx;
	k = (k-1)/2;
    }
    // OK. We can now call recursively camKdTreeRecurs with the right part
    return camFPKdTreeRecurs(node->right, dev, idx, points + nbPoints/2, nbPoints - nbPoints / 2);
}

CamFPKdTreeNode *camKeypointsCompileKdTree(CamKeypoints **models, int nbModels)
{
    CamFPKdTreeNode *kdTree, *kdTreeCheck;
    CamKeypoint **points;
    double deviation[128], tmpdev;
    int index[128], tmpidx; 
    double sum, avg, dev, diff;
    int i, j, k, nbPoints = 0;
    
    // Initialization of kdTree and points index
    for (i = 0; i < nbModels; i++) nbPoints += models[i]->nbPoints;
    kdTree = malloc(sizeof(CamFPKdTreeNode) * nbPoints * 2);
    points = malloc(sizeof(CamKeypoint*) * nbPoints);
    for (i = 0, nbPoints = 0; i < nbModels; i++) {
	for (j = 0; j < models[i]->nbPoints; j++, nbPoints++) {
	    points[nbPoints] = models[i]->keypoint[j];
	}
    }
    
    // Initialization of deviation and index
    for (i = 0; i < points[0]->size; i++) {
	sum = 0;
	for (j = 0; j < nbPoints; j++) {
	    sum += points[j]->descriptor[i];
	}
	avg = sum / nbPoints;
	dev = 0;
	for (j = 0; j < nbPoints; j++) {
	    diff = points[j]->descriptor[i] - avg;
	    dev += diff * diff;
	}
	dev /= nbPoints;

    	// Insert this into the priority queue
	deviation[i] = dev;
	index[i] = i;
	k = i;
	while (k > 0 && deviation[(k-1)/2] < deviation[k]) {
	    // Swap (k-1)/2 and k
	    tmpdev = deviation[(k-1)/2];
	    deviation[(k-1)/2] = deviation[k];
	    deviation[k] = tmpdev;
	    tmpidx = index[(k-1)/2];
	    index[(k-1)/2] = index[k];
	    index[k] = tmpidx;
	    k = (k-1)/2;
	}
    }

    // Recursively call kdTreeRecurs
    kdTreeCheck = camFPKdTreeRecurs(kdTree, deviation, index, points, nbPoints);
    assert(kdTreeCheck <= kdTree + nbPoints * 2);
    free(points);
    return kdTree;
}


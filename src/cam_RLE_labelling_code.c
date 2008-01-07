#ifdef PARAM
int camRLEEncode(CamImage *source, CamRLEImage *dest, PARAM)
#else
int camRLEEncode(CamImage *source, CamRLEImage *dest)
#endif
{
    int x,xp,y,nbRuns,l;
    int width,height;
    CAM_PIXEL *srcptr,m;
    CamRun *newRun;

    CamInternalROIPolicyStruct iROI;
    
    // ROI (Region Of Interest) management
    CAM_CHECK(camRLEEncode,camInternalROIPolicy(source, NULL, &iROI, 0));
    CAM_CHECK_ARGS(camRLEEncode,iROI.nChannels==1);
    
    // Binary images processing 
    if (source->depth==CAM_DEPTH_1U) {
	return camRLEEncode1U(source,dest);
    }

    CAM_CHECK_ARGS(camRLEEncode,(source->depth&CAM_DEPTH_MASK)<=(sizeof(CAM_PIXEL)*8));
    CAM_CHECK_ARGS(camRLEEncode,(source->depth&CAM_DEPTH_MASK)>=8);

    width=iROI.srcroi.width;
    height=iROI.srcroi.height;
    srcptr=(CAM_PIXEL*)iROI.srcptr;

    // Automatic allocation
    if (dest->allocated==0) camRLEAllocate(dest,height*width+2);
    CAM_CHECK_ARGS(camRLEEncodeLUT,(dest->nSize==sizeof(CamRLEImage)));

    dest->height=height;
    dest->width=width;

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
	x=0; xp=0;
	m=MAP(srcptr[xp]);
	do {
	    l=x;
	    x++;
            xp+=iROI.srcinc;
	    
            while (x<width) {
                n=MAP(srcptr[xp]);
                if (n==m) {
                    xp+=iROI.srcinc;
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
    	    camRLEReallocate(dest,dest->allocated*2);
            // dest->nbRuns=0;
            // camSetErrorStr("number or runs is too high");
	    // return 0;
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
	camRLEReallocate(dest,dest->allocated*2);
        // dest->nbRuns=0;
        // camSetErrorStr("number or runs is too high");
	// return 0;
    }

    dest->nbRuns=nbRuns;
    return 1;    
}



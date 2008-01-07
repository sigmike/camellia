#define SWAP(x,y) temp=x;x=y;y=temp
#ifdef LINE
dx=abs(x2-x1)+1;
dy=abs(y2-y1)+1;
if (dx>dy) { /* X or Y axis ? */
    if (x1>x2) {
	SWAP(x1,x2);
	SWAP(y1,y2);
    }
    if (y2>y1) {
	yincr=1;
	ptrincr=iROI.srclinc;
    } else {
	yincr=-1;
	ptrincr=-iROI.srclinc;
    }
    x=x1;
    y=y1;
    runl=dx/dy;
    error=2*(dx-runl*dy);
    correction=2*dy;
    acc=0;
    inc=iROI.srcinc;
    
    /* Pointers initialization */
    INITPOINTERS;
    
    for(i=0;i<dy; i++, y+=yincr) {
	acc+=error;
	runl2=runl;
	if (acc>dy) {
            acc-=correction;
	    ++runl2;
	}
	for (j=0;j<runl2;j++,x++) {
	    SETPIXEL;
	    ptrX+=inc;
#ifdef COLOR
	    ptrG+=inc;
	    ptrB+=inc;
#endif
	}
        ptrX+=ptrincr;
#ifdef COLOR
        ptrG+=ptrincr;
        ptrB+=ptrincr;
#endif
    }
} else {
    if (y1>y2) {
	SWAP(x1,x2);
	SWAP(y1,y2);
    }
    if (x2>x1) {
	xincr=1;
	ptrincr=iROI.srcinc;
    } else {
	xincr=-1;
	ptrincr=-iROI.srcinc;
    }
    x=x1;
    y=y1;
    runl=dy/dx;
    error=2*(dy-runl*dx);
    correction=2*dx;
    acc=0;
    inc=iROI.srclinc;
    
    /* Pointers initialization */
    INITPOINTERS;
    
    for(i=0;i<dx; i++, x+=xincr, ptrX+=ptrincr) {
	acc+=error;
	runl2=runl;
	if (acc>dx) {
	    acc-=correction;
	    ++runl2;
	}
	for (j=0;j<runl2;j++,y++) {
	    SETPIXEL;
            ptrX+=inc;
#ifdef COLOR
            ptrG+=inc;
            ptrB+=inc;
#endif
        }
#ifdef COLOR
	ptrG+=ptrincr;
	ptrB+=ptrincr;
#endif
		
    }
}
#endif

#ifdef CIRCLE
INITPOINTERS(0,-1);
INITPOINTERS(1,1);
INITPOINTERS(2,-1);
INITPOINTERS(3,1);
run=1024/rx;
error=1024-rx*run;
for (x=0,y=ry,pos=0,acc=0;x<rx;x++) {
    val=camDrawCircleData[pos]*ry;		
    curval=y*1024;
	
    if (curval-val>1024) {
	do {
    	    y--;
	    ptrX[0]+=incy;
	    ptrX[1]-=incy;
	    ptrX[2]+=incy;
	    ptrX[3]-=incy;
#ifdef COLOR
	    ptrB[0]+=incy;
	    ptrB[1]-=incy;
	    ptrB[2]+=incy;
	    ptrB[3]-=incy;
	    ptrG[0]+=incy;
	    ptrG[1]-=incy;
	    ptrG[2]+=incy;
	    ptrG[3]-=incy;
#endif
	    SETPIXEL0;
	    SETPIXEL1;
	    SETPIXEL2;
	    SETPIXEL3;
	    curval-=1024;
	} while (curval-val>1024);
    } else {
	/* Plot the pixel */
	SETPIXEL0;
	SETPIXEL1;
	SETPIXEL2;
	SETPIXEL3;
    }
	
    pos+=run;
    acc+=error;
    if (acc>rx) {
	pos++;
	acc-=rx;
    }
	
    ptrX[0]+=incx;
    ptrX[1]+=incx;
    ptrX[2]-=incx;
    ptrX[3]-=incx;
#ifdef COLOR
    ptrB[0]+=incx;
    ptrB[1]+=incx;
    ptrB[2]-=incx;
    ptrB[3]-=incx;
    ptrG[0]+=incx;
    ptrG[1]+=incx;
    ptrG[2]-=incx;
    ptrG[3]-=incx;
#endif
}
/* Draw remaining vertical points */
while (y>=0) {
    y--;
    ptrX[0]+=incy;
    ptrX[1]-=incy;
    ptrX[2]+=incy;
    ptrX[3]-=incy;
#ifdef COLOR
    ptrB[0]+=incy;
    ptrB[1]-=incy;
    ptrB[2]+=incy;
    ptrB[3]-=incy;
    ptrG[0]+=incy;
    ptrG[1]-=incy;
    ptrG[2]+=incy;
    ptrG[3]-=incy;
#endif
    SETPIXEL0;
    SETPIXEL1;
    SETPIXEL2;
    SETPIXEL3;
};
#endif

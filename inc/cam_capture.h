#ifndef __CAM_CAPTURE_H
#define __CAM_CAPTURE_H

#include <atlbase.h>
#include <qedit.h>
#include <stdio.h>

#include "camellia.h"

#define SafeRelease(p) { if( (p) != 0 ) { (p)->Release(); (p)= NULL; } }

#include <dshow.h>

class CamCaptureGraph  
{
public:
    void showPropPage();
    bool createFilterGraph(IMoniker * videoDeive);
    CamCaptureGraph();
    virtual ~CamCaptureGraph();
    
    void setDesiredVideoFormat(int w, int h, int fps);
    // Set the window to display the video
    void SetRenderWindow(HWND  hWnd, bool isScaling = false) { m_pRenderCWnd = hWnd; m_fIsScaling = isScaling; }
    
    // Render a video file
    void RenderFile(char * vname = NULL);
    // Automatically find a capturing device to capture video
    // If fSetup = true, a dialog is popped first to ask for video format
    bool CaptureLive(bool fSetup = false);
    
    // Init the given video device, but does NOT start the capture;
    bool initCaptureLive (IMoniker * videoDeive, bool fSetup = false);
    
    // Destroy the whole Graph
    void Destroy() { destroyFilterGraph(); };
    
    // Pause the video
    void Pause() { m_pMediaControl->Pause(); }
    // Continue play the video
    void Play();
    // Stop the video
    void Stop() { m_pMediaControl->Stop(); }
    LONGLONG Seek( int offset );
    // Seek in the video if the media type allows
    
    // this is only useful before the video graph is built
    // if you don't want to show the video, 
    // call it first thing after the object is constructed.
    void setVideoFlag(bool flag); 
    
protected:
    // Create the filter graph to render the video (capture or playback)
    virtual bool createFilterGraph();
    // Release the filter graph
    virtual void destroyFilterGraph();
    // Start the playing
    void startGraph();
    // Stop playing
    void stopGraph();
    // automaticly find a usable capturing device
    HRESULT findCaptureDevice(IBaseFilter ** ppSrcFilter);
    
    ///////////////////////////////////////////////////////////////////
    // Build the capture graph
    virtual bool buildCaptureGraph(IBaseFilter * filter = 0);
    virtual bool buildRenderFileGraph();
    
    ///////////////////////////////////////////////////////////////////
    // The following 3 functions are for derivation if you need extra
    // filters in the graph, implement the 3 functions to do it
    ///////////////////////////////////////////////////////////////////
    // create and add extra filters, called in createFilterGraph() to
    // create extra filters into the graph
    virtual void addExtraFilters() {}
    // release the resource related to the extra filters
    virtual void releaseExtraFilters() {}
    // initialization the extra filters after the graph is finished
    virtual bool initExtraFilters()    {return false;}
    
    // Get the input or output pin to connect filters
    IPin* get_pin( IBaseFilter* pFilter, PIN_DIRECTION dir );
    
    // DirectShow interface pointers
    IGraphBuilder *m_pGraphBuilder; // build render graph
    ICaptureGraphBuilder2 * m_pCaptureGraphBuilder2; // for capture
    IMediaControl *m_pMediaControl; // MediaControl
    IVideoWindow *m_pVideoWindow;   // window to play video
    IFilterGraph *m_pFilterGraph;   // Filter Graph
    IMediaSeeking *m_pMediaSeeking; // Seeking interface
    
    HWND m_pRenderCWnd;	    // The CWnd to display the video in
    WCHAR m_aVName[100];    // The video file name to render
    bool m_fIsCapturing;    // What kind of graph we are building
    bool m_fIsScaling;      // scale the video to the display window
    bool m_fShowCaptureProperties; // Setup dialog before capture
    bool m_fstartGraph;     // has the video graph started;
    bool m_showVideoFlag;   // default = false;
    // Do we want the filtergraph to be viewable by GraphEdit
    
    bool m_fRegisterFilterGraph;
    DWORD m_dwGraphRegister;
    void removeGraphFromRot(DWORD pdwRegister);
    HRESULT addGraphToRot(IUnknown *pUnkGraph, DWORD *pdwRegister);
     
    int	m_desiredWidth, m_desiredHeight;
    int	m_desiredFrameRate;
    
    HRESULT selectVideoFormat();

public:
    IBaseFilter * m_pSrcFilter;
};

class CamCaptureGrabberCB:  public ISampleGrabberCB
{
public:	// The interface to control the tracker with the SampleGrabberFilter
    CamCaptureGrabberCB();
    ~CamCaptureGrabberCB();
    
    void SetGrabMediaType( AM_MEDIA_TYPE mt ) {m_mediaType = mt;}
      
    // The function that called when every frame is being processed
    // So we implement our tracking algorithm here now. It is like
    // the Transform() in Transform filter.
    STDMETHODIMP SampleCB(double SampleTime, IMediaSample *pSample);
    
    STDMETHODIMP BufferCB(double SampleTime, BYTE *pBuffer, long BufferLen) {return E_NOTIMPL;}

    // Control the tracking parameters
    void SetParams(int *params);
    void GetParams(int *params);
    
public: // The interface needed by ISampleGrabber Filter
    void unlockImage();
    CamImage * lockImage();
    // fake out any COM ref counting
    STDMETHODIMP_(ULONG) AddRef() { return 2; }
    STDMETHODIMP_(ULONG) Release() { return 1; }
    // fake out any COM QI'ing
    STDMETHODIMP QueryInterface(REFIID riid, void ** ppv) {
        if( riid == IID_ISampleGrabberCB || riid == IID_IUnknown ) {
            *ppv = (void *) static_cast<ISampleGrabberCB*> ( this );
            return NOERROR;
        }    
        return E_NOINTERFACE;
    }
    
protected:
        
    // The image dimension. We need them for change the coordinate of windows 
    // (left_top is (0, 0) to that of the image (left_bottom is (0,0))
    int  m_imgWidth, m_imgHeight;
    
    // For calculating the Frame rate
    double m_dFrameRate, m_dPrevTime;
    
    // The media type we are processing
    AM_MEDIA_TYPE m_mediaType;
    CRITICAL_SECTION m_cs;
       
public:
    CamImage *m_image;    
    HANDLE m_imageReady;
};

///////////////////////////////////////////////////////////////////////
// This class build a graph with a grabbing filter to get each image 
// frame and process it with CamCaptureGrabberCB::SampleCB() callback
// function.
class CamCaptureDirectShow : public CamCaptureGraph  
{
public:
    CamCaptureDirectShow();
    virtual ~CamCaptureDirectShow();
    
protected:

    virtual bool buildCaptureGraph(IBaseFilter *filter = 0);
    
    virtual void addExtraFilters();
    virtual void releaseExtraFilters();
    virtual bool initExtraFilters();
    
    // ColorConverter is needed to convert other color to RGB24
    IBaseFilter *m_pColorConv;
    // Grab filter which get image data for each frame
    IBaseFilter *m_pGrabFilter;
    // The interface to set callback function
    ISampleGrabber *m_pSampleGrabber;
    
public: 
    void unlockImage();
    CamImage *lockImage();
    void setImage(CamImage &image);

    // The callback interface to track object through the frames
    CamCaptureGrabberCB *m_pTrackingCB;
    CamImage * waitForImage(int timeout);
};

#endif // __CAM_CAPTURE_H

#include "camellia.h"
#include "camellia_internals.h"

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

// Can't compile DirectShow code with VC++ 6.0
#ifdef CAM_DIRECTSHOW

#include "cam_capture.h"
#include <winbase.h>

CamCaptureGraph::CamCaptureGraph()
{
    // Put the pointers to NULL to indicate no initialization yet
    m_pCaptureGraphBuilder2 = NULL;
    m_pGraphBuilder  = NULL;
    m_pMediaControl  = NULL;
    m_pVideoWindow   = NULL;
    m_pFilterGraph   = NULL;
    m_pMediaSeeking  = NULL;
    m_pSrcFilter     = NULL;
    // Default setup for the render methods
    m_pRenderCWnd    = NULL;	// show video in new window
    m_fIsCapturing   = false;	// no preview live capture
    m_fIsScaling     = false;	// no scaling
    m_showVideoFlag  = false;    // never display the video content by default
    // Do we want the filter graph to be viewable to GraphEdit
    m_fRegisterFilterGraph = true;
    m_dwGraphRegister = 0;	// init it to indicate if it is registered.

    // Desired default video format: 320x240 * 15 fps
    m_desiredFrameRate = 15; 
    m_desiredHeight = 240;
    m_desiredWidth = 320; 
}

CamCaptureGraph::~CamCaptureGraph()
{
    // Release resouses
    destroyFilterGraph();
}

bool CamCaptureGraph::buildRenderFileGraph()
{
    HRESULT hr;

    addExtraFilters();

    hr = m_pGraphBuilder->RenderFile( m_aVName , NULL );
    if (FAILED(hr)) return false;
    hr = m_pGraphBuilder->QueryInterface(IID_IMediaSeeking,(void**)&m_pMediaSeeking);
    if (FAILED(hr)) { m_pMediaSeeking = NULL; return false;}
    m_pMediaSeeking->SetTimeFormat(&TIME_FORMAT_FRAME);
    return true;
}

bool CamCaptureGraph::buildCaptureGraph(IBaseFilter * pSrcFilter )
{
    HRESULT hr = CoCreateInstance (CLSID_CaptureGraphBuilder2 , NULL, CLSCTX_INPROC,
        IID_ICaptureGraphBuilder2, (void **) &m_pCaptureGraphBuilder2);
    if (FAILED(hr))	return false;  // unable to build graph 
 
    // Attach the filter graph to the capture graph
    hr = m_pCaptureGraphBuilder2->SetFiltergraph(m_pGraphBuilder);
    if (FAILED(hr))	return false;  // unable to build graph 

    if (pSrcFilter == 0) {
        // Use the system device enumerator and class enumerator to find
        // a video capture/preview device, such as a desktop USB video camera.
        hr = findCaptureDevice(&pSrcFilter);
        if (FAILED(hr))	return false;  // unable to build graph 
    }

    m_pSrcFilter = pSrcFilter;

    // Add Capture filter to our graph.
    hr = m_pGraphBuilder->AddFilter(pSrcFilter, L"Video Capture");

    addExtraFilters();
    if (FAILED(hr))	return false;  // unable to build graph 

    // Connect the extra filters into the graph
    IPin* pSourceOut = get_pin( pSrcFilter, PINDIR_OUTPUT );
    hr = m_pGraphBuilder->Render( pSourceOut );

    if (FAILED(hr))	return false;  // unable to build graph 

    // Now that the filter has been added to the graph and we have
    // rendered its stream, we can release this reference to the filter.
    SafeRelease( pSourceOut );

    // Don't release the source filter, it will be released in the destructor
    //SafeRelease( pSrcFilter );
    return true;
}

bool CamCaptureGraph::createFilterGraph()
{
    IPin* pGrabIn = NULL, *pGrabOut = NULL;
    HRESULT hr;

    hr = CoCreateInstance( CLSID_FilterGraph, NULL, CLSCTX_INPROC, 
        IID_IGraphBuilder, (void **)&m_pGraphBuilder );
    if (FAILED(hr)) return false; // unable to build graph 
    else {
        m_pGraphBuilder->QueryInterface(IID_IMediaControl,(void**)&m_pMediaControl);
        m_pGraphBuilder->QueryInterface(IID_IVideoWindow, (void**)&m_pVideoWindow );
        m_pGraphBuilder->QueryInterface(IID_IFilterGraph, (void**)&m_pFilterGraph);

        if (m_fIsCapturing) // Create the capture graph builder
	{ if (!buildCaptureGraph()) return false; }
        else  // Render a video file
	{ if (!buildRenderFileGraph()) return false; }
    }
	// Do some initialization of the extra filters
    initExtraFilters();
    // release the Pin
    SafeRelease( pGrabIn );
    SafeRelease( pGrabOut );

    return true; // success build the graph
}

void CamCaptureGraph::destroyFilterGraph()
{
    this->m_fstartGraph= false;
    stopGraph();
    // release the DirectShow Objects
    SafeRelease( m_pVideoWindow  );
    SafeRelease( m_pGraphBuilder );
    SafeRelease( m_pCaptureGraphBuilder2 );
    SafeRelease( m_pMediaControl );
    SafeRelease( m_pFilterGraph  );
    SafeRelease( m_pMediaSeeking );
    SafeRelease( m_pSrcFilter );
    // release the extra filters resource
    releaseExtraFilters();
}

void CamCaptureGraph::startGraph() 
{
    HRESULT hr;
    long  w, h;
    RECT rc;
    if( m_pMediaControl )
    {
        if (m_pRenderCWnd) { // has render window
            m_pVideoWindow->put_Owner((OAHWND)m_pRenderCWnd);
            //m_pVideoWindow->put_Owner(NULL); // use NULL to display in a seperate window

            m_pVideoWindow->put_WindowStyle(WS_CHILD|WS_CLIPSIBLINGS|WS_CLIPCHILDREN);
            m_pVideoWindow->put_MessageDrain((OAHWND)m_pRenderCWnd );
            //m_pVideoWindow->put_MessageDrain(0);

            // Get the dimension of the video source through the IBasicVideo interface.
            IBasicVideo *pBasicVideo; // used to get the video source's dimension
            m_pGraphBuilder->QueryInterface(IID_IBasicVideo, (void **)&pBasicVideo);
            pBasicVideo->get_SourceWidth(&w);
            pBasicVideo->get_SourceHeight(&h);
            SafeRelease(pBasicVideo);
            // The window's client area
            GetClientRect(m_pRenderCWnd, &rc );

            if (!m_fIsScaling) // display at original size
                m_pVideoWindow->SetWindowPosition( 0, 0, w, h );
            else // if want the video to take full window
                m_pVideoWindow->SetWindowPosition( rc.left, rc.top, rc.right - rc.left, rc.bottom - rc.top );
        }

        // Add the graph to Rot so that the graphedit can view it
        if (m_dwGraphRegister) {
            hr = addGraphToRot(m_pGraphBuilder, &m_dwGraphRegister);
            if (FAILED(hr))
            {
                // Msg(TEXT("Failed to register filter graph with ROT!  hr=0x%x"), hr);
                m_dwGraphRegister = 0;
            }
        }
        //hr = m_pMediaControl->Run();
    }
}

void CamCaptureGraph::stopGraph() 
{
    if( m_pMediaControl )
    {
        m_pMediaControl->Stop();
        m_pVideoWindow->put_Visible(OAFALSE);
        m_pVideoWindow->put_Owner(NULL);
        m_pVideoWindow->put_MessageDrain(0);
    }

    if (m_dwGraphRegister)
        removeGraphFromRot(m_dwGraphRegister);
}

HRESULT CamCaptureGraph::findCaptureDevice(IBaseFilter ** ppSrcFilter)
{
    HRESULT hr;
    IBaseFilter * pSrc = NULL;
    CComPtr <IMoniker> pMoniker =NULL;
    ULONG cFetched;

    // Create the system device enumerator
    CComPtr <ICreateDevEnum> pDevEnum =NULL;

    hr = CoCreateInstance (CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC,
        IID_ICreateDevEnum, (void ** ) &pDevEnum);

    // Create an enumerator for the video capture devices
    CComPtr <IEnumMoniker> pClassEnum = NULL;

    hr = pDevEnum->CreateClassEnumerator (CLSID_VideoInputDeviceCategory, &pClassEnum, 0);
	if (pClassEnum == NULL) return E_FAIL;
	
    // Use the first video capture device on the device list.
    // Note that if the Next() call succeeds but there are no monikers,
    // it will return S_FALSE (which is not a failure).  Therefore, we
    // check that the return code is S_OK instead of using SUCCEEDED() macro.
    if (S_OK == (pClassEnum->Next (1, &pMoniker, &cFetched)))
    {
        // Bind Moniker to a filter object
        hr = pMoniker->BindToObject(0,0,IID_IBaseFilter, (void**)&pSrc);
        if (FAILED(hr))
        {
            // Msg(TEXT("Couldn't bind moniker to filter object!  hr=0x%x"), hr);
            return hr;
        }
    }
    else
    {
        // Msg(TEXT("Unable to access video capture device!"));   
        return E_FAIL;
    }
    // Copy the found filter pointer to the output parameter.
    // Do NOT Release() the reference, since it will still be used
    // by the calling function.
    *ppSrcFilter = pSrc;
    return hr;
}

IPin* CamCaptureGraph::get_pin( IBaseFilter* pFilter, PIN_DIRECTION dir )
{
    IEnumPins*  pEnumPins = 0;
    IPin*       pPin = 0;

    if( pFilter )
    {
        pFilter->EnumPins( &pEnumPins );
        if( pEnumPins != 0 )
        {
            for(;;)
            {
                ULONG  cFetched = 0;
                PIN_DIRECTION pinDir = PIN_DIRECTION(-1); 
                pPin = 0;

                pEnumPins->Next( 1, &pPin, &cFetched );
                if( cFetched == 1 && pPin != 0 )
                {
                    pPin->QueryDirection( &pinDir );
                    if( pinDir == dir ) break;
                    pPin->Release();
                }
            }
            pEnumPins->Release();
        }
    }

    return pPin;
}

HRESULT CamCaptureGraph::addGraphToRot(IUnknown *pUnkGraph, DWORD *pdwRegister) 
{
    IMoniker * pMoniker;
    IRunningObjectTable *pROT;
    WCHAR wsz[128];
    HRESULT hr;

    if (FAILED(GetRunningObjectTable(0, &pROT))) {
        return E_FAIL;
    }

    wsprintfW(wsz, L"FilterGraph %08x pid %08x", (DWORD_PTR)pUnkGraph, 
        GetCurrentProcessId());

    hr = CreateItemMoniker(L"!", wsz, &pMoniker);
    if (SUCCEEDED(hr)) {
        hr = pROT->Register(0, pUnkGraph, pMoniker, pdwRegister);
        pMoniker->Release();
    }
    pROT->Release();
    return hr;
}

void CamCaptureGraph::removeGraphFromRot(DWORD pdwRegister)
{
    IRunningObjectTable *pROT;

    if (SUCCEEDED(GetRunningObjectTable(0, &pROT))) {
        pROT->Revoke(pdwRegister);
        pROT->Release();
    }
}

void CamCaptureGraph::RenderFile(char * vname)
{
    destroyFilterGraph();

    m_fIsCapturing = false;
    if( vname && strlen(vname) > 0 ) // change to wide character
        MultiByteToWideChar( CP_ACP, 0, vname, -1, m_aVName, sizeof(m_aVName)/2 );
    else // did not specify the file name, will be asked in createFilterGraph()
        *m_aVName = '\0';

    if (createFilterGraph())
        startGraph();
    else {
        destroyFilterGraph();
        MessageBox(NULL, "Unable to build Filter Graph", NULL, MB_OK|MB_ICONEXCLAMATION|MB_TASKMODAL);
    }
}

bool CamCaptureGraph::CaptureLive(bool fSetup)
{
    destroyFilterGraph();

    m_fIsCapturing = true;
    m_fShowCaptureProperties = fSetup;

    if (createFilterGraph()) {
        startGraph();
	return true;
    } else {
        destroyFilterGraph();
	return false;
    }
}

LONGLONG CamCaptureGraph::Seek(int offset)
{
    LONGLONG cur_pos = 0;
    if (m_pMediaSeeking) {
        m_pMediaControl->Pause();
        m_pMediaSeeking->GetCurrentPosition(&cur_pos);
        cur_pos += offset;
        m_pMediaSeeking->SetPositions(&cur_pos, AM_SEEKING_AbsolutePositioning,
            NULL, AM_SEEKING_NoPositioning);
        m_pMediaControl->StopWhenReady();
    }
    return cur_pos;
};

bool CamCaptureGraph::createFilterGraph(IMoniker *videoDevice)
{
    IPin* pGrabIn = NULL, *pGrabOut = NULL;
    HRESULT hr;
    IBaseFilter *psrcFilter = 0;

    hr = CoCreateInstance( CLSID_FilterGraph, NULL, CLSCTX_INPROC, 
        IID_IGraphBuilder, (void **)&m_pGraphBuilder );
    if (FAILED(hr)) return false; // unable to build graph 
    else {
        m_pGraphBuilder->QueryInterface(IID_IMediaControl,(void**)&m_pMediaControl);
        m_pGraphBuilder->QueryInterface(IID_IVideoWindow, (void**)&m_pVideoWindow );
        m_pGraphBuilder->QueryInterface(IID_IFilterGraph, (void**)&m_pFilterGraph);

        if (m_fIsCapturing) {
            // Create the capture graph builder
            if(videoDevice != 0)
            {
                IPropertyBag *pBag;

                hr = videoDevice->BindToStorage(0, 0, IID_IPropertyBag, (void **)&pBag);
                if(SUCCEEDED(hr))
                {
                    VARIANT var;
                    var.vt = VT_BSTR;
                    hr = pBag->Read(L"FriendlyName", &var, NULL);
                    if (hr == NOERROR) {
                        lstrcpyW(m_aVName, var.bstrVal);
                        SysFreeString(var.bstrVal);
                    }
                    pBag->Release();
                }
                hr = videoDevice->BindToObject(0, 0, IID_IBaseFilter, (void**)&psrcFilter);
            }
            buildCaptureGraph(psrcFilter);


        } else  // Render a video file
            buildRenderFileGraph();
    }
    // Do some initialization of the extra filters
    initExtraFilters();
    // release the Pin
    SafeRelease( pGrabIn );
    SafeRelease( pGrabOut );

    this->m_fstartGraph = false;
    return true; // success build the graph

}

bool CamCaptureGraph::initCaptureLive (IMoniker * videoDevice, bool fSetup) {
    destroyFilterGraph();

    m_fIsCapturing = true;
    m_fShowCaptureProperties = fSetup;

    if (!createFilterGraph(videoDevice)) {
        destroyFilterGraph();
        MessageBox(NULL, "Unable to build Filter Graph", NULL, MB_OK|MB_ICONEXCLAMATION|MB_TASKMODAL);
        return false;
    }

    return true;
}

void CamCaptureGraph::Play(){ 
    if (!this->m_fstartGraph)
        this->startGraph();
    this->m_fstartGraph = true;
    m_pMediaControl->Run(); 
}

void CamCaptureGraph::showPropPage()
{
    if (m_pSrcFilter) {

        ISpecifyPropertyPages *pSpec;
        CAUUID cauuid;
        HRESULT hr;
        hr = m_pSrcFilter->QueryInterface(IID_ISpecifyPropertyPages, (void **)&pSpec);
        if (hr == S_OK) { // show dialog
            hr = pSpec->GetPages(&cauuid);
            hr = OleCreatePropertyFrame(m_pRenderCWnd, 30, 30, m_aVName, 1,
                (IUnknown **)& m_pSrcFilter, cauuid.cElems, (GUID *)cauuid.pElems, 0, 0, NULL);
            // Release the memory
            CoTaskMemFree(cauuid.pElems);
            pSpec->Release();
        }
    }
}

HRESULT CamCaptureGraph::selectVideoFormat() {
    IAMStreamConfig *pSC;
    // Get the Media Stream config interface
    HRESULT hr = m_pCaptureGraphBuilder2->FindInterface(&PIN_CATEGORY_CAPTURE,
        &MEDIATYPE_Video, m_pSrcFilter, IID_IAMStreamConfig, (void **)&pSC);

    if (!m_fShowCaptureProperties) { 

        // get format being used NOW
        AM_MEDIA_TYPE *pmt;

        hr = pSC->GetFormat(&pmt);

        BITMAPINFOHEADER bih;
        bih.biBitCount = 16;
        bih.biClrImportant = 0;
        bih.biClrUsed = 0;
        bih.biCompression = 0;
        bih.biHeight = m_desiredHeight;
        bih.biPlanes = 1;
        bih.biSize = sizeof(BITMAPINFOHEADER);
        bih.biSizeImage =  m_desiredHeight * m_desiredWidth * 16/ 8;
        bih.biWidth = m_desiredWidth;
        bih.biXPelsPerMeter = 0;
        bih.biYPelsPerMeter = 0;

        VIDEOINFOHEADER vih;
        vih.rcSource.top = 0;
        vih.rcSource.left = 0;
        vih.rcSource.bottom = 0;
        vih.rcSource.right = 0;
        vih.rcTarget.top = 0;
        vih.rcTarget.left = 0;
        vih.rcTarget.bottom = 0;
        vih.rcTarget.right = 0;
        vih.dwBitRate =  m_desiredFrameRate* m_desiredHeight * m_desiredWidth*2; //15 * 640* 480* 24/ 8;
        vih.dwBitErrorRate = 0;
        vih.AvgTimePerFrame = 10000000 / m_desiredFrameRate;
        vih.bmiHeader = bih;

        pmt->bFixedSizeSamples = TRUE;
        pmt->bTemporalCompression = FALSE;
        pmt->cbFormat = sizeof(VIDEOINFOHEADER);
        pmt->formattype = FORMAT_VideoInfo;
        pmt->lSampleSize = m_desiredHeight * m_desiredWidth*2; //UYVY format //640* 480* 24 / 8;
        pmt->majortype = MEDIATYPE_Video;
        pmt->pbFormat = (unsigned char *)&vih;
        pmt->pUnk = 0;

        //pmt->subtype = MEDIASUBTYPE_UYVY; //MEDIASUBTYPE_RGB24;

        hr = pSC->SetFormat(pmt);
    }
    else { // set capture config by property page
        ISpecifyPropertyPages *pSpec;
        CAUUID cauuid;
        hr = pSC->QueryInterface(IID_ISpecifyPropertyPages, (void **)&pSpec);
        if (hr == S_OK) { // show dialog
            hr = pSpec->GetPages(&cauuid);
            hr = OleCreatePropertyFrame(m_pRenderCWnd, 30, 30, NULL, 1,
                (IUnknown **)&pSC, cauuid.cElems, (GUID *)cauuid.pElems, 0, 0, NULL);
            // Release the memory
            CoTaskMemFree(cauuid.pElems);
            pSpec->Release();
        }
    }
    pSC->Release();

    return hr;
}

void CamCaptureGraph::setDesiredVideoFormat(int w, int h, int fps) {
    m_desiredFrameRate = fps;
    m_desiredWidth = w;
    m_desiredHeight = h;
}

void CamCaptureGraph::setVideoFlag(bool flag) {
    m_showVideoFlag = flag;
}

static const GUID CLSID_ColorSpaceConverter = {0x1643E180,0x90F5,0x11CE, {0x97, 0xD5, 0x00, 0xAA, 0x00, 0x55, 0x59, 0x5A }};

CamCaptureGrabberCB::CamCaptureGrabberCB()
{ 
    m_dFrameRate = 0; 
    m_image = 0;

    InitializeCriticalSection(&m_cs);
    m_imageReady = CreateEvent(NULL, false, false, "ImageReady");
}

CamCaptureGrabberCB::~CamCaptureGrabberCB() 
{
    DeleteCriticalSection(&m_cs);
    CloseHandle(m_imageReady);
}

// This callback function is called every time the new image frame is obtained
STDMETHODIMP CamCaptureGrabberCB::SampleCB(double SampleTime, IMediaSample *pSample)
{
    VIDEOINFOHEADER *pvi = (VIDEOINFOHEADER *) m_mediaType.pbFormat;
    BYTE *pData;                // Pointer to the actual image buffer
    pSample->GetPointer(&pData);

    // Get the image properties from the BITMAPINFOHEADER
    m_imgWidth  = pvi->bmiHeader.biWidth;
    m_imgHeight = pvi->bmiHeader.biHeight;

    EnterCriticalSection(&m_cs);

    // Successfully locked the resources
    if (m_image != 0) {
        if (m_image->imageData == NULL) {
            camAllocateImageEx(m_image, m_imgWidth, m_imgHeight, CAM_DEPTH_8U, CAM_CHANNELSEQ_BGR);
        }

        int size = pSample->GetSize();
        // Image is reversed.
        for (int y=0;y<m_imgHeight;y++) {
            memcpy(m_image->imageData+y*m_image->widthStep, pData+m_imgWidth*3*(m_imgHeight-1-y), m_imgWidth*3);
        }
        SetEvent(m_imageReady);
    }

    LeaveCriticalSection(&m_cs);

    m_dFrameRate = 0.8 * m_dFrameRate + 0.2 / (SampleTime - m_dPrevTime);
    m_dPrevTime = SampleTime;

    return S_OK;
}

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CamCaptureDirectShow::CamCaptureDirectShow()
{
    // Set zero for the extra filter (ISampleGrabber) here
    m_pColorConv = NULL;
    m_pGrabFilter = NULL;
    m_pSampleGrabber = NULL;
    m_pTrackingCB = NULL;
}

CamCaptureDirectShow::~CamCaptureDirectShow()
{
    destroyFilterGraph();
    if (m_pTrackingCB)	delete m_pTrackingCB; 
}

void CamCaptureDirectShow::addExtraFilters()
{
    // Instanciate the ISampleGrabber filter
    HRESULT hr = CoCreateInstance(CLSID_SampleGrabber, NULL, CLSCTX_INPROC_SERVER, 
        IID_IBaseFilter, (LPVOID *)&m_pGrabFilter);
    m_pGrabFilter->QueryInterface(IID_ISampleGrabber, (void **)&m_pSampleGrabber);
    if (FAILED(hr)) {
        MessageBox(NULL, "Fail to init ISampleGrabber", NULL, MB_OK|MB_ICONEXCLAMATION|MB_TASKMODAL);
        return;
    }

    // Specify what media type to process, to make sure it is connected correctly
    // We now process full decompressed RGB24 image data
    AM_MEDIA_TYPE   mt;
    ZeroMemory(&mt, sizeof(AM_MEDIA_TYPE));
    mt.majortype = MEDIATYPE_Video;
    mt.subtype = MEDIASUBTYPE_RGB24;
    mt.formattype = FORMAT_VideoInfo;
    hr = m_pSampleGrabber->SetMediaType(&mt);
    // Set working mode as continuous with no buffer
    m_pSampleGrabber->SetOneShot(FALSE);
    m_pSampleGrabber->SetBufferSamples(FALSE);
    // add the grabber into the graph
    hr = m_pFilterGraph->AddFilter(m_pGrabFilter, L"Grabber");

    // Sometimes, we may need the color converter to add in the SampleGrabber
    // Filter. So just add one into the graph. If it is needed, it will be used
    // Otherwise, it will be idle. 
    hr = CoCreateInstance( CLSID_ColorSpaceConverter, NULL, CLSCTX_INPROC_SERVER, 
        IID_IBaseFilter, (void **)&m_pColorConv );
    m_pFilterGraph->AddFilter(m_pColorConv, L"ColorChange");

    // Normally, we dont need to connect the PIN by ourselves. 
    // it will be correctly connected when using Render().
    // So we do not connect these two filters together.

    if (m_showVideoFlag == false) {
        IBaseFilter * pNull;

        hr = CoCreateInstance(CLSID_NullRenderer, NULL, CLSCTX_INPROC_SERVER, 
            IID_IBaseFilter, (LPVOID*) &pNull);
        m_pFilterGraph->AddFilter(pNull, L"NullRender");

        // Null rendered is not automatically connect
        // Do the manual connection

        IPin*		InPin;	// renderer input
        IPin*		OutPin;	// decoder or other filter output;
        IEnumPins*	EnumPins;
        ULONG fetched;

        pNull->EnumPins(&EnumPins);
        EnumPins->Reset();
        EnumPins->Next(1, &InPin, &fetched);
        EnumPins->Release();

        m_pColorConv->EnumPins(&EnumPins);
        EnumPins->Reset();
        EnumPins->Next(1, &OutPin, &fetched);
        EnumPins->Release();

        m_pGraphBuilder->Connect(OutPin, InPin);

        // we could release the reference to the null render
        // after it has been added to the video graph.
        //SafeRelease(pNull);
    }
}

void CamCaptureDirectShow::releaseExtraFilters() {
    // Stop the tracking

    // Release the resourse
    SafeRelease(m_pSampleGrabber);
    SafeRelease(m_pColorConv);
    SafeRelease(m_pGrabFilter);
}

bool CamCaptureDirectShow::initExtraFilters() {
	if (m_pSampleGrabber) {
		// Init the grabber filter
		AM_MEDIA_TYPE   mt;
		// Set the CallBack Interface
		if (!m_pTrackingCB)
			m_pTrackingCB = new CamCaptureGrabberCB();
		m_pSampleGrabber->SetCallback(m_pTrackingCB, 0);

		// Set the media type (needed to get the bmp header info
		m_pSampleGrabber->GetConnectedMediaType(&mt);
		m_pTrackingCB->SetGrabMediaType(mt);
		return true;
	} else return false;
}


// controlled access to the images
CamImage *CamCaptureDirectShow::lockImage()
{
    return m_pTrackingCB->lockImage();

}
void CamCaptureDirectShow::unlockImage()
{
    m_pTrackingCB->unlockImage();
}

CamImage *CamCaptureGrabberCB::lockImage()
{
    EnterCriticalSection(&m_cs);
    return m_image;
}

void CamCaptureGrabberCB::unlockImage()
{
    LeaveCriticalSection(&m_cs);
}

CamImage *CamCaptureDirectShow::waitForImage(int timeout)
{
    DWORD ret = WaitForSingleObject(m_pTrackingCB->m_imageReady, timeout);

    if (ret != WAIT_OBJECT_0)  {
        throw "Timeout";
    }

    return m_pTrackingCB->m_image;
}

void CamCaptureDirectShow::setImage(CamImage &image)
{
    m_pTrackingCB->lockImage();
    m_pTrackingCB->m_image=&image;
    m_pTrackingCB->unlockImage();
}

bool CamCaptureDirectShow::buildCaptureGraph(IBaseFilter * pSrcFilter) {
    HRESULT hr = CoCreateInstance (CLSID_CaptureGraphBuilder2 , NULL, CLSCTX_INPROC,
        IID_ICaptureGraphBuilder2, (void **) &m_pCaptureGraphBuilder2);
    if (FAILED(hr))	return false;  // unable to build graph 
    // Attach the filter graph to the capture graph

    hr = m_pCaptureGraphBuilder2->SetFiltergraph(m_pGraphBuilder);
    if (FAILED(hr))	return false;  // unable to build graph 

    if (pSrcFilter == 0) {
        // Use the system device enumerator and class enumerator to find
        // a video capture/preview device, such as a desktop USB video camera.
        hr = findCaptureDevice(&pSrcFilter);
        if (FAILED(hr))	return false;  // unable to build graph 

    }

    m_pSrcFilter = pSrcFilter;

    // Add Capture filter to our graph.
    hr = m_pGraphBuilder->AddFilter(pSrcFilter, L"Video Capture");

    addExtraFilters();

    if (FAILED(selectVideoFormat()))	return false;  // unable to build graph 

    // Connect the extra filters into the graph
    IPin* pOut = get_pin( pSrcFilter, PINDIR_OUTPUT );
    IPin * pIn = get_pin( m_pGrabFilter, PINDIR_INPUT);

    hr = m_pGraphBuilder->Connect(pOut, pIn);
    SafeRelease(pOut); SafeRelease(pIn);
    if (FAILED(hr))	return false;  // unable to build graph 

    pOut = get_pin(m_pGrabFilter, PINDIR_OUTPUT);
    hr = m_pGraphBuilder->Render( pOut );
    SafeRelease(pOut);
    if (FAILED(hr))	return false;  // unable to build graph 

    // Now that the filter has been added to the graph and we have
    // rendered its stream, we can release this reference to the filter.

    // Don't release the source filter, it will be released in the destructor
    //SafeRelease( pSrcFilter );
    return true;
}

CamCapture::CamCapture(int options)
{
    CoInitialize(NULL); //  First thing first, we must initalize the COM interface
    CamCaptureDirectShow *h=new CamCaptureDirectShow;
    handle=(void*)h;
    h->setVideoFlag((bool)(options & CAM_CAPTURE_DISPLAY));
    if (!h->CaptureLive(!(options & CAM_CAPTURE_AUTO_SOURCE))) {
	delete h;
	handle=NULL;
    } else h->Play();
}

CamCapture::~CamCapture()
{
    delete (CamCaptureDirectShow*)handle;
}

bool CamCapture::capture(CamImage &image)
{
    CamCaptureDirectShow *h=(CamCaptureDirectShow *)handle;
    if (h==NULL) return false;
    h->setImage(image);
    h->waitForImage(1000);
    return true;
}

#else
#ifdef HAVE_LINUX_VIDEODEV_H 

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev.h>

typedef enum {
  IO_METHOD_READ,
  IO_METHOD_MMAP,
} io_method;

typedef struct _CamCaptureV4L 
{
  struct video_capability vid_cap;
  struct video_mbuf vid_buf;
  struct video_mmap vid_mmap;
  struct video_window vid_win;
  struct video_channel vid_chan;
  struct video_picture vid_pic;
  struct video_tuner vid_tuner;

  unsigned char *map;
  
  int fd;
  io_method io;
  
} CamCaptureV4L;

CamCapture::CamCapture(int options)
{
    CamCaptureV4L *ptr=new CamCaptureV4L;
    handle=(void*)ptr;

    char *sdevice="/dev/video0";
    ptr->fd = open(sdevice, O_RDWR /* required */ | O_NONBLOCK, 0);
    if (ptr->fd == -1) {
        char s[256];
        sprintf(s,"Couldn't open %s",sdevice);
        camSetErrorStr(s);
	delete ptr; handle = NULL;
        return;
    }

    if (ioctl(ptr->fd, VIDIOCGCAP, &ptr->vid_cap) == -1) {
	delete ptr; handle = NULL;
        camError("CamCapture","VIDIOCGCAP");
        return;
    }

    if (!(ptr->vid_cap.type & VID_TYPE_CAPTURE)) {
	delete ptr; handle = NULL;
        char s[256];
        sprintf(s,"%s is not a video capture device",sdevice);
        camError("CamCapture",s);
        return;
    }

    ptr->map = (unsigned char *)-1;

    if ((options & CAM_CAPTURE_USE_READ) || (ioctl(ptr->fd, VIDIOCGMBUF, &ptr->vid_buf) == -1)) {
        ptr->io = IO_METHOD_READ;
    } else {
        ptr->io = IO_METHOD_MMAP;
        ptr->map = (unsigned char *)mmap(0, ptr->vid_buf.size, PROT_READ|PROT_WRITE, MAP_SHARED, ptr->fd, 0);
        if ((unsigned char *)-1 == (unsigned char *)ptr->map) {
            // fprintf(stderr,"mmap() failed: falling back to read() method");
            ptr->io = IO_METHOD_READ;
        }
    }

    if (ioctl(ptr->fd, VIDIOCGWIN, &ptr->vid_win) == -1) {
	delete ptr; handle = NULL;
        camError("CamCapture","VIDIOCGWIN");
        return;
    }

    if (ioctl(ptr->fd, VIDIOCGCHAN, &(ptr->vid_chan)) == -1) {
	delete ptr; handle = NULL;
        camError("CamCapture","VIDIOCGCHAN");
        return;
    }
    ptr->vid_chan.channel = 0;
    ptr->vid_chan.norm = VIDEO_MODE_PAL;
    if (ioctl(ptr->fd, VIDIOCSCHAN, &(ptr->vid_chan)) == -1) {
	delete ptr; handle = NULL;
        camError("CamCapture","VIDIOCSCHAN");
        return;
    }

    ptr->vid_mmap.format = VIDEO_PALETTE_RGB24;

    if (ioctl(ptr->fd, VIDIOCGPICT, &(ptr->vid_pic)) == -1) {
	delete ptr; handle = NULL;
        camError("CamCapture","VIDIOCGPICT");
        return;
    }
    ptr->vid_pic.palette = VIDEO_PALETTE_RGB24;
    if (ioctl(ptr->fd, VIDIOCSPICT, &(ptr->vid_pic)) == -1) {
	delete ptr; handle = NULL;
        camError("CamCapture","VIDIOCSPICT");
        return;
    }
}

CamCapture::~CamCapture()
{
    CamCaptureV4L *ptr=(CamCaptureV4L*)handle;
    if (!ptr) return;
    if (ptr->io == IO_METHOD_MMAP)
        munmap(ptr->map, ptr->vid_buf.size);
    close(ptr->fd);
    delete ptr;
}

bool CamCapture::capture(CamImage &image)
{
    CamCaptureV4L *ptr=(CamCaptureV4L*)handle;

    if (!ptr) return false;

    // Check image allocation
    if (image.imageData == NULL) {
        camAllocateRGBImage(&image,ptr->vid_win.width,ptr->vid_win.height);
    }
    if ((image.width != (int)ptr->vid_win.width)||(image.height != (int)ptr->vid_win.height)) {
        camDeallocateImage(&image);
        camAllocateRGBImage(&image,ptr->vid_win.width,ptr->vid_win.height);
    }
    // Video devices generally don't care about padding and lines alignement
    if ((int)ptr->vid_win.width*3 != image.widthStep) {
        return false;
    }

    unsigned long i, imgsz = ptr->vid_win.width * ptr->vid_win.height * 3;
    unsigned char buf, *p = image.imageData;

    switch(ptr->io)
    {
        case IO_METHOD_MMAP:
            ptr->vid_mmap.frame = 0;
            ptr->vid_mmap.width = ptr->vid_win.width;
            ptr->vid_mmap.height = ptr->vid_win.height;
            if(ioctl(ptr->fd, VIDIOCMCAPTURE, &ptr->vid_mmap) == -1) {
                camError("CamCapture","VIDIOCMCAPTURE");
                return false;
            }
            if(ioctl(ptr->fd, VIDIOCSYNC, &ptr->vid_mmap.frame) == -1) {
                camError("CamCapture","VIDIOCSYNC");
                return false;
            }
            memcpy(image.imageData,ptr->map,imgsz);
            break;
        case IO_METHOD_READ:
            while(read(ptr->fd, image.imageData, imgsz) <= 0);
            break;
    }

    // Converts from BGR to RGB
    i = ptr->vid_win.width * ptr->vid_win.height;
    while(0 < i)
    {
        buf = p[2]; p[2] = p[0]; p[0] = buf;
        p += 3;
        i--;
    }

    return true;
}

#else

CamCapture::CamCapture(int options)
{
    handle=NULL;
}

CamCapture::~CamCapture()
{
}

bool CamCapture::capture(CamImage &image)
{
    return false;
}
#endif
#endif 

void* camCaptureInit(int options)
{
    CamCapture *h=new CamCapture(options);
    if (!h->ready()) { delete h; return NULL; }
    return h;
}

int camCapture(void *handle, CamImage *image)
{
    if (handle==NULL) return NULL;
    CamCapture *h=(CamCapture*)handle;
    return (h->capture(*image))?1:0;
}

int camCaptureOver(void *handle)
{
    if (handle==NULL) return 1;
    CamCapture *h=(CamCapture*)handle;
    delete h;
    return 1;
}


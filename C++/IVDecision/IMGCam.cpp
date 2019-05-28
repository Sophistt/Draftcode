#include "StdAfx.h"
#include "IMGCam.h"

CIMGCam::CIMGCam(void)
{
	if( ! DShowLib::InitLibrary( "ISB3200016679" ) )
	{
		/*return*/ ;
	}
	pGrabber = new DShowLib::Grabber();
	ASSERT( pGrabber );
}

CIMGCam::~CIMGCam(void)
{
	delete pGrabber;
}

int CIMGCam::InitCam()
{

	DShowLib::Grabber::tVidCapDevListPtr pVidCapDevList = pGrabber->getAvailableVideoCaptureDevices();

	bool bMustRestart = false;
	if (!pGrabber->isDevValid())
	{
		pGrabber->closeDev();
	}
	else
	{
		if( pGrabber->isLive() )
		{
			bMustRestart = true;
			pGrabber->stopLive();
		}
	}

	// Show the device page.
	pGrabber->showDevicePage();


	// Check if there is a valid device.
	if( pGrabber->isDevValid() )
	{

		FrameTypeInfoArray acceptedTypes = FrameTypeInfoArray::createRGBArray();

		pHandlerSink = FrameHandlerSink::create( acceptedTypes, 1 );
		pHandlerSink->setSnapMode( true );

		pGrabber->setSinkType( pHandlerSink );

		///////////////////////////////
		pGrabber->startLive( false );
		//Sleep( 250 ); // give the device time to adjust automatic settings i.e. auto exposure
		pHandlerSink->snapImages( 1, 1000 );
		//pGrabber->stopLive();
		MemBufferCollection::tMemBufferPtr pActiveBuf = pHandlerSink->getLastAcqMemBuffer();
		/////////////////////////////

		//// get pointer to the image data
		BYTE* pbImgData = pActiveBuf->getPtr();

		SIZE dim = pActiveBuf->getFrameType().dim;

		img_width = dim.cx;
		img_height = dim.cy;

		//srcImage = cvCreateImage(cvSize(dim.cx, dim.cy), 8, 4);
		//memcpy( srcImage->imageData, pbImgData, 4*(dim.cx*dim.cy)*sizeof(BYTE));

		//cvNamedWindow("aa",0);
		//cvShowImage("aa", srcImage);

		//SetTimer(1, 20, NULL);

		return 1;
	}
	else
	{
		return 0;//AfxMessageBox( "No device was selected." );
	}
}

int CIMGCam::CloseCam()
{
	// Stop live mode.
	pGrabber->stopLive();

	// this call will also succeed if no device is open
	pGrabber->closeDev();

	return 1;
}

int CIMGCam::SnapOneFrame( char *imageData )
{

	Error e = pHandlerSink->snapImages( 1 );

	if( e.isError() )
	{
		// Display an error.
		//::MessageBox( 0, e.toString().c_str(), "Error", MB_OK|MB_ICONERROR );
		return 0;
	}
	else
	{
		//pGrabber->stopLive();
		MemBufferCollection::tMemBufferPtr pActiveBuf = pHandlerSink->getLastAcqMemBuffer();
		///////////////////////////

		// get pointer to the image data
		BYTE* pbImgData = pActiveBuf->getPtr();

		memcpy( imageData, pbImgData, 4*(img_width*img_height)*sizeof(BYTE));

		return 1;
	}
}
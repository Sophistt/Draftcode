#pragma once

using namespace _DSHOWLIB_NAMESPACE;


class CIMGCam
{
public:
	CIMGCam(void);
public:
	~CIMGCam(void);

public:
	int InitCam();
	int CloseCam();
	int SnapOneFrame( char *imageData );

	int img_width;
	int img_height;

private:
	DShowLib::Grabber* pGrabber;
	//IplImage *srcImage;
	smart_ptr<FrameHandlerSink> pHandlerSink;
};

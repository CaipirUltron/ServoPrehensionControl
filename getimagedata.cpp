#include "mex.h"
#include <stdio.h>
#include <Windows.h>
#include "NuiApi.h"
#include <math.h>

#include <iostream>

#define __No__NuiSkeleton_h__
#define imageHeight 480
#define imageWidth  640
bool Nui_GotDepthAlert(double  Depthes[][imageWidth]);
bool Nui_GotVideoAlert(double  RGBinfo[][imageWidth][3]);
void KinectInit();
void KinectShutdown();
HRESULT	hr;
BSTR m_instanceId;
INuiSensor * m_pNuiSensor = NULL;
HANDLE m_pDepthStreamHandle = NULL;
HANDLE m_pVideoStreamHandle = NULL;
const NUI_IMAGE_FRAME * pImageFrame = NULL;

NUI_IMAGE_VIEW_AREA area;

int main(){}

void KinectInit()
{
    if ( !m_pNuiSensor )
    {
        HRESULT hr = NuiCreateSensorByIndex(0, &m_pNuiSensor);

        if ( FAILED(hr) )
        {
            mexErrMsgIdAndTxt("Getimagedata:init","Error when initializing Kinect.");
            return;
        }

        SysFreeString(m_instanceId);

        m_instanceId = m_pNuiSensor->NuiDeviceConnectionId();
    }
    
	hr=m_pNuiSensor->NuiInitialize( NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);

	hr = m_pNuiSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,
         NUI_IMAGE_RESOLUTION_640x480, 0, 2, NULL, &m_pVideoStreamHandle );

	hr = m_pNuiSensor->NuiImageStreamOpen( NUI_IMAGE_TYPE_DEPTH,
		 NUI_IMAGE_RESOLUTION_320x240, 0, 2, NULL, &m_pDepthStreamHandle);
    
//     LONG *pangle;
//     hr = NuiCameraElevationSetAngle(0.0);
//     if ( FAILED(hr) ) {
//         mexErrMsgIdAndTxt("Getimagedata:init", "Error when zeroing Kinect angle.");
//         return;
//     }

//     std::cout << "Angle: " << pangle[0] << std::endl;    
}

void KinectShutdown()
{
	m_pNuiSensor->NuiShutdown();
    m_pNuiSensor->Release();
    m_pNuiSensor = NULL;
}

bool Nui_GotDepthAlert(double Depthes[][imageWidth/2])
{
    const NUI_IMAGE_FRAME * pImageFrame = NULL;
    HRESULT hr = NuiImageStreamGetNextFrame(
        m_pDepthStreamHandle,	
        66,//2/30 millisecond timeout
        &pImageFrame );
    if( FAILED( hr ) )
		return 1;
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    if( LockedRect.Pitch != 0 )
    {
        BYTE * pBuffer = (BYTE*) LockedRect.pBits;
        USHORT * pBufferRun = (USHORT*) pBuffer;
//         const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);
		double RealDepth;
//         long vx,vy;
		for( int y = 0 ; y < imageHeight/2 ; y++ )
        {	
            for( int x = 0 ; x < imageWidth/2 ; x++ )
            {
                pBufferRun++;
				//RealDepth = (*pBufferRun & 0x0fff);
                //USHORT depth = pBufferRun->depth;
                RealDepth = (*pBufferRun >> 3);
//                 NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
//                         NUI_IMAGE_RESOLUTION_640x480,
//                         NUI_IMAGE_RESOLUTION_320x240,
//                         &area,
//                         x,y, (*pBufferRun),
//                         &vx,&vy);
//                 vx = max(0, min(vx, imageWidth - 2));
//                 vy = max(0, min(vy, imageHeight - 2));
				Depthes[y][x] = RealDepth;
                //++pBufferRun;
            }
        }	
    }
    else
    {
	  //OutputDebugString( L"Buffer length of received texture is bogus\r\n" );
    }
    NuiImageStreamReleaseFrame( m_pDepthStreamHandle, pImageFrame );
	return 0;
}

bool Nui_GotVideoAlert(double RGBinfo[][imageWidth][3])
{
    const NUI_IMAGE_FRAME * pImageFrame = NULL;

    HRESULT hr = NuiImageStreamGetNextFrame(
        m_pVideoStreamHandle,
        66,
        &pImageFrame );
    if( FAILED( hr ) )
        return 1;
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    area = pImageFrame->ViewArea;
    if( LockedRect.Pitch != 0 )
    {
        BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		int base, b, a= 0;
		for (int j= 0;j< imageHeight*4;j+= 4){
			b= 0;
			base = j*imageWidth;
			for(int i= 0;i< (imageWidth*4);i+= 4)
			{
				RGBinfo[a][b][0]= (double)pBuffer[base+i+2]; //R
				RGBinfo[a][b][1]= (double)pBuffer[base+i+1]; //G
				RGBinfo[a][b][2]= (double)pBuffer[base+i+0]; //B
				b++;
			}
			a++;
		}
    }
	else
    {
		//OutputDebugString( L"Buffer length of received texture is bogus\r\n" ); 
    }
    NuiImageStreamReleaseFrame( m_pVideoStreamHandle, pImageFrame );
	return 0;
}

void mexFunction( int nlhs, mxArray *plhs[],
                  int nrhs, const mxArray *prhs[])
{	
	    if(nlhs!=2)
		{
			mexErrMsgIdAndTxt("Getimagedata:nlhs","One output required.");
		}
		double *Dep, *Rgb, *Option;
		int dims[3] = {480, 640, 3};
		plhs[0] = mxCreateDoubleMatrix(240, 320, mxREAL);
		plhs[1] = mxCreateNumericArray(3, dims, mxDOUBLE_CLASS, mxREAL);
		Dep = mxGetPr(plhs[0]);
		Rgb = mxGetPr(plhs[1]);
		Option = mxGetPr(prhs[0]);
		double (*Depxy)[320] = (double (*)[320])Dep;
		double (*Rgbxyz)[640][3] = (double (*)[640][3])Rgb;
		switch ((int)*Option)
		{
			case 1 :
				KinectInit();			
				mexLock();
				break;
			case 2 :
                Nui_GotVideoAlert(Rgbxyz);;
				Nui_GotDepthAlert(Depxy);
				break;
			case 3:
				KinectShutdown();
				mexUnlock();
				break;
			default:
				break;
		}
}
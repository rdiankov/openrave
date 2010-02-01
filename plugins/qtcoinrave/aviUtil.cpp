// Copyright (C) 2006-2008 Carnegie Mellon University (rdiankov@cs.cmu.edu)
//
// This file is part of OpenRAVE.
// OpenRAVE is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifdef _WIN32

#define AVIIF_KEYFRAME  0x00000010L // this frame is a key frame.

#include <string>
#include <list>

#include <windows.h>
#include <memory.h>
#include <mmsystem.h>
#include <vfw.h>

BOOL AVI_Init()
{
    /* first let's make sure we are running on 1.1 */
    WORD wVer = HIWORD(VideoForWindowsVersion());
    if (wVer < 0x010a){
         /* oops, we are too old, blow out of here */
         //MessageBeep(MB_ICONHAND);
         MessageBoxA(NULL, "Cant't init AVI File - Video for Windows version is to old", "Error", MB_OK|MB_ICONSTOP);
         return FALSE;
    }

    AVIFileInit();

    return TRUE;
}

BOOL AVI_FileOpenWrite(PAVIFILE * pfile, const char* filename)
{
    HRESULT hr = AVIFileOpenA(pfile,           // returned file pointer
                   filename,                  // file name
                   OF_WRITE | OF_CREATE,      // mode to open file with
                   NULL);                     // use handler determined
                                              // from file extension....
    if (hr != AVIERR_OK)
            return FALSE;

    return TRUE;
}

DWORD getFOURCC(const char* value)
{
	if(stricmp(value, "DIB") == 0)
	{
		return mmioFOURCC(value[0],value[1],value[2],' ');
	}
	else if((stricmp(value, "CVID") == 0)
		 || (stricmp(value, "IV32") == 0)
		 || (stricmp(value, "MSVC") == 0)
		 || (stricmp(value, "IV50") == 0))
	{
		return mmioFOURCC(value[0],value[1],value[2],value[3]);
	}
	else
	{
		return NULL;
	}
}

// Fill in the header for the video stream....
// The video stream will run in rate ths of a second....
BOOL AVI_CreateStream(PAVIFILE pfile, PAVISTREAM * ps, int rate, // sample/second
                      unsigned long buffersize, int rectwidth, int rectheight,
					  const char* _compressor)
{
		AVISTREAMINFO strhdr;
		memset(&strhdr, 0, sizeof(strhdr));
        strhdr.fccType                = streamtypeVIDEO;// stream type
		strhdr.fccHandler             = getFOURCC(_compressor);
        //strhdr.fccHandler             = 0; // no compression!
		//strhdr.fccHandler             = mmioFOURCC('D','I','B',' '); // Uncompressed
		//strhdr.fccHandler             = mmioFOURCC('C','V','I','D'); // Cinpak
		//strhdr.fccHandler             = mmioFOURCC('I','V','3','2'); // Intel video 3.2
		//strhdr.fccHandler             = mmioFOURCC('M','S','V','C'); // Microsoft video 1
		//strhdr.fccHandler             = mmioFOURCC('I','V','5','0'); // Intel video 5.0
		//strhdr.dwFlags                = AVISTREAMINFO_DISABLED;
		//strhdr.dwCaps                 = 
		//strhdr.wPriority              = 
		//strhdr.wLanguage              = 
        strhdr.dwScale                = 1;
        strhdr.dwRate                 = rate;               // rate fps
		//strhdr.dwStart                =  
		//strhdr.dwLength               = 
		//strhdr.dwInitialFrames        = 
        strhdr.dwSuggestedBufferSize  = buffersize;
		strhdr.dwQuality              = -1; // use the default
		//strhdr.dwSampleSize           = 
        SetRect(&strhdr.rcFrame, 0, 0,              // rectangle for stream
            (int) rectwidth,
            (int) rectheight);
		//strhdr.dwEditCount            = 
		//strhdr.dwFormatChangeCount    =
		//strcpy(strhdr.szName, "Full Frames (Uncompressed)");

        // And create the stream;
        HRESULT hr = AVIFileCreateStream(pfile,             // file pointer
                                 ps,                // returned stream pointer
                                 &strhdr);          // stream header
        if (hr != AVIERR_OK) {
                return FALSE;
        }

        return TRUE;
}

std::string getFOURCCVAsString(DWORD value)
{
    std::string returnValue = "";
	if( value == 0 )
		return returnValue;

	DWORD ch0 = value & 0x000000FF;
	returnValue.push_back((char) ch0);
	DWORD ch1 = (value & 0x0000FF00)>>8;
	returnValue.push_back((char) ch1);
	DWORD ch2 = (value & 0x00FF0000)>>16;
	returnValue.push_back((char) ch2);
	DWORD ch3 = (value & 0xFF000000)>>24;
	returnValue.push_back((char) ch3);

	return returnValue;
}

std::string dumpAVICOMPRESSOPTIONS(AVICOMPRESSOPTIONS opts)
{
	char tmp[255];
    std::string returnValue = "Dump of AVICOMPRESSOPTIONS\n";

	returnValue += "DWORD  fccType = streamtype("; returnValue += getFOURCCVAsString(opts.fccType); returnValue += ")\n";
	returnValue += "DWORD  fccHandler = "; returnValue += getFOURCCVAsString(opts.fccHandler); returnValue += "\n";

	_snprintf(tmp, 255, "DWORD  dwKeyFrameEvery = %d\n", opts.dwKeyFrameEvery);
	returnValue += tmp;

	_snprintf(tmp, 255, "DWORD  dwQuality = %d\n", opts.dwQuality);
	returnValue += tmp;

	_snprintf(tmp, 255, "DWORD  dwBytesPerSecond = %d\n", opts.dwBytesPerSecond);
	returnValue += tmp;

	if((opts.dwFlags & AVICOMPRESSF_DATARATE) == AVICOMPRESSF_DATARATE){strcpy(tmp, "DWORD  fccType = AVICOMPRESSF_DATARATE\n");}
	else if((opts.dwFlags & AVICOMPRESSF_INTERLEAVE) == AVICOMPRESSF_INTERLEAVE){strcpy(tmp, "DWORD  fccType = AVICOMPRESSF_INTERLEAVE\n");}
	else if((opts.dwFlags & AVICOMPRESSF_KEYFRAMES) == AVICOMPRESSF_KEYFRAMES){strcpy(tmp, "DWORD  fccType = AVICOMPRESSF_KEYFRAMES\n");}
	else if((opts.dwFlags & AVICOMPRESSF_VALID) == AVICOMPRESSF_VALID){strcpy(tmp, "DWORD  fccType = AVICOMPRESSF_VALID\n");}
	else {_snprintf(tmp, 255, "DWORD  dwFlags = Unknown(%d)\n", opts.dwFlags);}
	returnValue += tmp;

	_snprintf(tmp, 255, "LPVOID lpFormat = %d\n", opts.lpFormat);
	returnValue += tmp;

	_snprintf(tmp, 255, "DWORD  cbFormat = %d\n", opts.cbFormat);
	returnValue += tmp;

	_snprintf(tmp, 255, "LPVOID lpParms = %d\n", opts.lpParms);
	returnValue += tmp;

	_snprintf(tmp, 255, "DWORD  cbParms = %d\n", opts.cbParms); 
	returnValue += tmp;

	_snprintf(tmp, 255, "DWORD  dwInterleaveEvery = %d\n", opts.dwInterleaveEvery);
	returnValue += tmp;

	return returnValue;
}

BOOL AVI_SetOptions(PAVISTREAM * ps, PAVISTREAM * psCompressed, LPBITMAPINFOHEADER lpbi,
					const char* _compressor)
{
    AVICOMPRESSOPTIONS opts;
    AVICOMPRESSOPTIONS FAR * aopts[1] = {&opts};

    memset(&opts, 0, sizeof(opts));
    opts.fccType = streamtypeVIDEO;
    opts.fccHandler = getFOURCC(_compressor);

    /* display the compression options dialog box if specified compressor is unknown */
    if(getFOURCC(_compressor) == NULL)
    {
	    if (!AVISaveOptions(NULL, 0, 1, ps, (LPAVICOMPRESSOPTIONS FAR *) &aopts))
	    {
		    return FALSE;
	    }
    }		

    HRESULT hr = AVIMakeCompressedStream(psCompressed, *ps, &opts, NULL);
    if (hr != AVIERR_OK) {
            return FALSE;
    }

    hr = AVIStreamSetFormat(*psCompressed, 0,
                           lpbi,                    // stream format
                           lpbi->biSize             // format size
                               + lpbi->biClrUsed * sizeof(RGBQUAD)
                               );
    if (hr != AVIERR_OK) {
    return FALSE;
    }

    return TRUE;
}

BOOL AVI_SetText(PAVIFILE pfile, PAVISTREAM psText, char *szText, int width, int height, int TextHeight)
{
    // Fill in the stream header for the text stream....
    AVISTREAMINFO strhdr;
    DWORD dwTextFormat;
    // The text stream is in 60ths of a second....

    memset(&strhdr, 0, sizeof(strhdr));
    strhdr.fccType                = streamtypeTEXT;
    strhdr.fccHandler             = mmioFOURCC('D', 'R', 'A', 'W');
    strhdr.dwScale                = 1;
    strhdr.dwRate                 = 60;
    strhdr.dwSuggestedBufferSize  = sizeof(szText);
    SetRect(&strhdr.rcFrame, 0, (int) height,
        (int) width, (int) height + TextHeight); // #define TEXT_HEIGHT 20

    // ....and create the stream.
    HRESULT hr = AVIFileCreateStream(pfile, &psText, &strhdr);
    if (hr != AVIERR_OK) {
            return FALSE;
    }

    dwTextFormat = sizeof(dwTextFormat);
    hr = AVIStreamSetFormat(psText, 0, &dwTextFormat, sizeof(dwTextFormat));
    if (hr != AVIERR_OK) {
            return FALSE;
    }

    return TRUE;
}

BOOL AVI_AddFrame(PAVISTREAM psCompressed, int time, LPBITMAPINFOHEADER lpbi)
{
	int ImageSize = lpbi->biSizeImage;
	if (ImageSize == 0)
	{
		if (lpbi->biBitCount == 24)
		{
			ImageSize = lpbi->biWidth * lpbi->biHeight * 3;
		}
	}
	HRESULT hr = AVIStreamWrite(psCompressed, // stream pointer
		time, // time of this frame
		1, // number to write
		(LPBYTE) lpbi + // pointer to data
		lpbi->biSize +
		lpbi->biClrUsed * sizeof(RGBQUAD),
		ImageSize, // lpbi->biSizeImage, // size of this frame
		AVIIF_KEYFRAME, // flags....
		NULL,
		NULL);
	if (hr != AVIERR_OK)
	{
		char strMsg[255];
		_snprintf(strMsg, 255, "Error: AVIStreamWrite, error %d",hr);
		MessageBoxA(NULL, strMsg, "", MB_OK);
		return FALSE;
	}
	
	return TRUE;
}

BOOL AVI_AddText(PAVISTREAM psText, int time, char *szText)
{
    int iLen = strlen(szText);

    HRESULT hr = AVIStreamWrite(psText,
                    time,
                    1,
                    szText,
                    iLen + 1,
                    AVIIF_KEYFRAME,
                    NULL,
                    NULL);
    if (hr != AVIERR_OK)
            return FALSE;

    return TRUE;
}

BOOL AVI_CloseStream(PAVISTREAM ps, PAVISTREAM psCompressed, PAVISTREAM psText)
{
        if (ps)
                AVIStreamClose(ps);

        if (psCompressed)
                AVIStreamClose(psCompressed);

        if (psText)
                AVIStreamClose(psText);



        return TRUE;
}

BOOL AVI_CloseFile(PAVIFILE pfile)
{
    if (pfile)
        AVIFileClose(pfile);

    return TRUE;
}

BOOL AVI_Exit()
{
    AVIFileExit();

    return TRUE;
}

std::list<std::pair<int,string> > GET_CODECS() { return std::list<std::pair<int,string> >(); }

/* Here are the additional functions we need! */
static PAVIFILE pfile = NULL; 
static PAVISTREAM ps = NULL;
static PAVISTREAM psCompressed = NULL; 
static int count = 0;
static int s_biSizeImage = 0;
// Initialization... 
bool START_AVI(const char* file_name, int _frameRate, int width, int height, int bits, int codecid=-1)
{
    if(! AVI_Init())
	{
		return false;
	}

    if(! AVI_FileOpenWrite(&pfile, file_name))
	{
		return false;
	}

    if(! AVI_CreateStream(pfile, &ps, _frameRate, width*height/bits, width, height, "none")) {
        return false;
    } 

    BITMAPINFOHEADER bi;
    memset(&bi, 0, sizeof(bi));
    bi.biSize = sizeof(BITMAPINFOHEADER);
    bi.biWidth = width;
    bi.biHeight = height;
    bi.biPlanes = 1;
    bi.biBitCount = bits;
    bi.biCompression = BI_RGB;
    bi.biSizeImage = width * height * bits /8;
    if(! AVI_SetOptions(&ps, &psCompressed, &bi, "none")) {
        return false;
    }

    s_biSizeImage = bi.biSizeImage;
	return true;
}

//Now we can add frames
// ie. ADD_FRAME_FROM_DIB_TO_AVI(yourDIB, "CVID", 25);
bool ADD_FRAME_FROM_DIB_TO_AVI(void* pdata)
{
    HRESULT hr = AVIStreamWrite(psCompressed, // stream pointer
		count, // time of this frame
		1, // number to write
		pdata,
		s_biSizeImage, // lpbi->biSizeImage, // size of this frame
		AVIIF_KEYFRAME, // flags....
		NULL,
		NULL);
	if (hr != AVIERR_OK)
	{
		char strMsg[255];
		_snprintf(strMsg, 255, "Error: AVIStreamWrite, error %d", hr);
		MessageBoxA(NULL, strMsg, "", MB_OK);
		return FALSE;
	}

	count++;
	return true;
}

// The end... 
bool STOP_AVI()
{
    count = 0;
    if(! AVI_CloseStream(ps, psCompressed, NULL))
    {
        return false;
    }

    if(! AVI_CloseFile(pfile))
    {
        return false;
    }

    if(! AVI_Exit())
    {
        return false;
    }

    return true;
} 

#else

#include "qtcoin.h"
#include <list>

#if defined(ENABLE_FFMPEG)

extern "C" {
#ifdef HAVE_NEW_FFMPEG
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#else
#include <ffmpeg/avformat.h>
#include <ffmpeg/avcodec.h>
#endif
}

//#include <malloc.h>

static AVFormatContext *output = NULL;
static AVStream *stream = NULL;
static AVFrame *picture = NULL;
static AVFrame *yuv420p = NULL;
static char *picture_buf = NULL, *outbuf = NULL;
static int picture_size = 0;
static int outbuf_size = 0;

bool STOP_AVI()
{
    if( output == NULL )
        return false;

    RAVELOG_INFOA("stopping avi\n");
	free(picture_buf); picture_buf = NULL;
	free(picture); picture = NULL;
	free(yuv420p); yuv420p = NULL;
    free(outbuf); outbuf = NULL;
    avcodec_close(stream->codec);

	av_write_trailer(output);
#if LIBAVFORMAT_VERSION_INT >= (52<<16)
	url_fclose(output->pb);
#else
    url_fclose(&output->pb);
#endif
	av_freep(&output->streams[0]);
	av_free(output); output = NULL;
    
    return true;
}

std::list<std::pair<int,string> > GET_CODECS() {
    std::list<std::pair<int,string> > lcodecs;
    av_register_all();
	AVOutputFormat *fmt;
	fmt = first_oformat;
	while (fmt != NULL) {
        lcodecs.push_back(make_pair((int)fmt->video_codec,fmt->long_name));
		fmt = fmt->next;
	}
    return lcodecs;
}

bool START_AVI(const char* filename, int _frameRate, int width, int height, int bits, int codecid)
{
    if( bits != 24 ) {
        RAVELOG_WARNA("START_AVI only supports 24bits\n");
        return false;
    }
    
	AVOutputFormat *fmt;
	AVCodecContext *codec_ctx;
	AVCodec *codec;

    av_register_all();

	if (output != NULL) STOP_AVI();

    CodecID video_codec = codecid == -1 ? CODEC_ID_MPEG4 : (CodecID)codecid;
	fmt = first_oformat;
	while (fmt != NULL) {
		if (fmt->video_codec == video_codec)
            break;
		fmt = fmt->next;
	}
	if (fmt == NULL) {
		RAVELOG_WARNA("START_AVI: mpeg4 codec not found\n");
        return false;
	}

	output = (AVFormatContext*)av_mallocz(sizeof(AVFormatContext));
	if (output == NULL) {
		RAVELOG_WARNA("START_AVI: Out of Memory\n");
        return false;
	}

	output->oformat = fmt;
	snprintf(output->filename, sizeof(output->filename), "%s", filename);

	stream = av_new_stream(output, 0);
	if (stream == NULL) {
		RAVELOG_WARNA("START_AVI: Out of Memory\n");
        return false;
	}

	codec_ctx = stream->codec;
	codec_ctx->codec_id = video_codec;
	codec_ctx->codec_type = CODEC_TYPE_VIDEO;
	codec_ctx->bit_rate = 4000000;
	codec_ctx->width = width;
	codec_ctx->height = height;
	codec_ctx->time_base= (AVRational){1,_frameRate};
    codec_ctx->gop_size = 10;
    codec_ctx->max_b_frames = 1;
    codec_ctx->pix_fmt = PIX_FMT_YUV420P;

	if (av_set_parameters(output, NULL) < 0) {
		RAVELOG_WARNA("START_AVI: set parameters failed\n");
        return false;
	}

	codec = avcodec_find_encoder(codec_ctx->codec_id);
	if (codec == NULL) {
		RAVELOG_WARNA("START_AVI: codec not found\n");
        return false;
	}

    RAVELOG_DEBUGA("opening %s, w:%d h:%dx fps:%d, codec: %s\n", output->filename, width, height, _frameRate, codec->name);

	if (avcodec_open(codec_ctx, codec) < 0) {
		RAVELOG_WARNA("START_AVI: Unable to open codec\n");
        return false;
	}

	if (url_fopen(&output->pb, filename, URL_WRONLY) < 0) {
		RAVELOG_WARNA("START_AVI: Unable to open %s for writing\n", filename);
        return false;
	}

	av_write_header(output);

	picture = avcodec_alloc_frame();
	yuv420p = avcodec_alloc_frame();

	outbuf_size = 100000;
	outbuf = (char*)malloc(outbuf_size);
	if (outbuf == NULL) {
		RAVELOG_WARNA("START_AVI: Out of Memory\n");
        return false;
	}

	picture_size = avpicture_get_size(PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);
    picture_buf = (char*)malloc(picture_size);
	if (picture_buf == NULL) {
		RAVELOG_WARNA("START_AVI: Out of Memory\n");
        return false;
	}

	avpicture_fill((AVPicture*)yuv420p, (uint8_t*)picture_buf, PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);

    return true;
}

// always 24 bits
bool ADD_FRAME_FROM_DIB_TO_AVI(void* pdata)
{
    if( output == NULL )
        return false;
	int size;

    // flip vertically
    static vector<char> newdata;
    newdata.resize(stream->codec->height*stream->codec->width*3);
    char* penddata = (char*)pdata + stream->codec->height*stream->codec->width*3;

    for(int i = 0; i < stream->codec->height; ++i) {
        memcpy(&newdata[i*stream->codec->width*3], (char*)penddata - (i+1)*stream->codec->width*3, stream->codec->width*3);
    }
    
	picture->data[0] = (uint8_t*)&newdata[0];
	picture->linesize[0] = stream->codec->width * 3;

#ifdef HAVE_NEW_FFMPEG
    struct SwsContext *img_convert_ctx;
    img_convert_ctx = sws_getContext(stream->codec->width,
                                     stream->codec->height,
                                     PIX_FMT_BGR24,
                                     stream->codec->width,
                                     stream->codec->height,
                                     PIX_FMT_YUV420P,
                                     SWS_BICUBIC /* flags */,
                                     NULL, NULL, NULL);
    if (!sws_scale(img_convert_ctx, picture->data, picture->linesize, 0,
                   stream->codec->height, yuv420p->data, yuv420p->linesize)) {
        sws_freeContext(img_convert_ctx);
        RAVELOG_ERRORA("ADD_FRAME sws_scale failed\n");
        return false;
    }
 
    sws_freeContext(img_convert_ctx);
#else
    if( img_convert((AVPicture*)yuv420p, PIX_FMT_YUV420P, (AVPicture*)picture, PIX_FMT_BGR24, stream->codec->width, stream->codec->height) ) {
        RAVELOG_ERRORA("ADD_FRAME img_convert failed\n");
        return false;
    }
#endif
    
    size = avcodec_encode_video(stream->codec, (uint8_t*)outbuf, outbuf_size, yuv420p);
	if (size == -1) {
		RAVELOG_WARNA("error encoding frame\n");
        return false;
	}

    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = (uint8_t*)outbuf;
    pkt.size = size;
    pkt.stream_index = stream->index;
	if( av_write_frame(output, &pkt) < 0)
        RAVELOG_WARNA("av_write_frame failed\n");
    
    return true;
}

#else

#include <list>

// no ffmpeg
bool STOP_AVI() { return false; }
std::list<std::pair<int,string> > GET_CODECS() { return std::list<std::pair<int,string> >(); }
bool START_AVI(const char* filename, int _frameRate, int width, int height, int bits,int)
{
    RAVELOG_WARNA("avi recording to file %s not enabled\n", filename);
    return false;
}
bool ADD_FRAME_FROM_DIB_TO_AVI(void* pdata)
{
    return false;
}

#endif

#endif

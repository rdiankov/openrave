// -*- coding: utf-8 -*-
// Copyright (C) 2011 Rosen Diankov <rosen.diankov@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
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
#define __STDC_CONSTANT_MACROS

#include "plugindefs.h"

#ifdef _WIN32

#define AVIIF_KEYFRAME  0x00000010L // this frame is a key frame.

#include <string>
#include <list>

#include <windows.h>
#include <memory.h>
#include <mmsystem.h>
#include <vfw.h>

class VideoGlobalState
{
public:
    VideoGlobalState() {
        /* first let's make sure we are running on 1.1 */
        WORD wVer = HIWORD(VideoForWindowsVersion());
        if (wVer < 0x010a){
            throw openrave_exception("can't init avi library");
        }
        AVIFileInit();
    }
    virtual ~VideoGlobalState() {
        AVIFileExit();
    }
};

#else

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

class VideoGlobalState
{
public:
    VideoGlobalState()
    {
        av_register_all();
    }
    virtual ~VideoGlobalState() {}
};

#endif

static boost::shared_ptr<VideoGlobalState> s_pVideoGlobalState;

class ViewerRecorder : public ModuleBase
{
    boost::multi_array<uint32_t,2> _vwatermarkimage;
    int _nFrameCount, _nVideoWidth, _nVideoHeight;
    float _frameRate;
    uint64_t _starttime;
    bool _bSimTime;
    std::string _filename;
    boost::shared_ptr<void> _callback;
    
public:
    ViewerRecorder(EnvironmentBasePtr penv, std::istream& sinput) : ModuleBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nRecords the images produced from a viewer into video file. The recordings can be synchronized to real-time or simulation time, by default simulation time is used. Each instance can record only one file at a time. To record multiple files simultaneously, create multiple VideoRecorder instances";
        RegisterCommand("Start",boost::bind(&ViewerRecorder::_StartCommand,this,_1,_2),
                        "Starts recording a file, this will stop all previous recordings and overwrite any previous files stored in this location. Format::\n\n  Start [width] [height] [framerate] codec [codec] timing [simtime/realtime] filename [filename]\n\n");
        RegisterCommand("Stop",boost::bind(&ViewerRecorder::_StopCommand,this,_1,_2),
                        "Stops recording and saves the file. Format::\n\n  Stop\n\n");
        RegisterCommand("GetCodecs",boost::bind(&ViewerRecorder::_GetCodecsCommand,this,_1,_2),
                        "Return all the possible codecs, one codec per line:[video_codec id] [name]");
        RegisterCommand("SetWatermark",boost::bind(&ViewerRecorder::_SetWatermarkCommand,this,_1,_2),
                        "Set a WxHx4 image as a watermark. Each color is an unsigned integer ordered as A|B|G|R. The origin should be the top left corner");
        _nFrameCount = _nVideoWidth = _nVideoHeight = 0;
        _frameRate = 0;
        _bSimTime = true;
        _starttime = 0;
        _bWroteURL = false;
        _bWroteHeader = false;
#ifdef _WIN32
        _pfile = NULL; 
        _ps = NULL;
        _psCompressed = NULL; 
        _biSizeImage = 0;
#else
        _output = NULL;
        _stream = NULL;
        _picture = NULL;
        _yuv420p = NULL;
        _picture_buf = NULL; 
        _outbuf = NULL;
        _picture_size = 0;
        _outbuf_size = 0;
#endif
    }
    virtual ~ViewerRecorder()
    {
        _Reset();
    }

protected:
    bool _StartCommand(ostream& sout, istream& sinput)
    {
        if( !s_pVideoGlobalState ) {
            s_pVideoGlobalState.reset(new VideoGlobalState());
        }
        try {
            int codecid=-1;
            _frameRate = 30000.0/1001.0;
            _bSimTime = true;
            _filename = "";
            _Reset();
            sinput >> _nVideoWidth >> _nVideoHeight >> _frameRate;
            string cmd;
            while(!sinput.eof()) {
                sinput >> cmd;
                if( !sinput ) {
                    break;
                }
                std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
                if( cmd == "codec" ) {
                    sinput >> codecid;
                }
                else if( cmd == "filename" ) {
                    // raed the rest
                    if( !getline(sinput, _filename) ) {
                        return false;
                    }
                    boost::trim(_filename);
                    break;
                }
                else if( cmd == "timing" ) {
                    string type;
                    sinput >> type;
                    if( type == "simtime" ) {
                        _bSimTime = true;
                    }
                    else if( type == "realtime" ) {
                        _bSimTime = false;
                    }
                    else {
                        RAVELOG_WARN("unknown cmd");
                    }
                }
                else {
                    return false;
                }
                if( sinput.fail() || !sinput ) {
                    break;
                }
            }
            if( !sinput ) {
                RAVELOG_WARN("invalid format");
                return false;
            }

            _StartVideo(_filename,_frameRate,_nVideoWidth,_nVideoHeight,24,codecid);
            _starttime = _bSimTime ? GetEnv()->GetSimulationTime() : (GetNanoPerformanceTime()/1000);
            ViewerBasePtr pviewer;
            _callback = pviewer->RegisterViewerImageCallback(boost::bind(&ViewerRecorder::_ViewerImageCallback,this,_1,_2,_3));
            BOOST_ASSERT(!!_callback);
            return !!_callback;
        }
        catch(const openrave_exception& ex) {
            RAVELOG_WARN("%s\n",ex.what());
            _Reset();
        }
        return false;
    }
    bool _StopCommand(ostream& sout, istream& sinput)
    {
        _Reset();
        return true;
    }
    bool _GetCodecsCommand(ostream& sout, istream& sinput)
    {
        std::list<std::pair<int,string> > listcodecs;
        _GetCodecs(listcodecs);
        sout << listcodecs.size() << endl;
        FOREACH(it,listcodecs) {
            sout << it->first << " " << it->second << endl;
        }
        return true;
    }

    bool _SetWatermarkCommand(ostream& sout, istream& sinput)
    {
        int W, H;
        sinput >> W >> H;
        _vwatermarkimage.resize(boost::extents[H][W]);
        for(int i = 0; i < H; ++i) {
            for(int j = 0; j < W; ++j) {
                sinput >> _vwatermarkimage[i][j];
            }
        }
        return !!sinput;
        //_vwatermarkimage.resize(boost::extents[0][0]);
    }

    void _ViewerImageCallback(const uint8_t*, int width, int height)
    {
        //_AddWatermarkToImage(_ivOffscreen.getBuffer(), VIDEO_WIDTH, VIDEO_HEIGHT);
        //

//    uint64_t curtime = _bRealtimeVideo ? GetMicroTime() : GetEnv()->GetSimulationTime();
//    _nVideoTimeOffset += (curtime-_nLastVideoFrame);
//    
//    while(_nVideoTimeOffset >= (1000000.0/VIDEO_FRAMERATE) ) {
//        if( !ADD_FRAME_FROM_DIB_TO_AVI(_ivOffscreen.getBuffer()) ) {
//            RAVELOG_WARN("Failed adding frames, stopping avi recording\n");
//            _bSaveVideo = false;
//            return true;
//        }
//        
//        _nVideoTimeOffset -= (1000000.0/VIDEO_FRAMERATE);
//    }  
//    _nLastVideoFrame = curtime;

    }

    void _AddWatermarkToImage(unsigned char* image, int width, int height)
    {
        if( _vwatermarkimage.size() == 0 ) {
            return;
        }
        // the origin of the image is the bottom left corner
        for(int i = 0; i < height; ++i) {
            for(int j = 0; j < width; ++j) {
                int iwater = _vwatermarkimage.shape()[0]-(i%_vwatermarkimage.shape()[0])-1;
                int jwater = j%_vwatermarkimage.shape()[1];
                uint32_t color = _vwatermarkimage[iwater][jwater];
                uint32_t A = color>>24;
                for(int k = 0; k < 3; ++k) {
                    image[k] = ((uint32_t)image[k]*(255-A)+((color>>(8*k))&0xff)*A)>>8;
                }
                image += 3;
            }
        }
    }

#ifdef _WIN32
    PAVIFILE _pfile;
    PAVISTREAM _ps;
    PAVISTREAM _psCompressed; 
    PAVISTREAM _psText;
    int _biSizeImage;

    void _GetCodecs(std::list<std::pair<int,std::string> >& lcodecs) { lcodecs.clear(); }
.. 
    void _StartVideo(const std::string& filename, double frameRate, int width, int height, int bits, int codecid=-1)
    {
        _nFrameCount = 0;
        HRESULT hr = AVIFileOpenA(_pfile, filename.c_str(), OF_WRITE | OF_CREATE, NULL);
        BOOST_ASSERT( hr == AVIERR_OK );
        _CreateStream(pfile, &ps, (int)frameRate, width*height/bits, width, height, "none");

        BITMAPINFOHEADER bi;
        memset(&bi, 0, sizeof(bi));
        bi.biSize = sizeof(BITMAPINFOHEADER);
        bi.biWidth = width;
        bi.biHeight = height;
        bi.biPlanes = 1;
        bi.biBitCount = bits;
        bi.biCompression = BI_RGB;
        bi.biSizeImage = width * height * bits /8;
        _SetOptions(&bi, "none");
        _biSizeImage = bi.biSizeImage;
    }

    void _Reset()
    {
        if (!!_ps) {
            AVIStreamClose(_ps);
            _ps = NULL;
        }
        if (!!_psCompressed) {
            AVIStreamClose(_psCompressed);
            _psCompressed = NULL;
        }
        if (!!_psText) {
            AVIStreamClose(_psText);
            _psText = NULL;
        }
        if( _pfile) {
            AVIFileClose(_pfile);
            _pfile = NULL;
        }
    }

    DWORD _getFOURCC(const char* value)
    {
        if(stricmp(value, "DIB") == 0) {
            return mmioFOURCC(value[0],value[1],value[2],' ');
        }
        else if((stricmp(value, "CVID") == 0) || (stricmp(value, "IV32") == 0) || (stricmp(value, "MSVC") == 0) || (stricmp(value, "IV50") == 0)) {
            return mmioFOURCC(value[0],value[1],value[2],value[3]);
        }
        else {
            return NULL;
        }
    }

    // Fill in the header for the video stream....
    // The video stream will run in rate ths of a second....
    void _CreateStream(int rate, unsigned long buffersize, int rectwidth, int rectheight, const char* _compressor)
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
        SetRect(&strhdr.rcFrame, 0, 0, (int) rectwidth, (int) rectheight);
        //strhdr.dwEditCount            = 
        //strhdr.dwFormatChangeCount    =
        //strcpy(strhdr.szName, "Full Frames (Uncompressed)");

        // And create the stream;
        HRESULT hr = AVIFileCreateStream(pfile, _ps, &strhdr);
        BOOST_ASSERT(hr == AVIERR_OK);
    }

    void _SetOptions(LPBITMAPINFOHEADER lpbi, const char* _compressor)
    {
        AVICOMPRESSOPTIONS opts;
        AVICOMPRESSOPTIONS FAR * aopts[1] = {&opts};

        memset(&opts, 0, sizeof(opts));
        opts.fccType = streamtypeVIDEO;
        opts.fccHandler = getFOURCC(_compressor);

        /* display the compression options dialog box if specified compressor is unknown */
        if(getFOURCC(_compressor) == NULL) {
            if (!AVISaveOptions(NULL, 0, 1, _ps, (LPAVICOMPRESSOPTIONS FAR *) &aopts)) {
                throw OPENRAVE_EXCEPTION_FORMAT0("failed to set options",ORE_Assert);
            }
        }

        HRESULT hr = AVIMakeCompressedStream(_psCompressed, *_ps, &opts, NULL);
        BOOST_ASSERT(hr == AVIERR_OK);
    
        hr = AVIStreamSetFormat(*_psCompressed, 0, lpbi/*stream format*/, lpbi->biSize + lpbi->biClrUsed * sizeof(RGBQUAD)/*format size*/);
        BOOST_ASSERT(hr == AVIERR_OK);
    }

    void _SetText(char *szText, int width, int height, int TextHeight)
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
        SetRect(&strhdr.rcFrame, 0, (int) height, (int) width, (int) height + TextHeight); // #define TEXT_HEIGHT 20

        // ....and create the stream.
        HRESULT hr = AVIFileCreateStream(_pfile, &_psText, &strhdr);
        BOOST_ASSERT(hr == AVIERR_OK);

        dwTextFormat = sizeof(dwTextFormat);
        hr = AVIStreamSetFormat(_psText, 0, &dwTextFormat, sizeof(dwTextFormat));
        BOOST_ASSERT(hr == AVIERR_OK);
    }

    void _AddFrame(void* pdata)
    {
        HRESULT hr = AVIStreamWrite(psCompressed/*stream pointer*/, _nFrameCount/*time of this frame*/, 1/*number to write*/, pdata, _biSizeImage/*size of this frame*/, AVIIF_KEYFRAME/*flags....*/, NULL, NULL);
        BOOST_ASSERT(hr == AVIERR_OK);
        _nFrameCount++;
    }

    void _AddText(int time, char *szText)
    {
        HRESULT hr = AVIStreamWrite(_psText, time,  1, szText, strlen(szText) + 1, AVIIF_KEYFRAME, NULL, NULL);
        BOOST_ASSERT(hr == AVIERR_OK);
        return hr == AVIERR_OK;
    }

#else
    // ffmpeg

    AVFormatContext *_output;
    AVStream *_stream;
    AVFrame *_picture;
    AVFrame *_yuv420p;
    char *_picture_buf, *_outbuf;
    int _picture_size;
    int _outbuf_size;
    bool _bWroteURL, _bWroteHeader;

    void _Reset()
    {
        RAVELOG_DEBUG("stopping avi\n");
        free(_picture_buf); _picture_buf = NULL;
        free(_picture); _picture = NULL;
        free(_yuv420p); _yuv420p = NULL;
        free(_outbuf); _outbuf = NULL;
        if( !!_stream ) {
            avcodec_close(_stream->codec);
            _stream = NULL;
        }

        if( !!_output ) {
            if( _bWroteHeader ) {
                av_write_trailer(_output);
            }
            if( _bWroteURL ) {
#if LIBAVFORMAT_VERSION_INT >= (52<<16)
                url_fclose(_output->pb);
#else
                url_fclose(&_output->pb);
#endif
            }
            if( _output->streams[0] ) {
                av_freep(&_output->streams[0]);
            }
            av_free(_output);
            _output = NULL;
        }
        _bWroteURL = _bWroteHeader = false;
    }

    void _GetCodecs(std::list<std::pair<int,string> >& lcodecs)
    {
        if( !s_pVideoGlobalState ) {
            s_pVideoGlobalState.reset(new VideoGlobalState());
        }
        lcodecs.clear();
        AVOutputFormat *fmt = first_oformat;
        while (fmt != NULL) {
            lcodecs.push_back(make_pair((int)fmt->video_codec,fmt->long_name));
            fmt = fmt->next;
        }
    }

    void _StartVideo(const std::string& filename, double frameRate, int width, int height, int bits, int codecid=-1)
    {
        if( bits != 24 ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("START_AVI only supports 24bits",ORE_InvalidArguments);
        }
    
        AVCodecContext *codec_ctx;
        AVCodec *codec;

        CodecID video_codec = codecid == -1 ? CODEC_ID_MPEG4 : (CodecID)codecid;
        AVOutputFormat *fmt = first_oformat;
        while (fmt != NULL) {
            if (fmt->video_codec == video_codec) {
                break;
            }
            fmt = fmt->next;
        }
        BOOST_ASSERT(!!fmt);

        _output = (AVFormatContext*)av_mallocz(sizeof(AVFormatContext));
        BOOST_ASSERT(!!_output);

        _output->oformat = fmt;
        snprintf(_output->filename, sizeof(_output->filename), "%s", filename.c_str());

        _stream = av_new_stream(_output, 0);
        BOOST_ASSERT(!!_stream);

        codec_ctx = _stream->codec;
        codec_ctx->codec_id = video_codec;
        codec_ctx->codec_type = CODEC_TYPE_VIDEO;
        codec_ctx->bit_rate = 4000000;
        codec_ctx->width = width;
        codec_ctx->height = height;
        if( RaveFabs(frameRate-29.97)<0.01 ) {
            codec_ctx->time_base= (AVRational){1001,30000};
        }
        else {
            codec_ctx->time_base= (AVRational){1,(int)frameRate};
        }
        codec_ctx->gop_size = 10;
        codec_ctx->max_b_frames = 1;
        codec_ctx->pix_fmt = PIX_FMT_YUV420P;

        if (av_set_parameters(_output, NULL) < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0("set parameters failed",ORE_Assert);
        }

        codec = avcodec_find_encoder(codec_ctx->codec_id);
        BOOST_ASSERT(!!codec);

        RAVELOG_DEBUG(str(boost::format("opening %s, w:%d h:%dx fps:%f, codec: %s")%_output->filename%width%height%frameRate%codec->name));
        if (avcodec_open(codec_ctx, codec) < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0("Unable to open codec",ORE_Assert);
        }

        int ret = url_fopen(&_output->pb, filename.c_str(), URL_WRONLY);
        if (ret < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT("_StartVideo: Unable to open %s for writing: %d\n", filename%ret,ORE_Assert);
        }
        _bWroteURL = true;

        av_write_header(_output);
        _bWroteHeader = true;

        _picture = avcodec_alloc_frame();
        _yuv420p = avcodec_alloc_frame();

        _outbuf_size = 500000;
        _outbuf = (char*)malloc(_outbuf_size);
        BOOST_ASSERT(!!_outbuf);

        _picture_size = avpicture_get_size(PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);
        _picture_buf = (char*)malloc(_picture_size);
        BOOST_ASSERT(!!_picture_buf);

        avpicture_fill((AVPicture*)_yuv420p, (uint8_t*)_picture_buf, PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);
    }

    void _AddFrame(void* pdata)
    {
        BOOST_ASSERT( !!_output );
        int size;

        // flip vertically
        static vector<char> newdata;
        newdata.resize(_stream->codec->height*_stream->codec->width*3);
        char* penddata = (char*)pdata + _stream->codec->height*_stream->codec->width*3;

        for(int i = 0; i < _stream->codec->height; ++i) {
            memcpy(&newdata[i*_stream->codec->width*3], (char*)penddata - (i+1)*_stream->codec->width*3, _stream->codec->width*3);
        }
    
        _picture->data[0] = (uint8_t*)&newdata[0];
        _picture->linesize[0] = _stream->codec->width * 3;

#ifdef HAVE_NEW_FFMPEG
        struct SwsContext *img_convert_ctx;
        img_convert_ctx = sws_getContext(_stream->codec->width, _stream->codec->height, PIX_FMT_BGR24, _stream->codec->width, _stream->codec->height, PIX_FMT_YUV420P, SWS_BICUBIC /* flags */, NULL, NULL, NULL);
        if (!sws_scale(img_convert_ctx, _picture->data, _picture->linesize, 0, _stream->codec->height, _yuv420p->data, _yuv420p->linesize)) {
            sws_freeContext(img_convert_ctx);
            throw OPENRAVE_EXCEPTION_FORMAT0("ADD_FRAME sws_scale failed",ORE_Assert);
        }
 
        sws_freeContext(img_convert_ctx);
#else
        if( img_convert((AVPicture*)_yuv420p, PIX_FMT_YUV420P, (AVPicture*)_picture, PIX_FMT_BGR24, _stream->codec->width, _stream->codec->height) ) {
            throw OPENRAVE_EXCEPTION_FORMAT0("ADD_FRAME img_convert failed",ORE_Assert);
        }
#endif
    
        size = avcodec_encode_video(_stream->codec, (uint8_t*)_outbuf, _outbuf_size, _yuv420p);
        if (size == -1) {
            throw OPENRAVE_EXCEPTION_FORMAT0("error encoding frame",ORE_Assert);
        }

        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = (uint8_t*)_outbuf;
        pkt.size = size;
        pkt.stream_index = _stream->index;
        if( av_write_frame(_output, &pkt) < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0("av_write_frame failed",ORE_Assert);
        }
        _nFrameCount++;
    }
#endif
};

ModuleBasePtr CreateViewerRecorder(EnvironmentBasePtr penv, std::istream& sinput) { return ModuleBasePtr(new ViewerRecorder(penv,sinput)); }
void DestroyViewerRecordingStaticResources()
{
#ifdef _WIN32
    s_pVideoGlobalState.clear();
#endif
}

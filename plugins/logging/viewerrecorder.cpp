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

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/version.hpp>

#include <boost/lexical_cast.hpp>

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
        if (wVer < 0x010a) {
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
    virtual ~VideoGlobalState() {
    }
};

#endif

static boost::shared_ptr<VideoGlobalState> s_pVideoGlobalState;

class ViewerRecorder : public ModuleBase
{
    inline boost::shared_ptr<ViewerRecorder> shared_module() {
        return boost::dynamic_pointer_cast<ViewerRecorder>(shared_from_this());
    }
    inline boost::shared_ptr<ViewerRecorder const> shared_module_const() const {
        return boost::dynamic_pointer_cast<ViewerRecorder const>(shared_from_this());
    }
    struct VideoFrame
    {
        VideoFrame() : _timestamp(0), _bProcessed(false) {
        }
        vector<uint8_t> _vimagememory;
        int _width, _height, _pixeldepth;
        uint64_t _timestamp;
        bool _bProcessed;
    };

    boost::mutex _mutex; // for video data passing
    boost::mutex _mutexlibrary; // for video encoding library resources
    boost::condition _condnewframe;
    bool _bContinueThread, _bStopRecord;
    boost::shared_ptr<boost::thread> _threadrecord;

    boost::multi_array<uint32_t,2> _vwatermarkimage;
    int _nFrameCount, _nVideoWidth, _nVideoHeight;
    float _framerate;
    uint64_t _frameindex;
    uint64_t _starttime, _frametime;
    std::string _filename;
    UserDataPtr _callback;
    int _nUseSimulationTime; // 0 to record as is, 1 to record with respect to simulation, 2 to control simulation to viewer updates
    dReal _fSimulationTimeMultiplier; // how many times to make the simulation time faster
    list<boost::shared_ptr<VideoFrame> > _listAddFrames, _listFinishedFrames;
    boost::shared_ptr<VideoFrame> _frameLastAdded;

public:
    ViewerRecorder(EnvironmentBasePtr penv, std::istream& sinput) : ModuleBase(penv)
    {
        __description = ":Interface Author: Rosen Diankov\n\nRecords the images produced from a viewer into video file. The recordings can be synchronized to real-time or simulation time, by default simulation time is used. Each instance can record only one file at a time. To record multiple files simultaneously, create multiple VideoRecorder instances";
        RegisterCommand("Start",boost::bind(&ViewerRecorder::_StartCommand,this,_1,_2),
                        "Starts recording a file, this will stop all previous recordings and overwrite any previous files stored in this location. Format::\n\n  Start [width] [height] [framerate] codec [codec] timing [simtime/realtime/controlsimtime[=timestepmult]] viewer [name]\\n filename [filename]\\n\n\nBecause the viewer and filenames can have spaces, the names are ready until a newline is encountered");
        RegisterCommand("Stop",boost::bind(&ViewerRecorder::_StopCommand,this,_1,_2),
                        "Stops recording and saves the file. Format::\n\n  Stop\n\n");
        RegisterCommand("GetCodecs",boost::bind(&ViewerRecorder::_GetCodecsCommand,this,_1,_2),
                        "Return all the possible codecs, one codec per line:[video_codec id] [name]");
        RegisterCommand("SetWatermark",boost::bind(&ViewerRecorder::_SetWatermarkCommand,this,_1,_2),
                        "Set a WxHx4 image as a watermark. Each color is an unsigned integer ordered as A|B|G|R. The origin should be the top left corner");
        _nFrameCount = _nVideoWidth = _nVideoHeight = 0;
        _framerate = 0;
        _nUseSimulationTime = 1;
        _fSimulationTimeMultiplier = 1;
        _starttime = 0;
        _bContinueThread = true;
        _bStopRecord = true;
        _frameindex = 0;
#ifdef _WIN32
        _pfile = NULL;
        _ps = NULL;
        _psCompressed = NULL;
        _psText = NULL;
        _biSizeImage = 0;
#else
        _bWroteURL = false;
        _bWroteHeader = false;
        _output = NULL;
        _stream = NULL;
        _picture = NULL;
        _yuv420p = NULL;
        _picture_buf = NULL;
        _outbuf = NULL;
        _picture_size = 0;
        _outbuf_size = 0;
#endif
        _threadrecord.reset(new boost::thread(boost::bind(&ViewerRecorder::_RecordThread,this)));
    }
    virtual ~ViewerRecorder()
    {
        RAVELOG_VERBOSE("~ViewerRecorder\n");
        _bContinueThread = false;
        _Reset();
        {
            boost::mutex::scoped_lock lock(_mutex);
            _condnewframe.notify_all();
        }
        _threadrecord->join();
    }

    virtual void Destroy() {
        _Reset();
    }

protected:
    bool _StartCommand(ostream& sout, istream& sinput)
    {
        if( !s_pVideoGlobalState ) {
            s_pVideoGlobalState.reset(new VideoGlobalState());
        }
        try {
            ViewerBasePtr pviewer;
            int codecid=-1;
            _Reset();
            sinput >> _nVideoWidth >> _nVideoHeight >> _framerate;
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
                    if( !getline(sinput, _filename) ) {
                        return false;
                    }
                    boost::trim(_filename);
                }
                else if( cmd == "timing" ) {
                    string type;
                    sinput >> type;
                    if( type == "simtime" ) {
                        _nUseSimulationTime = 1;
                    }
                    else if( type.size() >= 14 && type.substr(0,14) == "controlsimtime" ) {
                        if( type.size() > 14 && type[14] == '=' ) {
                            _fSimulationTimeMultiplier = boost::lexical_cast<dReal>(type.substr(15));
                        }
                        RAVELOG_DEBUG(str(boost::format("controlling sim time when recording, timestep=%f")%_fSimulationTimeMultiplier));
                        _nUseSimulationTime = 2;
                    }
                    else if( type == "realtime" ) {
                        _nUseSimulationTime = 0;
                    }
                    else {
                        RAVELOG_WARN("unknown cmd");
                    }
                }
                else if( cmd == "viewer" ) {
                    string name;
                    if( !getline(sinput, name) ) {
                        return false;
                    }
                    boost::trim(name);
                    pviewer = GetEnv()->GetViewer(name);
                }
                else {
                    return false;
                }
                if( sinput.fail() || !sinput ) {
                    break;
                }
            }
            boost::mutex::scoped_lock lock(_mutex);
            if( !pviewer ) {
                RAVELOG_WARN("invalid viewer\n");
            }
            else{
                stringstream ss;
                ss << "Resize " << _nVideoWidth << " " << _nVideoHeight << " ";
                pviewer->SendCommand(sout,ss);
            }
            RAVELOG_INFO("video filename: %s, %d x %d @ %f frames/sec\n",_filename.c_str(),_nVideoWidth,_nVideoHeight,_framerate);
            _StartVideo(_filename,_framerate,_nVideoWidth,_nVideoHeight,24,codecid);
            _starttime = 0;
            if( _nUseSimulationTime == 2 ) {
                _frametime = (uint64_t)(1000000.0f*_fSimulationTimeMultiplier/_framerate);
            }
            else {
                _frametime = (uint64_t)(1000000.0f/_framerate);
            }
            _callback = pviewer->RegisterViewerImageCallback(boost::bind(&ViewerRecorder::_ViewerImageCallback,shared_module(),_1,_2,_3,_4));
            BOOST_ASSERT(!!_callback);
            _bStopRecord = false;
            return !!_callback;
        }
        catch(const std::exception& ex) {
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

    bool _SetWatermarkCommand(ostream& sout, istream& sinput)
    {
        boost::mutex::scoped_lock lock(_mutex);
        int W, H;
        sinput >> W >> H;
        _vwatermarkimage.resize(boost::extents[H][W]);
        for(int i = 0; i < H; ++i) {
            for(int j = 0; j < W; ++j) {
                sinput >> _vwatermarkimage[i][j];
            }
        }
        return !!sinput;
    }

    void _ViewerImageCallback(const uint8_t* memory, int width, int height, int pixeldepth)
    {
        boost::mutex::scoped_lock lock(_mutex);
        if( !GetEnv() || !_callback ) {
            // recorder already destroyed and this thread is just remaining
            return;
        }
        uint64_t timestamp = _nUseSimulationTime ? GetEnv()->GetSimulationTime() : utils::GetMicroTime();
        boost::shared_ptr<VideoFrame> frame;

        if( _listAddFrames.size() > 0 ) {
            BOOST_ASSERT( timestamp-_starttime >= _listAddFrames.back()->_timestamp-_starttime );
            if( _listAddFrames.back()->_timestamp == timestamp ) {
                // if the timestamps match, then take the newest frame
                frame = _listAddFrames.back();
                _listAddFrames.pop_back();
            }
        }
        if( !frame ) {
            if( _listFinishedFrames.size() > 0 ) {
                frame = _listFinishedFrames.back();
                _listFinishedFrames.pop_back();
            }
            else {
                frame.reset(new VideoFrame());
            }
        }
        frame->_width = width;
        frame->_height = height;
        frame->_pixeldepth = pixeldepth;
        //RAVELOG_VERBOSE("image frame is %d x %d\n",width,height);
        frame->_timestamp = timestamp;
        frame->_vimagememory.resize(width*height*pixeldepth);
        std::copy(memory,memory+width*height*pixeldepth,frame->_vimagememory.begin());
        _listAddFrames.push_back(frame);
        if( _starttime == 0 ) {
            _starttime = timestamp;
        }
        RAVELOG_VERBOSE(str(boost::format("new frame %d\n")%(timestamp-_starttime)));
        _condnewframe.notify_one();
        if( _nUseSimulationTime == 2 ) {
            // calls the environment lock, which might be taken if the environment is destroying the problem
            // therefore need to take it first
            while(_bContinueThread && !_bStopRecord) {
                boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv = _LockEnvironment(100000);
                if( !!lockenv ) {
                    GetEnv()->StepSimulation(_fSimulationTimeMultiplier/_framerate);
                    break;
                }
            }
        }
    }

    boost::shared_ptr<EnvironmentMutex::scoped_try_lock> _LockEnvironment(uint64_t timeout)
    {
        // try to acquire the lock
#if BOOST_VERSION >= 103500
        boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),boost::defer_lock_t()));
#else
        boost::shared_ptr<EnvironmentMutex::scoped_try_lock> lockenv(new EnvironmentMutex::scoped_try_lock(GetEnv()->GetMutex(),false));
#endif
        uint64_t basetime = utils::GetMicroTime();
        while(utils::GetMicroTime()-basetime<timeout ) {
            lockenv->try_lock();
            if( !!*lockenv ) {
                break;
            }
        }
        if( !*lockenv ) {
            lockenv.reset();
        }
        return lockenv;
    }

    void _RecordThread()
    {
        while(_bContinueThread) {
            boost::shared_ptr<VideoFrame> frame;
            uint64_t numstores=0;
            {
                boost::mutex::scoped_lock lock(_mutex);
                if( !_bContinueThread ) {
                    return;
                }
                if( _listAddFrames.size() == 0 ) {
                    _condnewframe.wait(lock);
                    if( _listAddFrames.size() == 0 ) {
                        continue;
                    }
                }
                if( _bStopRecord ) {
                    continue;
                }

                if(( _listAddFrames.front()->_timestamp-_starttime > _frametime) && !!_frameLastAdded ) {
                    frame = _frameLastAdded;
                    numstores = (_listAddFrames.front()->_timestamp-_starttime-1)/_frametime;
                    RAVELOG_VERBOSE(str(boost::format("previous frame repeated %d times\n")%numstores));
                }
                else {
                    uint64_t lastoffset = _listAddFrames.back()->_timestamp - _starttime;
                    if( lastoffset < _frametime ) {
                        // not enough frames to predict what's coming next so wait
                        continue;
                    }
                    list<boost::shared_ptr<VideoFrame> >::iterator itframe = _listAddFrames.begin(), itbest = _listAddFrames.end();
                    uint64_t bestdist = 0;
                    while(itframe != _listAddFrames.end()) {
                        uint64_t offset = (*itframe)->_timestamp - _starttime;
                        uint64_t dist = offset >= _frametime ? (offset-_frametime) : (_frametime-offset);
                        if(( itbest == _listAddFrames.end()) ||( bestdist > dist) ) {
                            itbest = itframe;
                            bestdist = dist;
                        }
                        if( !_bContinueThread ) {
                            return;
                        }
                        ++itframe;
                    }
                    frame = *itbest;
                    size_t prevsize = _listAddFrames.size();
                    _listAddFrames.erase(_listAddFrames.begin(),itbest);
                    if( frame->_timestamp-_starttime <= _frametime ) {
                        // the frame is before the next mark, so erase it
                        _listAddFrames.erase(itbest);
                    }
                    RAVELOG_VERBOSE(str(boost::format("frame size: %d -> %d\n")%prevsize%_listAddFrames.size()));
                    numstores = 1;
                }
            }

            if( !frame->_bProcessed ) {
                _AddWatermarkToImage(&frame->_vimagememory.at(0), frame->_width, frame->_height, frame->_pixeldepth);
                frame->_bProcessed = true;
            }

            try {
                _starttime += _frametime*numstores;
                for(uint64_t i = 0; i < numstores; ++i) {
                    _AddFrame(&frame->_vimagememory.at(0));
                }
                _frameLastAdded = frame;
            }
            catch(const std::exception& ex) {
                RAVELOG_WARN("%s\n",ex.what());
            }
        }
    }

    void _AddWatermarkToImage(uint8_t* memory, int width, int height, int pixeldepth)
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
                for(int k = 0; k < min(pixeldepth,3); ++k) {
                    memory[k] = ((uint32_t)memory[k]*(255-A)+((color>>(8*k))&0xff)*A)>>8;
                }
                memory += pixeldepth;
            }
        }
    }

    void _Reset()
    {
        {
            RAVELOG_DEBUG("ViewerRecorder _Reset\n");
            _bStopRecord = true;
            boost::mutex::scoped_lock lock(_mutex);
            _nFrameCount = 0;
            _nVideoWidth = _nVideoHeight = 0;
            _framerate = 30000.0f/1001.0f;
            _starttime = 0;
            _callback.reset();
            _nUseSimulationTime = true;
            _listAddFrames.clear();
            _listFinishedFrames.clear();
            _frameLastAdded.reset();
            _filename = "";
        }
        {
            RAVELOG_DEBUG("ViewerRecorder _ResetLibrary\n");
            boost::mutex::scoped_lock lock(_mutexlibrary);
            _ResetLibrary();
        }
    }

#ifdef _WIN32
    PAVIFILE _pfile;
    PAVISTREAM _ps, _psCompressed, _psText;
    int _biSizeImage;

    bool _GetCodecsCommand(ostream& sout, istream& sinput)
    {
        return false;
    }

    void _StartVideo(const std::string& filename, double frameRate, int width, int height, int bits, int codecid=-1)
    {
        _nFrameCount = 0;
        HRESULT hr = AVIFileOpenA(&_pfile, filename.c_str(), OF_WRITE | OF_CREATE, NULL);
        BOOST_ASSERT( hr == AVIERR_OK );
        _CreateStream((int)_framerate, width*height/bits, width, height, "none");

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

    void _ResetLibrary()
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
        strhdr.fccType                = streamtypeVIDEO;    // stream type
        strhdr.fccHandler             = _getFOURCC(_compressor);
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
        strhdr.dwRate                 = rate;                   // rate fps
        //strhdr.dwStart                =
        //strhdr.dwLength               =
        //strhdr.dwInitialFrames        =
        strhdr.dwSuggestedBufferSize  = buffersize;
        strhdr.dwQuality              = -1;     // use the default
        //strhdr.dwSampleSize           =
        SetRect(&strhdr.rcFrame, 0, 0, (int) rectwidth, (int) rectheight);
        //strhdr.dwEditCount            =
        //strhdr.dwFormatChangeCount    =
        //strcpy(strhdr.szName, "Full Frames (Uncompressed)");

        // And create the stream;
        HRESULT hr = AVIFileCreateStream(_pfile, &_ps, &strhdr);
        BOOST_ASSERT(hr == AVIERR_OK);
    }

    void _SetOptions(LPBITMAPINFOHEADER lpbi, const char* _compressor)
    {
        AVICOMPRESSOPTIONS opts;
        AVICOMPRESSOPTIONS FAR * aopts[1] = { &opts};

        memset(&opts, 0, sizeof(opts));
        opts.fccType = streamtypeVIDEO;
        opts.fccHandler = _getFOURCC(_compressor);

        /* display the compression options dialog box if specified compressor is unknown */
        if(_getFOURCC(_compressor) == NULL) {
            if (!AVISaveOptions(NULL, 0, 1, &_ps, (LPAVICOMPRESSOPTIONS FAR *) &aopts)) {
                throw OPENRAVE_EXCEPTION_FORMAT0("failed to set options",ORE_Assert);
            }
        }

        HRESULT hr = AVIMakeCompressedStream(&_psCompressed, _ps, &opts, NULL);
        BOOST_ASSERT(hr == AVIERR_OK);

        hr = AVIStreamSetFormat(_psCompressed, 0, lpbi /*stream format*/, lpbi->biSize + lpbi->biClrUsed * sizeof(RGBQUAD) /*format size*/);
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
        SetRect(&strhdr.rcFrame, 0, (int) height, (int) width, (int) height + TextHeight);     // #define TEXT_HEIGHT 20

        // ....and create the stream.
        HRESULT hr = AVIFileCreateStream(_pfile, &_psText, &strhdr);
        BOOST_ASSERT(hr == AVIERR_OK);

        dwTextFormat = sizeof(dwTextFormat);
        hr = AVIStreamSetFormat(_psText, 0, &dwTextFormat, sizeof(dwTextFormat));
        BOOST_ASSERT(hr == AVIERR_OK);
    }

    void _AddFrame(void* pdata)
    {
        boost::mutex::scoped_lock lock(_mutexlibrary);
        HRESULT hr = AVIStreamWrite(_psCompressed /*stream pointer*/, _nFrameCount /*time of this frame*/, 1 /*number to write*/, pdata, _biSizeImage /*size of this frame*/, AVIIF_KEYFRAME /*flags....*/, NULL, NULL);
        BOOST_ASSERT(hr == AVIERR_OK);
        _nFrameCount++;
    }

    void _AddText(int time, char *szText)
    {
        HRESULT hr = AVIStreamWrite(_psText, time,  1, szText, strlen(szText) + 1, AVIIF_KEYFRAME, NULL, NULL);
        BOOST_ASSERT(hr == AVIERR_OK);
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

    void _ResetLibrary()
    {
        free(_picture_buf); _picture_buf = NULL;
        free(_picture); _picture = NULL;
        free(_yuv420p); _yuv420p = NULL;
        free(_outbuf); _outbuf = NULL;
        if( !!_stream ) {
            avcodec_close(_stream->codec);
            _stream = NULL;
        }

        if( !!_output ) {
            RAVELOG_DEBUG("stopping avi\n");
            if( _bWroteHeader ) {
                av_write_trailer(_output);
            }
            if( _bWroteURL ) {
#if LIBAVFORMAT_VERSION_INT >= (54<<16)
                avio_close(_output->pb);
#elif LIBAVFORMAT_VERSION_INT >= (52<<16)
                url_fclose(_output->pb);
#else
                url_fclose(&_output->pb);
#endif
            }
            avformat_free_context(_output);
            _output = NULL;
        }
        _bWroteURL = _bWroteHeader = false;
    }

    bool _GetCodecsCommand(ostream& sout, istream& sinput)
    {
        if( !s_pVideoGlobalState ) {
            s_pVideoGlobalState.reset(new VideoGlobalState());
        }
#if LIBAVFORMAT_VERSION_INT >= (52<<16)
        AVOutputFormat *fmt = av_oformat_next(NULL); //first_oformat;
#else
        AVOutputFormat *fmt = first_oformat;
#endif
        while (fmt != NULL) {
#if defined(LIBAVCODEC_VERSION_INT) && LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(54,25,0) // introduced at http://git.libav.org/?p=libav.git;a=commit;h=104e10fb426f903ba9157fdbfe30292d0e4c3d72
            if( fmt->video_codec != AV_CODEC_ID_NONE && !!fmt->name ) {
#else
            if( fmt->video_codec != CODEC_ID_NONE && !!fmt->name ) {
#endif            
                string mime_type;
                if( !!fmt->mime_type ) {
                    mime_type = fmt->mime_type;
                }
                if( mime_type.size() == 0 ) {
                    mime_type = "*";
                }
                sout << fmt->video_codec << " " << mime_type << " " << fmt->name << endl;
            }
            fmt = fmt->next;
        }
        return true;
    }

    void _StartVideo(const std::string& filename, double frameRate, int width, int height, int bits, int codecid=-1)
    {
        OPENRAVE_ASSERT_OP_FORMAT0(bits,==,24,"START_AVI only supports 24bits",ORE_InvalidArguments);
        OPENRAVE_ASSERT_OP_FORMAT0(filename.size(),>,0,"filename needs to be valid",ORE_InvalidArguments);
        AVCodecContext *codec_ctx;
        AVCodec *codec;

#if defined(LIBAVCODEC_VERSION_INT) && LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(54,25,0) // introduced at http://git.libav.org/?p=libav.git;a=commit;h=104e10fb426f903ba9157fdbfe30292d0e4c3d72
        AVCodecID video_codec = codecid == -1 ? AV_CODEC_ID_MPEG4 : (AVCodecID)codecid;
#else
        CodecID video_codec = codecid == -1 ? CODEC_ID_MPEG4 : (CodecID)codecid;
#endif
#if LIBAVFORMAT_VERSION_INT >= (52<<16)
        AVOutputFormat *fmt = av_oformat_next(NULL); //first_oformat;
#else
        AVOutputFormat *fmt = first_oformat;
#endif
        while (fmt != NULL) {
            if (fmt->video_codec == video_codec) {
                break;
            }
            fmt = fmt->next;
        }
        BOOST_ASSERT(!!fmt);

        _frameindex = 0;
        _output = avformat_alloc_context();
        BOOST_ASSERT(!!_output);

        _output->oformat = fmt;
        snprintf(_output->filename, sizeof(_output->filename), "%s", filename.c_str());

#if LIBAVFORMAT_VERSION_INT >= (54<<16)
        _stream = avformat_new_stream(_output, 0);
#else
        _stream = av_new_stream(_output, 0);
#endif
        BOOST_ASSERT(!!_stream);

        codec_ctx = _stream->codec;
        codec_ctx->codec_id = video_codec;
#ifdef HAS_AVMEDIA_TYPE_VIDEO
        codec_ctx->codec_type = AVMEDIA_TYPE_VIDEO;
#else
        codec_ctx->codec_type = CODEC_TYPE_VIDEO;
#endif
        codec_ctx->bit_rate = 4000000;
        codec_ctx->width = width;
        codec_ctx->height = height;
        if( RaveFabs(frameRate-29.97)<0.01 ) {
            codec_ctx->time_base.num = 1001;
            codec_ctx->time_base.den = 30000;
        }
        else {
            codec_ctx->time_base.num = 1;
            codec_ctx->time_base.den = (int)frameRate;
        }
        codec_ctx->gop_size = 10;
        codec_ctx->max_b_frames = 1;
#if LIBAVFORMAT_VERSION_INT >= (55<<16)
        codec_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
#else
        codec_ctx->pix_fmt = PIX_FMT_YUV420P;
#endif

#if LIBAVFORMAT_VERSION_INT >= (54<<16)
        // not necessary to set parameters?
#else
        if (av_set_parameters(_output, NULL) < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0("set parameters failed",ORE_Assert);
        }
#endif
        codec = avcodec_find_encoder(codec_ctx->codec_id);
        BOOST_ASSERT(!!codec);

        RAVELOG_DEBUG(str(boost::format("opening %s, w:%d h:%dx fps:%f, codec: %s")%_output->filename%width%height%frameRate%codec->name));

#if LIBAVFORMAT_VERSION_INT >= (54<<16)
        AVDictionary * RetunedAVDic=NULL;
        if (avcodec_open2(codec_ctx, codec,&RetunedAVDic) < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0("Unable to open codec",ORE_Assert);
        }

        int ret = avio_open(&_output->pb, filename.c_str(), AVIO_FLAG_WRITE);
        if (ret < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT("_StartVideo: Unable to open %s for writing: %d\n", filename%ret,ORE_Assert);
        }
        _bWroteURL = true;
        avformat_write_header(_output,NULL);
#else
        if (avcodec_open(codec_ctx, codec) < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0("Unable to open codec",ORE_Assert);
        }

        int ret = url_fopen(&_output->pb, filename.c_str(), URL_WRONLY);
        if (ret < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT("_StartVideo: Unable to open %s for writing: %d\n", filename%ret,ORE_Assert);
        }
        _bWroteURL = true;

        av_write_header(_output);
#endif
        _bWroteHeader = true;

#if LIBAVFORMAT_VERSION_INT >= (55<<16)
        _picture = av_frame_alloc();
        _yuv420p = av_frame_alloc();
#else
        _picture = avcodec_alloc_frame();
        _yuv420p = avcodec_alloc_frame();
#endif

        _outbuf_size = 500000;
        _outbuf = (char*)malloc(_outbuf_size);
        BOOST_ASSERT(!!_outbuf);

#if LIBAVFORMAT_VERSION_INT >= (55<<16)
        _picture_size = avpicture_get_size(AV_PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);
#else
        _picture_size = avpicture_get_size(PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);
#endif
        _picture_buf = (char*)malloc(_picture_size);
        BOOST_ASSERT(!!_picture_buf);

#if LIBAVFORMAT_VERSION_INT >= (55<<16)
        avpicture_fill((AVPicture*)_yuv420p, (uint8_t*)_picture_buf, AV_PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);
#else
        avpicture_fill((AVPicture*)_yuv420p, (uint8_t*)_picture_buf, PIX_FMT_YUV420P, codec_ctx->width, codec_ctx->height);
#endif
    }

    void _AddFrame(void* pdata)
    {
        boost::mutex::scoped_lock lock(_mutexlibrary);
        if( !_output ) {
            RAVELOG_DEBUG("video resources destroyed\n");
            return;
        }

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
#if LIBAVFORMAT_VERSION_INT >= (55<<16)
        img_convert_ctx = sws_getContext(_stream->codec->width, _stream->codec->height, AV_PIX_FMT_BGR24, _stream->codec->width, _stream->codec->height, AV_PIX_FMT_YUV420P, SWS_BICUBIC /* flags */, NULL, NULL, NULL);
#else
        img_convert_ctx = sws_getContext(_stream->codec->width, _stream->codec->height, PIX_FMT_BGR24, _stream->codec->width, _stream->codec->height, AV_PIX_FMT_YUV420P, SWS_BICUBIC /* flags */, NULL, NULL, NULL);
#endif
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


#if LIBAVFORMAT_VERSION_INT >= (54<<16)
        int got_packet = 0;
        AVPacket pkt;
        av_init_packet(&pkt);
        int ret = avcodec_encode_video2(_stream->codec, &pkt, _yuv420p, &got_packet);
        if( ret < 0 ) {
#if LIBAVFORMAT_VERSION_INT >= (55<<16)
            av_free_packet(&pkt);
#else
            av_destruct_packet(&pkt);
#endif
            throw OPENRAVE_EXCEPTION_FORMAT("avcodec_encode_video2 failed with %d",ret,ORE_Assert);
        }
        if( got_packet ) {
            if( _stream->codec->coded_frame) {
                _stream->codec->coded_frame->pts       = pkt.pts;
                _stream->codec->coded_frame->key_frame = !!(pkt.flags & AV_PKT_FLAG_KEY);
            }
            if( av_write_frame(_output, &pkt) < 0) {
#if LIBAVFORMAT_VERSION_INT >= (55<<16)
                av_free_packet(&pkt);
#else
                av_destruct_packet(&pkt);
#endif
                throw OPENRAVE_EXCEPTION_FORMAT0("av_write_frame failed",ORE_Assert);
            }
        }
#if LIBAVFORMAT_VERSION_INT >= (55<<16)
        av_free_packet(&pkt);
#else
        av_destruct_packet(&pkt);
#endif
#else
        int size = avcodec_encode_video(_stream->codec, (uint8_t*)_outbuf, _outbuf_size, _yuv420p);
        if (size < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0("error encoding frame",ORE_Assert);
        }

        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = (uint8_t*)_outbuf;
        pkt.size = size;
        pkt.stream_index = _stream->index;
        //RAVELOG_INFO("%d\n",index);
        pkt.pts = _frameindex++;
        if( av_write_frame(_output, &pkt) < 0) {
            throw OPENRAVE_EXCEPTION_FORMAT0("av_write_frame failed",ORE_Assert);
        }

#endif
        _nFrameCount++;
    }
#endif
};

ModuleBasePtr CreateViewerRecorder(EnvironmentBasePtr penv, std::istream& sinput) {
    return ModuleBasePtr(new ViewerRecorder(penv,sinput));
}
void DestroyViewerRecordingStaticResources()
{
#ifdef _WIN32
    s_pVideoGlobalState.reset();
#endif
}

// Copyright (C) 2011 Rosen Diankov (rdiankov@cs.cmu.edu)
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
#ifndef OPENRAVE_CAMERAVIEWER_H
#define OPENRAVE_CAMERAVIEWER_H

class QtCameraViewer : public ViewerBase
{
public:
    QtCameraViewer(EnvironmentBasePtr penv, std::istream& sinput) : ViewerBase(penv) {
        __description = ":Interface Author: Rosen Diankov\n\nProvides a camera image viewer (Qt4). Can attach to any camera by specifying the camera name in the type string when creating the interface. The qtcoin viewer needs to be set ";
        string name; sinput >> name;
        _psensor = penv->GetSensor(name);
        if( !_psensor || !_psensor->Supports(SensorBase::ST_Camera) ) {
            RAVELOG_WARN(str(boost::format("failed to find camera sensor %s")%name));
        }
        else {
            QWidgetList widgets = QApplication::topLevelWidgets();
            if( widgets.empty() ) {
                RAVELOG_WARN("no qt viewer active, so cannot display camera images\n");
            }
            else {
                // post on all of them
                for(int i = 0; i < widgets.size(); ++i) {
                    QApplication::postEvent(widgets.at(i),new MyCallbackEvent(boost::bind(&QtCameraViewer::_CreateImageWindow,this)));
                }
            }
        }
    }
    virtual ~QtCameraViewer() {
        QWidgetList widgets = QApplication::topLevelWidgets();
        // post on all of them
        for(int i = 0; i < widgets.size(); ++i) {
            QApplication::postEvent(widgets.at(i),new MyCallbackEvent(boost::bind(&QtCameraViewer::_DestroyImageWindow,_imagewindow)));
        }
        _imagewindow.reset();
    }

    virtual int main(bool bShow = true)
    {
        return 0;
    }
    virtual void quitmainloop()
    {
    }

    virtual void Reset()
    {
    }

    virtual void ViewerSetSize(int w, int h)
    {
    }
    virtual void ViewerMove(int x, int y)
    {
    }
    virtual void ViewerSetTitle(const std::string& ptitle)
    {
    }

protected:
    class QtImageWindow : public QWidget
    {
public:
        QtImageWindow(SensorBasePtr psensor) : QWidget(NULL), _psensor(psensor) {
            _pdata = boost::static_pointer_cast<SensorBase::CameraSensorData>(_psensor->CreateSensorData(SensorBase::ST_Camera));
            _pdatanew = boost::static_pointer_cast<SensorBase::CameraSensorData>(_psensor->CreateSensorData(SensorBase::ST_Camera));
            _pgeom = boost::static_pointer_cast<SensorBase::CameraGeomData>(_psensor->GetSensorGeometry(SensorBase::ST_Camera));
            if( !_pdata || !_pgeom ) {
                throw openrave_exception(str(boost::format("QtImageWindow: failed to create sensor data for sensor %s")%_psensor->GetName()));
            }
            QHBoxLayout *hbox = new QHBoxLayout(this);
            _label = new QLabel(this);
            hbox->addWidget(_label);
            setLayout(hbox);
            setWindowTitle(str(boost::format("camera %s")%_psensor->GetName()).c_str());
            startTimer(1);
        }

        void timerEvent(QTimerEvent* event)
        {
            if( _psensor->GetSensorData(_pdatanew) &&( _pdatanew->__stamp != _pdata->__stamp) ) {
                if( _pgeom->width*_pgeom->height*3 != (int)_pdatanew->vimagedata.size() ) {
                    RAVELOG_WARN(str(boost::format("QtImageWindow: sensor %s image wrong dims")%_psensor->GetName()));
                }
                else {
#if QT_VERSION >= 0x040400 // qt4.4+
                    _label->setPixmap(QPixmap::fromImage(QImage(&_pdatanew->vimagedata[0], _pgeom->width,_pgeom->height,QImage::Format_RGB888)));
#else
                    vimagedata.resize(4*_pgeom->width*_pgeom->height);
                    for(int i = 0; i < _pgeom->width*_pgeom->height; ++i) {
                        vimagedata[4*i+0] = _pdatanew->vimagedata[3*i+0];
                        vimagedata[4*i+1] = _pdatanew->vimagedata[3*i+1];
                        vimagedata[4*i+2] = _pdatanew->vimagedata[3*i+2];
                        vimagedata[4*i+3] = 0xff;
                    }
                    _label->setPixmap(QPixmap::fromImage(QImage(&vimagedata[0], _pgeom->width,_pgeom->height,QImage::Format_RGB32)));
#endif
                    swap(_pdatanew,_pdata);
                }
            }
        }

private:
        QLabel *_label;
        SensorBasePtr _psensor;
        boost::shared_ptr<SensorBase::CameraSensorData> _pdatanew, _pdata;
        boost::shared_ptr<SensorBase::CameraGeomData> _pgeom;
#if QT_VERSION < 0x040400 // qt4.4+
        vector<uint8_t> vimagedata;
#endif
    };

    virtual void _CreateImageWindow()
    {
        boost::mutex::scoped_lock lock(_mutex);
        if( !_imagewindow ) {
            _imagewindow.reset(new QtImageWindow(_psensor));
            _imagewindow->show();
        }
    }

    static void _DestroyImageWindow(boost::shared_ptr<QtImageWindow> imagewindow)
    {
    }

    SensorBasePtr _psensor;
    boost::shared_ptr<QtImageWindow> _imagewindow;
    boost::mutex _mutex;
};

#endif

// -*- coding: utf-8 -*-
// Copyright (C) 2012 Gustavo Puche, Rosen Diankov, OpenGrasp Team
//
// OpenRAVE Qt/OpenSceneGraph Viewer is licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <openrave/plugin.h>
#include <openrave/utils.h>

#include "qtosg.h"

#include <QApplication>
#include <QTimer>

#if defined(HAVE_X11_XLIB_H) && defined(Q_WS_X11)
#include <X11/Xlib.h>
#endif

using namespace OpenRAVE;

namespace qtosgrave {
ViewerBasePtr CreateQtOSGViewer(EnvironmentBasePtr penv, std::istream& sinput);

/// \brief helper class used to create a viewer in the UI thread
class QtOSGViewerCreator
{
public:
    QtOSGViewerCreator(EnvironmentBasePtr penv, std::istream& sinput)
    {
        _penv = penv;
        stringbuf buf;
        sinput.get(buf, 0); // get all the data, yes this is inefficient, not sure if there anyway to search in streams
        _s = buf.str();
    }

    void ProcessEvent()
    {
        boost::mutex::scoped_lock lock (_mutex);
        if( !_viewer ) {
            std::stringstream sinput(_s);
            _viewer = CreateQtOSGViewer(_penv, sinput);
//            if( !!_viewer ) {
//                _viewer->Show(1); // TODO remove once Show posts to queue
//            }
        }
    }

    ViewerBasePtr GetViewer() {
        boost::mutex::scoped_lock lock (_mutex);
        return _viewer;
    }

private:
    boost::mutex _mutex;
    EnvironmentBasePtr _penv;
    std::string _s;
    ViewerBasePtr _viewer;
};

}

// for some reason windows complains when the prototypes are different
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    //	Debug.
    RAVELOG_VERBOSE("Initiating QTOSGRave plugin...!!!!.\n");

    switch(type) {
    case PT_Viewer:
#if defined(HAVE_X11_XLIB_H) && defined(Q_WS_X11)
        // always check viewers since DISPLAY could change
        if ( XOpenDisplay( NULL ) == NULL ) {
            RAVELOG_WARN("no display detected, so cannot load viewer");
            return InterfaceBasePtr();
        }
#endif
        if( interfacename == "qtosg" ) {
            // not sure if should be creating application all the time...

            //if( QApplication::startingUp() ) {

            //CoreApplication::postEvent ( QObject * receiver, QEvent * event )
            // QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
            // QCoreApplication::setAttribute(Qt::AA_DisableHighDpiScaling);
            QCoreApplication *qapp = QApplication::instance();
            if( !qapp ) {
                RAVELOG_VERBOSE("qt is not initialized yet, so initializing...\n");
                static int s_QtArgc = 0; // has to be static!
                qapp = new QApplication(s_QtArgc,NULL);
                return qtosgrave::CreateQtOSGViewer(penv, sinput);
            }
            else {
                QWidgetList widgets = QApplication::topLevelWidgets();
                if( widgets.empty() ) {
                    RAVELOG_WARN("widget are empty, but application exists?\n");
                    return qtosgrave::CreateQtOSGViewer(penv, sinput); // no idea...
                }
                else {
                    // The function below can be posted before qt app's exec()
                    // is called, so the function below won't get executed
                    // immediately and the GetViewer function will time out.
                    //
                    // QThread::eventDispatcher(), which can be used to tell if
                    // an event loop is running in a thread, is only available
                    // in QT5.
                    return qtosgrave::CreateQtOSGViewer(penv, sinput);
#if 0
                    //RAVELOG_DEBUG("detect QApplication, so exiting from GUI thread in order to safely create\n");
                    RAVELOG_DEBUG("detect QApplication, so attempting to create new viewer in original GUI thread\n");
                    boost::shared_ptr<qtosgrave::QtOSGViewerCreator> creator(new qtosgrave::QtOSGViewerCreator(penv, sinput));
                    // post on all of them
                    for(int i = 0; i < widgets.size(); ++i) {
                        QApplication::postEvent(widgets.at(i), new qtosgrave::MyCallbackEvent(boost::bind(&qtosgrave::QtOSGViewerCreator::ProcessEvent, creator)));
                    }
                    uint32_t starttime = utils::GetMilliTime();
                    while(utils::GetMilliTime() - starttime < 5000 ) {
                        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                        if( !!creator->GetViewer() ) {
                            break;
                        }
                    }
                    ViewerBasePtr pviewer = creator->GetViewer();
                    if( !pviewer ) {
                        RAVELOG_WARN("timeout trying to create viewer!\n");
                    }
                    return pviewer;
#endif
                }


//                QApplication::postEvent(NULL, new qtosgrave::MyCallbackEvent(qtosgrave::RunTick));
//                QApplication::sendPostedEvents(NULL, CALLBACK_EVENT);
                //QApplication::exit(0); // exit from any loops in order to safely create the new viewer...
                //return qtosgrave::CreateQtOSGViewer(penv, sinput);
                //qtosgrave::MyObject m;
                //boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
                //RAVELOG_INFO("asdfasdf\n");
                //return InterfaceBasePtr();
            }
        }
        break;
    default:
        break;
    }

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Viewer].push_back("qtosg");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    // necessary since QApplication does not destroy all threads when last SoQt viewer is done
    //removePostedEvents - sometimes freezes on this function
    //QApplication::quit();
}

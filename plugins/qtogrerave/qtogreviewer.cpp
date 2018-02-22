#include "qtogreviewer.h"

namespace qtogrerave {

QtOgreViewer::QtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput) : ViewerBase(penv) {
    __description = ":Interface Author: Woody Chow\n\nQt/Ogre Viewer";

    // RegisterCommand("test", boost::bind(&QtOgreViewer::quitmainloop, this),
    //                 "testtttttttttt");
    main(true);
}

int QtOgreViewer::main(bool bShow)
{
    _ogreWindow = boost::make_shared<QtOgreWindow>();
    // _ogreWindow->show();
    _ogreWindow->showMaximized();
    QGuiApplication::instance()->exec();
}

void QtOgreViewer::quitmainloop()
{

}

ViewerBasePtr CreateQtOgreViewer(EnvironmentBasePtr penv, std::istream& sinput)
{
    return boost::make_shared<QtOgreViewer>(penv, sinput);
}

}; // namespace qtogrerave
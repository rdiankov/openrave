// This class is based on http://wiki.ogre3d.org/Integrating+Ogre+into+QT5

#ifndef QTOGREWINDOW_H
#define QTOGREWINDOW_H

/*
Qt headers
*/
#include <QtWidgets/QApplication>
#include <QtGui/QKeyEvent>
#include <QtGui/QWindow>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

/*
Ogre3D header
*/
#include <Ogre.h>

/*
Changed SdkCameraMan implementation to work with QKeyEvent, QMouseEvent, QWheelEvent
*/
#include "cameraman.h"

/*
With the headers included we now need to inherit from QWindow.
*/
class QtOgreWindow : public QWindow, public Ogre::FrameListener
{
    /*
    A QWindow still inherits from QObject and can have signals/slots; we need to add the appropriate
    Q_OBJECT keyword so that Qt's intermediate compiler can do the necessary wireup between our class
    and the rest of Qt.
    */
    Q_OBJECT

public:
    explicit QtOgreWindow(QWindow *parent = NULL);
    virtual ~QtOgreWindow();

    /*
    We declare these methods virtual to allow for further inheritance.
    */
    virtual void render(QPainter *painter);
    virtual void render();
    virtual void initialize();
    virtual void createScene();
#if OGRE_VERSION >= ((2 << 16) | (0 << 8) | 0)
    virtual void createCompositor();
#endif

    void setAnimating(bool animating);

    Ogre::Root* GetRoot() { return m_ogreRoot; }
    Ogre::SceneNode* GetMiscDrawNode() { return m_miscDrawNode; }
    Ogre::HlmsDatablock *datablockhack;

    void QueueRenderingUpdate(const boost::function<void()> &func) {
        boost::mutex::scoped_lock lock(_mutexFrameRenderingUpdate);
        _frameRenderingUpdateQueue.push_back(func);
    }

public slots:

    virtual void renderLater();
    virtual void renderNow();

    /*
    We use an event filter to be able to capture keyboard/mouse events. More on this later.
    */
    virtual bool eventFilter(QObject *target, QEvent *event);

signals:
    /*
    Event for clicking on an entity.
    */
    void entitySelected(Ogre::v1::Entity* entity);

protected:
    /*
    Ogre3D pointers added here. Useful to have the pointers here for use by the window later.
    */
    Ogre::Root* m_ogreRoot;
    Ogre::RenderWindow* m_ogreWindow;
    Ogre::SceneManager* m_ogreSceneMgr;
    Ogre::Camera* m_ogreCamera;
    Ogre::ColourValue m_ogreBackground;
    OgreQtBites::SdkQtCameraMan* m_cameraMan;

    // Node used to store all misc.Draw objects
    Ogre::SceneNode* m_miscDrawNode;

    bool m_update_pending;
    bool m_animating;

    /*
    The below methods are what is actually fired when they keys on the keyboard are hit.
    Similar events are fired when the mouse is pressed or other events occur.
    */
    virtual void keyPressEvent(QKeyEvent * ev);
    virtual void keyReleaseEvent(QKeyEvent * ev);
    virtual void mouseMoveEvent(QMouseEvent* e);
    virtual void wheelEvent(QWheelEvent* e);
    virtual void mousePressEvent(QMouseEvent* e);
    virtual void mouseReleaseEvent(QMouseEvent* e);
    virtual void exposeEvent(QExposeEvent *event);
    virtual bool event(QEvent *event);

    std::list<std::function<void()>> _frameRenderingUpdateQueue;
    boost::mutex _mutexFrameRenderingUpdate;

    /*
    FrameListener method
    */
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    /*
    Write log messages to Ogre log
    */
    void log(Ogre::String msg);
    void log(QString msg);
};

#endif // QTOGREWINDOW_H
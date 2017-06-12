#include "qtosgviewer.h"

#include <unistd.h>
#include <X11/Xlib.h>

static int qtosgrave_xio_errhandler(Display *)
{
    for (;;) {
        RAVELOG_FATAL("X fatal I/O error");
        pause();
    }
    return 0;
}

namespace qtosgrave {

    void SetXErrorHandlers() {
        XSetIOErrorHandler(qtosgrave_xio_errhandler);
    }
}


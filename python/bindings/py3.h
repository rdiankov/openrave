/*
    Python3 definition file to consistently map the code to Python 2 or
    Python 3.

    PyInt and PyLong were merged into PyLong in Python 3, so all PyInt functions
    are mapped to PyLong.
*/

#if PY_VERSION_HEX >= 0x03000000

/* Map PyInt -> PyLong */
#define PyNumber_Int                PyNumber_Long

#else   /* PY_VERSION_HEX < 0x03000000 */

/* Map PyBytes -> PyString */
#define PyBytes_AsString            PyString_AsString
#define PyBytes_Check               PyString_Check
#define PyBytes_FromStringAndSize   PyString_FromStringAndSize

#endif  /* PY_VERSION_HEX < 0x03000000 */

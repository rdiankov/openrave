rem If running this on MATLAB command line, the command might look like:

rem mex -lws2_32 -L"C:\Program Files\etc.." orcreate.cpp


mex.bat "ws2_32.lib" orcreate.cpp

mex.bat "ws2_32.lib" orread.cpp

mex.bat "ws2_32.lib" orwrite.cpp

# C++ call python

## Windows

1. Add header
   Add "-I Path/to/python/include" in compile command

   Note: if error "::hypot has not been declared" occurs, add "#define _hypot hypot" in your mingw32/7.x.x/include/c++/cmath

2. Add linker
   Add "Path/to/python/libs/python36.lib" in compile command, note that pythonxx.lib correspond to your python version.

3. Add dynamic library
   Copy "python36.dll" to your executable folder

4. SetPythonHomePath
   Before use "Py_Initialize()", use "Py_SetPythonHome(L"Path/to/python/Lib;Path/to/project/folder");"
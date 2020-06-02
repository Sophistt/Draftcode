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


## Linux

Using the default python interpretor is different from using the specific python interpretor in conda.

1. Using the default python2.7 is easy to compile with command "g++ call_python.cpp -I/usr/include/python2.7/ -lpyhton2.7"

2. Using the specific python interpretor is more complex. First add header
   Add "-I/Path/to/conda/envs/python/include/python3.6" in compile command

3. Add linker
   Add "-L/Path/to/conda/envs/python/lib -lpython3.6" in compile command

4. Add dynamic library
   if error "libpython36.so.1.0 was not found" occurs while executing the executable, add "Path/to/conda/envs/python/lib" in "/etc/ld.so.conf", then execute "sudo /sbin/ldconfig -v"

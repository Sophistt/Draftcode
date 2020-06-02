#include "Python.h"
#include <iostream>

int main() 
{   
    // Py_SetPythonHome(L"F:/Technology/Miniconda3/envs/py3.6/Lib;D:/Code/Draftcode/C++/call_python");
    Py_Initialize();
    PyRun_SimpleString("print('Hello!')");
    Py_Finalize();

    return 0;
}

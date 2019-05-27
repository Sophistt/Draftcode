# 版本限定 
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)     
# 项目名称 
PROJECT(TestClass)
# 设置C语言编译器选项： O3 优化代码，Wall 显示所有警告, march=native自动匹配编译器类型
SET(CMAKE_C_FLAGS "$ENV{CFLAGS} -O3 -Wall -march=native") 

# 使用 C++ 11 编译
add_definitions(-std=c++11)

# 定义头文件的路径变量
SET(INCLUDE_PATH ${PROJECT_SOURCE_DIR}/include) 
MESSAGE(STATUS "Include Path, ${INCLUDE_PATH}")

# 定义库文件路径
SET(LIB_PATH ${PROJECT_SOURCE_DIR}/libs)
MESSAGE(STATUS "Libs Path, ${LIB_PATH}")

# 定义源文件路径变量
SET(SOURCE_PATH ${PROJECT_SOURCE_DIR}/src) 
MESSAGE(STATUS "Source Path , ${SOURCE_PATH}") 

# 包含头文件搜索路径
INCLUDE_DIRECTORIES(${INCLUDE_PATH}) 

# 包含库搜索路径
LINK_DIRECTORIES(${LIB_PATH})

# 定义待链接库名字
# 链接静态库
#SET(LIBS_LIST libmymath.a)                                                                                     
# 链接动态库
#SET(LIBS_LIST libmymath.so)

# 添加源文件路径下所有源文件存放到变量中(*.c && *.cpp)
AUX_SOURCE_DIRECTORY(${SOURCE_PATH} SRC_LIST)
# 将输出的可执行文件放到工程目录的bin目录下
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# 设置生成可执行文件的名称
# SET(EXECUTABLE_FILE_NAME mymath.exe)
# 生成可执行文件
ADD_EXECUTABLE(${PROJECT_NAME} ${SRC_LIST})
# 设置可执行文件链接的库名称
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${LIBS_LIST})
cmake_minimum_required(VERSION 3.15)

project(Armor LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

#查找 yaml-cpp 库
find_package(yaml-cpp REQUIRED)

# 添加源文件
set(SOURCES
    ArmorFunction.cpp
    main.cpp
)

# 添加头文件
set(HEADERS
    commen.h
)

# 添加可执行文件
add_executable(Armor ${SOURCES} ${HEADERS})

# 链接 OpenCV 库
target_link_libraries(Armor ${OpenCV_LIBS})

#链接 yaml-cpp 库
target_link_libraries(Armor yaml-cpp)

# 包含 OpenCV 头文件路径
target_include_directories(Armor PRIVATE ${OpenCV_INCLUDE_DIRS})

#包含 yaml-cpp 头文件路径
target_include_directories(Armor PRIVATE ${YAML_CPP_INCLUDE_DIR})
cmake_minimum_required(VERSION 3.29)
project(PointCloud)

set(CMAKE_CXX_STANDARD 17)

#--- CMake configuration
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(CMAKE_DEBUG_POSTFIX "")
#--- OpenMP
find_package(OpenMP QUIET)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp")
endif()

#--- CGAL
find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDE_DIRS})
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

#--- Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

#--- Nanoflann
find_package(NanoFlann REQUIRED)
include_directories(${NANOFLANN_INCLUDE_DIR})

#--- PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#--- Boost
find_package(Boost REQUIRED
        COMPONENTS system filesystem thread)
include_directories(${Boost_INCLUDE_DIRS})

#--- CoppeliaSimRemoteAPI
set(COPPELIASIM_ROOT_DIR "C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu")
set(COPPELIASIM_INCLUDE_DIR "C:\\Program Files\\CoppeliaRobotics\\CoppeliaSimEdu\\programming\\include")
list(APPEND CMAKE_MODULE_PATH
        ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules
        ${COPPELIASIM_INCLUDE_DIR}/cmake)
find_package(CoppeliaSim REQUIRED)
set(Python3_EXECUTABLE "C:\\Users\\eryao\\miniconda3\\envs\\torch\\python.exe")
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
set(PYTHON_LIBRARY "/path/to/python310.lib")
set(GENERATE OFF CACHE BOOL "Generate wrappers for objects and methods (requires CoppeliaSim to be running).")
set(GENERATE_INCLUDE_OBJECTS "" CACHE STRING "Include only given objects (e.g.: \"sim,simIK\") or leave blank to include all objects in wrapper code generation.")
set(GENERATE_EXCLUDE_OBJECTS "simEigen,simB0,simRemoteApi,simQML,simOMPL,simUI,simIM,simIGL,simURDF,simSkel,simBWF,simCmd,simSubprocess,simURLDrop,simWS,simZMQ" CACHE STRING "Exclude given objects (e.g.: \"simUI,simIM\") from being generated. Note: option GENERATE_INCLUDE_OBJECTS, if different from \"\", has precedence over this.")
set(GENERATE_EXCLUDE_METHODS "sim.test,sim.auxFunc,sim.getStringSignal,sim.getInt32Signal,sim.getFloatSignal" CACHE STRING "Exclude given methods (e.g.: \"sim.test,simUI.foo\") from being generated.")
include(FetchContent)
FetchContent_Declare(jsoncons
        GIT_REPOSITORY https://github.com/danielaparker/jsoncons
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/jsoncons
)
FetchContent_GetProperties(jsoncons)
if(NOT jsoncons_POPULATED)
    FetchContent_Populate(jsoncons)
endif()

set(CPPZMQ_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(cppzmq
        GIT_REPOSITORY https://github.com/zeromq/cppzmq
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/cppzmq
)
FetchContent_GetProperties(cppzmq)
if(NOT cppzmq_POPULATED)
    FetchContent_Populate(cppzmq)
    add_subdirectory(${cppzmq_SOURCE_DIR} ${cppzmq_BINARY_DIR} EXCLUDE_FROM_ALL)
endif()

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)

add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
        COMMAND
        ${CMAKE_COMMAND} -E env
        PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/python/src"
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/tools/get_raw_calltips.py"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
        DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/tools/get_raw_calltips.py"
)

add_custom_command(
        OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
        COMMAND
        ${CMAKE_COMMAND} -E env
        PYTHONPATH="${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/python/src"
        ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/tools/get_constants.py"
        "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
        DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/tools/get_constants.py"
)

set(generatedFiles)
file(GLOB templateFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/cpp/templates/ ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/cpp/templates/*)
foreach(templateFile ${templateFiles})
    add_custom_command(
            OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}"
            COMMAND
            ${Python3_EXECUTABLE}
            "${COPPELIASIM_INCLUDE_DIR}/simStubsGen/external/pycpp/pycpp.py"
            -p "calltips_json=${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
            -p "constants_json=${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
            -p "include_objects=${GENERATE_INCLUDE_OBJECTS}"
            -p "exclude_objects=${GENERATE_EXCLUDE_OBJECTS}"
            -p "exclude_methods=${GENERATE_EXCLUDE_METHODS}"
            -i "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}"
            -o "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}"
            -P "${COPPELIASIM_PYTHON_DIR}"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/tools"
            -P "${CMAKE_CURRENT_SOURCE_DIR}"
            DEPENDS
            "${COPPELIASIM_INCLUDE_DIR}/simStubsGen/external/pycpp/pycpp.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/templates/${templateFile}"
            "${CMAKE_CURRENT_BINARY_DIR}/generated/calltips.json"
            "${CMAKE_CURRENT_BINARY_DIR}/generated/constants.json"
            "${CMAKE_CURRENT_SOURCE_DIR}/cpp_utils.py"
            "${COPPELIASIM_PYTHON_DIR}/calltip.py"
            "${COPPELIASIM_PYTHON_DIR}/calltip.lark"
    )
    list(APPEND generatedFiles "${CMAKE_CURRENT_BINARY_DIR}/generated/${templateFile}")
endforeach()
add_custom_target(generate_code DEPENDS ${generatedFiles})

add_library(RemoteAPIClient STATIC ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/cpp/RemoteAPIClient.cpp)
if(GENERATE)
    add_dependencies(RemoteAPIClient generate_code)
else()
    foreach(templateFile ${templateFiles})
        if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/cpp/${templateFile}")
            message(FATAL_ERROR "File ${CMAKE_CURRENT_SOURCE_DIR}/${templateFile} is missing")
        endif()
    endforeach()
endif()
target_compile_definitions(RemoteAPIClient PUBLIC -DSIM_REMOTEAPICLIENT_OBJECTS)
target_include_directories(RemoteAPIClient PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/jsoncons/include)
if(GENERATE)
    target_include_directories(RemoteAPIClient BEFORE PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
    set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/generated/RemoteAPIObjects.h)
    set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/generated/RemoteAPIObjects.cpp)
else()
    set_source_files_properties(RemoteAPIClient.h OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/cpp/RemoteAPIObjects.h)
    set_source_files_properties(RemoteAPIClient.cpp OBJECT_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/cpp/RemoteAPIObjects.cpp)
endif()
target_link_libraries(RemoteAPIClient PUBLIC cppzmq)

#--- Build
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj /utf-8")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/CoppeliaSimRemoteApi/cpp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ricp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/class)
file(GLOB_RECURSE SOURCE src/*.cpp class/*.cpp)
add_executable(PointCloud main.cpp ${SOURCE})
target_link_libraries (PointCloud ${PCL_LIBRARIES} ${Boost_LIBRARIES} RemoteAPIClient)


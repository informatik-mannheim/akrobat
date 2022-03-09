cmake_minimum_required (VERSION 3.13)

if (NOT ROYALE_ENABLE_QT)
    message(STATUS "Qt disabled Qt example will not build")
    return ()
endif ()
# On systems that need it, find QT based on the QTDIR environment variable
if (DEFINED ENV{QTDIR})
    set (CMAKE_PREFIX_PATH "$ENV{QTDIR}")
endif ()

if (POLICY CMP0020)
    cmake_policy (SET CMP0020 NEW)
endif ()

# Widgets finds its own dependencies.
find_package (Qt5Widgets 5 QUIET)

if (NOT Qt5Widgets_FOUND)
    message (
        "Qt example will not be build as no Qt was found! Please set the CMAKE_PREFIX_PATH to the lib/cmake folder inside your Qt installation"
    )
    return ()
endif ()

# As moc files are generated in the binary dir, tell CMake to always look for includes there:
set (CMAKE_INCLUDE_CURRENT_DIR ON)

set (CMAKE_PREFIX_PATH "../../../share")

set (SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
             "${CMAKE_CURRENT_SOURCE_DIR}/src/sampleQtViewer.cpp")

set (HEADERS)

set (MOC_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/inc/sampleQtViewer.hpp")

set (UI_FILES "${CMAKE_CURRENT_SOURCE_DIR}/src/mainwindow.ui")

find_package (royale REQUIRED)

link_directories (${royale_LIB_DIR})

qt5_wrap_ui (UI_HEADERS ${UI_FILES})
qt5_wrap_cpp (MOCED_HEADERS ${MOC_HEADERS})

project (sampleQtViewer)

add_executable (sampleQtViewer ${SOURCES} ${HEADERS} ${MOC_HEADERS} ${UI_HEADERS} ${MOCED_HEADERS}
                               ${ASSEMBLY_RC})

copy_royale_libs (sampleQtViewer)

target_include_directories (sampleQtViewer PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../inc
                                                   ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_include_directories (sampleQtViewer SYSTEM PRIVATE ${Qt5Widgets_INCLUDE_DIRS}
                                                          ${Qt5OpenGL_INCLUDE_DIRS})

target_link_libraries (sampleQtViewer "${royale_LIBS}")
target_link_libraries (sampleQtViewer Qt5::Widgets)

set_target_properties (sampleQtViewer PROPERTIES WIN32_EXECUTABLE ON)

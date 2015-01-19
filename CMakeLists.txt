cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcs-visualizer)

set(CMAKE_BUILD_TYPE Debug)

find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)

#Open CV for GMM - Mustafa
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp mainwindow.cpp pclviewer.cpp pcs.cpp object.cpp pcssegmentation.cpp)
set  (project_HEADERS mainwindow.h pclviewer.h pcs.h object.h pcssegmentation.h)
set  (project_FORMS   mainwindow.ui pclviewer.ui)
set  (VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pcs_visualizer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (pcs_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS})

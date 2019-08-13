CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#qt_automoc(myGLWidget.h)
QT5_WRAP_UI(UISrcs mainwindow.ui)

ADD_LIBRARY(opengl_util
	myGLWidget.cpp
	opengl/Mesh.cpp
	opengl/RenderedMesh.cpp
	${MOCSrcs}
 	${UISrcs}
)

target_link_libraries(opengl_util
	${GLEW_LIB}
	${GLUT_LIB}
	debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
    debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
    debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
    debug SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized SimTKcommon
    debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
	debug SimTKmath${CMAKE_DEBUG_POSTFIX} optimized SimTKmath
	debug SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized SimTKsimbody
	${QT_LIBRARIES}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}
)

ADD_LIBRARY(SharedThread
	SyncToolsIK.cpp
	${PROJECT_SOURCE_DIR}/src/SemaphoreRT.cpp
)

target_link_libraries(SharedThread
	${Boost_LIBRARY}
	${Boost_THREAD_LIBRARY}
	${Boost_TIMER_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	${Boost_PROGRAM_OPTIONS_LIBRARY}
	${Boost_FILESYSTEM_LIBRARY}
)


ADD_LIBRARY(Gui
	mainwindow.cpp
	qcustomplot.cpp
	${MOCSrcs}
	${UISrcs}
)

target_link_libraries(Gui
	opengl_util
	${QT_LIBRARIES}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}
	debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
    debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
    debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
    debug SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized SimTKcommon
    debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
	debug SimTKmath${CMAKE_DEBUG_POSTFIX} optimized SimTKmath
	debug SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized SimTKsimbody
	SharedThread
	${WINLIB}
	${WINMM}
)

ADD_LIBRARY(Gui2Model 
	mainwindow2Model.cpp
	qcustomplot.cpp
	${MOCSrcs}
	${UISrcs}
)

target_link_libraries(Gui2Model
	opengl_util
	${QT_LIBRARIES}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}	
	debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
    debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
    debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
    debug SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized SimTKcommon
    debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
	debug SimTKmath${CMAKE_DEBUG_POSTFIX} optimized SimTKmath
	debug SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized SimTKsimbody
	SharedThread
)


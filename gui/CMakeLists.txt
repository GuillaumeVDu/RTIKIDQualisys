CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#qt_automoc(myGLWidget.h)
#QT_WRAP_UI(UISrcs mainwindow.ui)

ADD_LIBRARY(opengl_util
	myGLWidget.cpp
	opengl/Mesh.cpp
	opengl/RenderedMesh.cpp
	${MOCSrcs}
 	${UISrcs}
)

ADD_LIBRARY(Gui STATIC
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
)

ADD_LIBRARY(Gui2Model STATIC
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
)

ADD_LIBRARY(GuiEmg STATIC
	mainwindowEMG.cpp
	qcustomplot.cpp
	${MOCSrcs}
	${UISrcs}
)

target_link_libraries(GuiEmg
	opengl_util
	${QT_LIBRARIES}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}
)

ADD_LIBRARY(shared STATIC
	SyncToolsEMG.cpp
	SyncToolsIK.cpp
	${CEINMS_PRODUCERS_HEADERS}/Semaphore.cpp
)

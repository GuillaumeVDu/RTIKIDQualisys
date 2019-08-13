CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

ADD_LIBRARY(QualisysTCPIP
	QualisysSocket/Markup.cpp
	QualisysSocket/Network.cpp
	QualisysSocket/RTPacket.cpp
	QualisysSocket/RTProtocol.cpp
)

TARGET_LINK_LIBRARIES(QualisysTCPIP
	${WINLIB}
	${WINMM}
)

ADD_LIBRARY(InvIkId
	InvKin.cpp
	InvKinImu.cpp
	IKSolverRT.cpp
	InvKinMarker.cpp
	InvDyn.cpp
	executionIK_ID_V2.cxx
	XMLInterpreterV2.cpp
	RTIKIDInterface.cpp
	KalmanFilterSimTK.cpp
	MotionCapturePreprocessing.cpp
	Laboratory.cxx
	QualisysClient.cpp
)


TARGET_LINK_LIBRARIES(InvIkId
	QualisysTCPIP
	SharedThread
	debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
    debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
    debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
    debug SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized SimTKcommon
    debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
	debug SimTKmath${CMAKE_DEBUG_POSTFIX} optimized SimTKmath
	debug SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized SimTKsimbody
	${XERCES_LIBRARY}
	${WINLIB}
	${WINMM}
)


ADD_LIBRARY(InvIkIdThreadV2
	IKSolverRT.cpp
	InvDynThread.cpp
	executionIK_ID_V2.cxx
	XMLInterpreterV2.cpp
	IKAndIDComputationV2.cpp
	KalmanFilterSimTK.cpp
	MotionCapturePreprocessing.cpp
	Laboratory.cxx
	QualisysClient.cpp
)

TARGET_LINK_LIBRARIES(InvIkIdThreadV2
	QualisysTCPIP
	FileIO
	SharedThread
	debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
    debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
    debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
    debug SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized SimTKcommon
    debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
	debug SimTKmath${CMAKE_DEBUG_POSTFIX} optimized SimTKmath
	debug SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized SimTKsimbody
	${XERCES_LIBRARY}
	${WINLIB}
	${WINMM}
)

ADD_EXECUTABLE(QualisysTest
	QualisysSocket/QualisysData.cpp
)

TARGET_LINK_LIBRARIES(QualisysTest
	QualisysTCPIP
)


ADD_EXECUTABLE(TestProgDoubleModel
	executable/TestDoubleModel.cpp
)

TARGET_LINK_LIBRARIES(TestProgDoubleModel
	InvIkIdThreadV2
	Gui2Model
	SharedThread
	opengl_util
	debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
    debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
    debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
    debug SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized SimTKcommon
    debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
	debug SimTKmath${CMAKE_DEBUG_POSTFIX} optimized SimTKmath
	debug SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized SimTKsimbody
	${XERCES_LIBRARY}
	${QT_LIBRARIES}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}
	${WINLIB}
	${WINMM}
)

ADD_EXECUTABLE(TestProgMarkerRTCaptureOfflineProcc executable/TestMarkerRTCaptureOfflineProcc.cpp
	CLIOption.cpp
)

TARGET_LINK_LIBRARIES(TestProgMarkerRTCaptureOfflineProcc
	InvIkId
	Gui
	SharedThread
	FileIO
	opengl_util
	debug osimTools${CMAKE_DEBUG_POSTFIX} optimized osimTools
    debug osimCommon${CMAKE_DEBUG_POSTFIX} optimized osimCommon
    debug osimSimulation${CMAKE_DEBUG_POSTFIX} optimized osimSimulation
    debug SimTKcommon${CMAKE_DEBUG_POSTFIX} optimized SimTKcommon
    debug osimActuators${CMAKE_DEBUG_POSTFIX} optimized osimActuators
	debug osimAnalyses${CMAKE_DEBUG_POSTFIX} optimized osimAnalyses
	debug SimTKmath${CMAKE_DEBUG_POSTFIX} optimized SimTKmath
	debug SimTKsimbody${CMAKE_DEBUG_POSTFIX} optimized SimTKsimbody
	${XERCES_LIBRARY}
	${QT_LIBRARIES}
	${OPENGL_gl_LIBRARY}
	${OPENGL_glu_LIBRARY}
	${WINLIB}
	${WINMM}
)


ADD_EXECUTABLE(TestProgMarkerThreadV2 executable/TestMarkerThreadV2.cpp
	CLIOption.cpp
)

TARGET_LINK_LIBRARIES(TestProgMarkerThreadV2
	SharedThread
	InvIkIdThreadV2
	Gui
	FileIO
	opengl_util
)

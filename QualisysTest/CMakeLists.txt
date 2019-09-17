CMAKE_MINIMUM_REQUIRED(VERSION 2.8)



ADD_EXECUTABLE(RTClientExample
	RTClientExample.cpp
	OutputSettings.cpp
	Output.cpp
	Operations.cpp
	Input.cpp
)

TARGET_LINK_LIBRARIES(RTClientExample
	QualisysTCPIP
	${WINLIB}
	${WINMM}
)
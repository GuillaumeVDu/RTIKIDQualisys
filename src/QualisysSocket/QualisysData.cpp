//#pragma warning( push )
//#pragma warning ( disable : ALL_CODE_ANALYSIS_WARNINGS )


#include "RTProtocol.h"
#pragma warning( pop )


#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <math.h>
#include <limits.h>
#include <string>
#include <iomanip>

// Including ROS and SFBcomm
#include <sstream>

using namespace std;

#define M_PI 3.14159265

// Start the main program
int main(int argc, char* argv[]) {
    const float deg2rad = M_PI/180.;



    string serverAddress = "10.76.115.143"; //"tracking1";		// The address of the computer connected to the Qualisys motion tracking system (ex: "130.75.144.179")
    int basePort = 22222; 					// The base port (as entered in QTM, TCP/IP port number, in the RT output tab of the workspace options

    // Defining global variables
    float x, y, z, roll, pitch, yaw;
    unsigned int markerCount;
    int markerIndex;
    int forceCount;
    float markerX, markerY, markerZ;
	unsigned int frameNumber;

    CRTPacket*             pRTPacket;
    CRTPacket::EPacketType eType;


    // Defining a protocol that connects to the Qualisys
    CRTProtocol poRTProtocol;

    // Connecting to the server
    cout << "Connecting to the Qualisys Motion Tracking system specified at: " << serverAddress << ":" << basePort << endl;

    if (!poRTProtocol.Connect((char*)serverAddress.data(), basePort, 0, 1, 7)){

        cout << "Could not find the Qualisys Motion Tracking system at: " << serverAddress << ":" << basePort << endl;
        return 0;
    }

    cout << "Connected to " << serverAddress << ":" << basePort << endl;
    cout << "Entering the measurement streaming loop..." << endl;

    int createBodyBufferFlag = 0;
    int createMarkerBufferFlag = 0;
    int printOnce = 0;

    // Infinite Measurement Loop
    while (true) {
        pRTPacket = poRTProtocol.GetRTPacket();
        frameNumber  = pRTPacket->GetFrameNumber();

        poRTProtocol.GetCurrentFrame(CRTProtocol::cComponentForce);

        if (poRTProtocol.ReceiveRTPacket(eType, true)) {
            switch (eType) {
                // Case 1 - sHeader.nType 0 indicates an error
                case CRTPacket::PacketError :
                    cout << "Error when streaming frames: " << poRTProtocol.GetRTPacket()->GetErrorString() << endl;
                    exit(1);
                    break;
                case CRTPacket::PacketNoMoreData :  // No more data
                    cout << "No more data" << endl;
                    exit(0);
                    break;

                // Case 2 - Data received
                case CRTPacket::PacketData:
                    markerCount  = pRTPacket->GetForcePlateCount();

                    if (markerCount <= 0) {
                        cout << "No Plate Found" << endl;
//                        exit(0);
                    } else {
                    	cout << "Number of Plate found: " << markerCount << endl;
                    	 for(int i = 0; i < markerCount; i++)
						{
                    		forceCount = pRTPacket->GetForceCount(i);
                    		cout << "force count : " << forceCount << endl;
							CRTPacket::SForce force;
							for(int j = 0; j < forceCount; j++)
							{
								pRTPacket->GetForceData(i, j, force);
								cout << "Plate: " << i <<
										", f x: " << force.fForceX <<
										", f y: " << force.fForceY <<
										", f z: " << force.fForceZ <<
										", t x: " << force.fMomentX <<
										", t y: " << force.fMomentY <<
										", t z: " << force.fMomentZ <<
										", p x: " << force.fApplicationPointX <<
										", p y: " << force.fApplicationPointY <<
										", p z: " << force.fApplicationPointZ <<endl;
							}
						}
                    }
                    break;

                default:
                    cout << "Unknown CRTPacket case" << endl;
            }

        }

        poRTProtocol.GetCurrentFrame(CRTProtocol::cComponent3d);

		if (poRTProtocol.ReceiveRTPacket(eType, true)) {
			switch (eType) {
				// Case 1 - sHeader.nType 0 indicates an error
				case CRTPacket::PacketError :
					cout << "Error when streaming frames: " << poRTProtocol.GetRTPacket()->GetErrorString() << endl;
					exit(1);
					break;
				case CRTPacket::PacketNoMoreData :  // No more data
					cout << "No more data" << endl;
					exit(0);
					break;

				// Case 2 - Data received
				case CRTPacket::PacketData:
					markerCount  = pRTPacket->Get3DMarkerCount();
					if (markerCount <= 0) {
						cout << "No marker Found" << endl;
						exit(0);
					} else {
						cout << "Number of marker found: " << markerCount << endl;
						for (int i = 0; i < markerCount; i++) {
							pRTPacket->Get3DMarker(i, markerX, markerY, markerZ);
//							if (isnan(markerX)||isnan(markerY)||isnan(markerZ)) {
//								cout << "Marker " << i+1 << "/" << markerCount << " not detected" << endl;
//								exit(0);
//							}
							cout << "Marker: " << i << ", X: " << markerX << ", Y: " << markerY << ", Z: " << markerZ << endl;
						}
					}
					break;
			default:
				cout << "Unknown CRTPacket case" << endl;
			}
		}

    }

    poRTProtocol.StreamFramesStop();
    poRTProtocol.Disconnect();
    return 1;
}

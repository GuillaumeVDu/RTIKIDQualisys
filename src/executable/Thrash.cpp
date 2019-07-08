#include "InvKinInvDynRealTime.h"
#include "OpenSimData.h"
#include "XMLInterpreter.h"
#include "InvKinMarker.h"
#include "InvDyn.h"

#include <boost/timer/timer.hpp>

#include <OpenSim/OpenSim.h>

int main(int argc, char **argv) {
	try {
		string modelName(argv[1]);
		string xmlName(argv[2]);
		string trcName(argv[3]);
		OpenSimData openSimData(modelName, true);

		boost::shared_ptr<XMLInterpreter> xmlInterpreter(
				new XMLInterpreter(xmlName));
		xmlInterpreter->readXML();

		InvKinMarker ikMarker(openSimData.getState(), openSimData.getModel());
		ikMarker.init(xmlInterpreter);

		InvDyn invDyn(openSimData.getState(), openSimData.getModel());

		OpenSim::MarkersReference markerRef(trcName);
		const SimTK::Array_<std::string>& markerNames = markerRef.getNames();
		SimTK::Vec2 markersValidTimRange = markerRef.getValidTimeRange();
		double start_time = markersValidTimRange[0];
		double final_time = markersValidTimRange[1];
		double dt = 1.0 / markerRef.getSamplingFrequency();
		openSimData.getState()->updTime() = start_time;
		int Nframes = int((final_time - start_time) / dt) + 1;

		cout << "Numbers of DOF: "
				<< openSimData.getModel()->getCoordinateSet().getSize() << endl;

		int index;
		for (int i = 0;
				i < openSimData.getModel()->getCoordinateSet().getSize(); i++) {
			if (openSimData.getModel()->getCoordinateSet().get(i).getName()
					== "pelvis_tilt") {
				index = i;
				break;
			}
		}
		boost::timer::auto_cpu_timer t;
		for (int i = 0; i < Nframes; i++) {
			SimTK::Array_<SimTK::Vec3> values;
			markerRef.getValues(*(openSimData.getState()), values);
			std::map<string, std::vector<double> > markerToPosition;
			for (int j = 0; j < markerNames.size(); j++) {
				std::vector<double> markerPos;
				markerPos.push_back(values[j][0]);
				markerPos.push_back(values[j][1]);
				markerPos.push_back(values[j][2]);
				markerToPosition[markerNames[j]] = markerPos;
			}
			openSimData.getState()->updTime() = start_time + i * dt;
			ikMarker.computeKinematics(markerToPosition);
			SimTK::Vector vect = invDyn.computeTorque();
//			cout << "Angle: "
//					<< SimTK::convertRadiansToDegrees(
//							ikMarker.getAngle("pelvis_tilt"))
//					<< "  Torque: " << vect[index] << endl;
			openSimData.visualizerShow();
		}
		cout << "Numbers of frames: " << Nframes << endl;
	} catch (const std::exception &ex) {
		cout << "Error: " << ex.what() << endl;
		return 1;
	}

	cout << "OpenSim environment simulation completed successfully." << endl;
	return 0;
}

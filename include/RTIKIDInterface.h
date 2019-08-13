// Copyright (c) 2015, Guillaume Durandau and Massimo Sartori
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "InvKinInvDynRealTime.h"
#include "XMLInterpreterV2.h"
#include "InvKinMarker.h"
#include "InvKinImu.h"
#include "InvDyn.h"
#include <boost/timer/timer.hpp>
#include <OpenSim/OpenSim.h>
#include "KalmanFilterSimTK.h"

#ifndef RTIKIDINTERFACE_H_
#define RTIKIDINTERFACE_H_

/**
 * Class interface for the IK, the ID and the Kalman filter.
 */

class RTIKIDInterface
{
public:

	/**
	 * Constructor
	 */
	RTIKIDInterface(const string& xmlName);

	/**
	 * Destructor
	 */
	virtual ~RTIKIDInterface();

	/**
	 * Get the computed angles.
	 * @param dofName Name of the joint
	 */
	double getAngle(const string& dofName);

	/**
	 * Get the computed velocity.
	 * @param dofName Name of the joint
	 */
	double getVelocity(const string& dofName);

	/**
	 * Get the computed Acceleration.
	 * @param dofName Name of the joint
	 */
	double getAcceleration(const string& dofName);

	/**
	 * Compute the torque.
	 * @return Simtk vector with the torque for all joints (joints order of the model).
	 */
	SimTK::Vector computeTorqueSimTK();

	/**
	 * Compute the torque.
	 * @param force The ground reaction force (fx, fy, fz).
	 * @param positionGround The position of the ground reaction force (x, y, z).
	 * @param torque The ground reaction torque (tx, ty, tz).
	 * @param appliedBodyName The name of the body on where the forces are applied.
	 * @return Simtk vector with the torque for all joints (joints order of the model).
	 */
	SimTK::Vector computeTorqueSimTK(const SimTK::Array_<SimTK::fVec9>& grf,
			const std::vector<std::string>& appliedBodyName);

	std::vector<double> computeTorqueStd(const SimTK::Array_<SimTK::fVec9>& grf,
			const std::vector<std::string>& appliedBodyName);

	/**
	 * Compute the torque.
	 * @return std vector with the torque for all joints (joints order of the model).
	 */
	std::vector<double> computeTorqueStd();

	/**
	 * Set time in the OpenSim State.
	 */
	inline void setTime(double time)
	{
		ptr_si_->updTime() = time;
	}

	/**
	 * Get the name of the joints.
	 * @return std Vector with the names of the joints in the OpenSim model order.
	 */
	inline std::vector<string> getDofNames() const
	{
		return dofNames_;
	}

	/**
	 * Get the name of the muscles.
	 * @return std Vector with the name of the muscles in the OpenSim model order.
	 */
	inline std::vector<string> getMuscleNames() const
	{
		return muscleNames_;
	}

	/**
	 * Get the OpenSim state
	 */
	inline const boost::shared_ptr<SimTK::State> getState() const
	{
		return ptr_si_;
	}

	/**
	 * Get the OpenSim model
	 */
	inline const boost::shared_ptr<OpenSim::Model> getModel() const
	{
		return ptr_model_;
	}

	/**
	 * Get the OpenSim state with write access
	 */
	inline boost::shared_ptr<SimTK::State> updState()
	{
		return ptr_si_;
	}

	/**
	 * Get the OpenSim model with write access
	 */
	inline boost::shared_ptr<OpenSim::Model> updModel()
	{
		return ptr_model_;
	}

	/**
	 * Compute the IK for marker
	 * @param markerGoal An array of marker position
	 */
	inline void runIKMarker(SimTK::Array_<SimTK::Vec3> markerGoal)
	{
		dynamic_cast<InvKinMarker*>(ik_.get())->computeKinematics(markerGoal);
	}

	/**
	 * Compute the IK for IMU
	 * @param markerGoal An array of IMU quaternion
	 */
	inline void runIMUMarker(SimTK::Array_<SimTK::dQuaternion> markerGoal)
	{
		dynamic_cast<InvKinImu*>(ik_.get())->computeKinematics(markerGoal);
	}

	/**
	 * Get the time between two frames for the Kalman filter (for timing purpose)
	 */
	inline double getDt()
	{
		return xmlInterpreter_->getDt();
	}

	/**
	 * Update the model position in the OpenSim Visualizer
	 */
	inline void show()
	{
// 		if (xmlInterpreter_->getVisualizerUse())
// 			ptr_model_->updVisualizer().show(*ptr_si_);
	}

	/**
	 * Get the name of the XML lab file
	 */
	inline const std::string& getLabFile()
	{
		return xmlInterpreter_->getLabFile();
	}

	/**
	 * Get the name of the OpenSim model file
	 */
	inline const std::string& getOsimFile()
	{
		return xmlInterpreter_->getOsimFile();
	}

	/**
	 * Get the name of the translation file
	 */
	inline const std::string& getTranslateFile()
	{
		return xmlInterpreter_->getTranslateFile();
	}

	// Get the IP of the Qualisys server
	inline const std::string& getIP()
	{
		return xmlInterpreter_->getIP();
	}

	/**
	 * Get the port of the Qualisys software
	 */
	inline const int& getPort()
	{
		return xmlInterpreter_->getPort();
	}

	/**
	 * Get the name of the body on which the ground reaction force is applied
	 */
	inline const std::vector<string>& getAppliedBody()
	{
		return xmlInterpreter_->getAppliedBody();
	}
	
	inline const boost::shared_ptr<XMLInterpreter>& getInterpreter()
	{
		return xmlInterpreter_;
	}

	/**
	 * Compute the inverse kinematic
	 * @param values A map mapping the name of the IMU or markers to the quaternion/position
	 */
	void runIK(const std::map<string, std::vector<double> >& values);

	/**
	 * Compute the Kalman filter for filtering the position and computing the velocity and acceleration
	 */
	void computeKalmanFilter();
	
	inline const std::vector<string>& getMarkersNames() const
	{
		return xmlInterpreter_->getMarkersNames();
	}

protected:

	boost::shared_ptr<XMLInterpreter> xmlInterpreter_;
	boost::shared_ptr<SimTK::State> ptr_si_; //!< OpenSim state
	boost::shared_ptr<OpenSim::Model> ptr_model_; //!< OpenSim model
	boost::shared_ptr<InvKin> ik_; //!<
	boost::shared_ptr<InvDyn> id_; //!<
	std::vector<string> dofNames_; //!<
	std::vector<string> muscleNames_; //!<
	std::vector<boost::shared_ptr<KalmanFilterSimTK> > kalmanVect_; //!< Vector of Kalman filter for each joint
	std::map<string, double> qddot_; //!< Map mapping name of the joint to the acceleration
	SimTK::Vector qddotSimTK_; //!< SimTK vector of acceleration
};

#endif /* RTIKIDINTERFACE_H_ */

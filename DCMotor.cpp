/* -------------------------------------------------------------------------- *
*                           DCMotor.cpp                                *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2012 Stanford University and the Authors                *
*                                                                            *
*                                                                            *
* Licensed under the Apache License, Version 2.0 (the "License"); you may    *
* not use this file except in compliance with the License. You may obtain a  *
* copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
*                                                                            *
* Unless required by applicable law or agreed to in writing, software        *
* distributed under the License is distributed on an "AS IS" BASIS,          *
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
* See the License for the specific language governing permissions and        *
* limitations under the License.                                             *
* -------------------------------------------------------------------------- */

/* 
* Author: Vinh Q. Nguyen, Andrew Lapre, Mark Price - Umass Amherst
* Contact: vinhnguyen@umass.edu
* Version 1.0 (Nov 2016)
*/

//-----------------------------------------------------------------------------
// INCLUDES
//-----------------------------------------------------------------------------
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "DCMotor.h"
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace std;
using std::string;
using SimTK::Vec3; using SimTK::Vector_; using SimTK::Vector;
using SimTK::SpatialVec; using SimTK::UnitVec3; using SimTK::State;


//-----------------------------------------------------------------------------
// CONSTRUCTOR(S) AND DESTRUCTOR
//-----------------------------------------------------------------------------

/* Destructor. */
DCMotor::~DCMotor()
{
}

/* Default constructor. */
DCMotor::DCMotor()
{
	constructProperties();
}

// Constructor with given body names.
//DCMotor::DCMotor(const Body& Rotor, const Body& Stator,
//	const SimTK::Vec3& axis, bool axisInGround)
//{
//	constructProperties();
//
//	setRotor(Rotor);
//	setStator(Stator);
//
//	set_axis(axis);
//	set_axis_is_global(axisInGround);
//}


//-----------------------------------------------------------------------------
// CONSTRUCTION
//-----------------------------------------------------------------------------

// Construct and initialize properties.
void DCMotor::constructProperties()
{
	setAuthors("Vinh Q Nguyen, Andrew Lapre, Mark Price");
	constructProperty_rotor();
	constructProperty_stator();
	constructProperty_axis_is_global(true);
	constructProperty_shaft_axis(Vec3(0, 0, 1)); // z direction
	constructProperty_optimal_force(1.0);
	constructProperty_voltageRatio(1);

	constructProperty_K(0.0);
	constructProperty_L(0.0);
	constructProperty_R(0.0);
	constructProperty_b(0.0);
	constructProperty_maximum_current(10.0);
	constructProperty_initial_current(0.0);
}


//-----------------------------------------------------------------------------
// GET AND SET
//-----------------------------------------------------------------------------

/* Set Rotor body. */
void DCMotor::setRotor(const Body& aBody)
{
	_rotor = &aBody;
	set_rotor(aBody.getName());
}

/* Set Stator body. */
void DCMotor::setStator(const Body& aBody)
{
	_stator = &aBody;
	set_stator(aBody.getName());
}

/* Motor torque constant. */
double DCMotor::getK() const
{
	return (get_K());
}

void DCMotor::setK(double K)
{
	set_K(K);
}

/* Motor inductance. */
double DCMotor::getL() const
{
	return (get_L());
}

void DCMotor::setL(double L)
{
	set_L(L);
}

/* Motor resistance. */
double DCMotor::getR() const
{
	return (get_R());
}

void DCMotor::setR(double R)
{
	set_R(R);
}

/* Rotor damping. */
double DCMotor::getRotorDamping() const
{
	return (get_b());
}

void DCMotor::setRotorDamping(double b)
{
	set_b(b);
}

/* Maximum current supplied. */
double DCMotor::getMaximumCurrent() const
{
	return (get_maximum_current());
}

void DCMotor::setMaximumCurrent(double maximum_current)
{
	set_maximum_current(maximum_current);
}


//-----------------------------------------------------------------------------
// COMPUTATIONS
//-----------------------------------------------------------------------------

/* Compute EMF torque magnitude of motor. */
double DCMotor::computeActuation(const State& s) const
{
	if (_model == NULL) return 0;

	// Torque
	double torque;
	torque = getK() * getCurrent(s);

	return (torque);
}

/* Compute motor stress. */
double DCMotor::getStress(const SimTK::State& s) const
{
	return getForce(s) / (getK() * getMaximumCurrent());
}

//-----------------------------------------------------------------------------
// APPLICATION
//-----------------------------------------------------------------------------

/* Apply the actuator torque to Rotor and Stator. */
void DCMotor::computeForce(const State& s,
	Vector_<SpatialVec>& bodyForces,
	Vector& generalizedForces) const
{
	if (_model == NULL) return;
	const SimbodyEngine& engine = getModel().getSimbodyEngine();

	const bool torqueIsGlobal = getAxisIsGlobal();
	const Vec3& axis = getShaftAxis();

	double force = 0;

	// find the speed of rotor respectively to stator
	SimTK::Vec3 omegaA(0), omegaB(0);
	engine.getAngularVelocity(s, *_rotor, omegaA);
	engine.getAngularVelocity(s, *_stator, omegaB);

	double speed = ~(omegaA - omegaB)*UnitVec3(axis);
	// set speed of motor in state s
	setSpeed(s, speed);

	if (isForceOverriden(s)) {
		force = computeOverrideForce(s);
	}
	else {
		force = computeActuation(s) - getRotorDamping() * speed;
	}
	setForce(s, force);

	if (!_rotor)
		return;

	setForce(s, force);
	Vec3 torque = force*UnitVec3(axis);

	if (!torqueIsGlobal)
		engine.transform(s, *_rotor, torque, engine.getGroundBody(), torque);

	applyTorque(s, *_rotor, torque, bodyForces);

	// if bodyB is not specified, use the ground body by default
	if (_stator)
		applyTorque(s, *_stator, -torque, bodyForces);
}

/* Sets the actual Body references _rotor and _stator */
void DCMotor::connectToModel(Model& model)
{
	Super::connectToModel(model);

	if (get_rotor().empty() || get_stator().empty())
		throw OpenSim::Exception(
			"DCMotor::connectToModel(): body name properties "
			"were not set.");

	// Look up the bodies by name in the Model, and record pointers to the
	// corresponding body objects.
	_rotor = model.updBodySet().get(get_rotor());
	_stator = model.updBodySet().get(get_stator());
}


//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------

/* Update this object based on its XML node.
*
* This method simply calls Object::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) and then calls
* a few methods in this class to ensure that variable members have been
* set in a consistent manner.
*/
void DCMotor::
updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	int documentVersion = versionNumber;
	bool converting = false;
	if (documentVersion < XMLDocument::getLatestVersion()) {
		if (documentVersion<10905) {
			// This used to be called "Force" back then
			XMLDocument::renameChildNode(aNode, "Rotor", "rotor"); 
			XMLDocument::renameChildNode(aNode, "Stator", "stator");
			XMLDocument::renameChildNode(aNode, "direction_A", "axis"); 
			converting = true;
		}
	}
	Super::updateFromXMLNode(aNode, versionNumber);
	if (converting) upd_shaft_axis() *= -1.0;
}


//-----------------------------------------------------------------------------
// IMPLEMENT MODELCOMPONENT INTERFACE
//-----------------------------------------------------------------------------

void DCMotor::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);

	// Add state variables
	addStateVariable("current");

	addCacheVariable("current_deriv", 0.0, SimTK::Stage::Dynamics);
}

void DCMotor::initStateFromProperties(SimTK::State& state) const
{
	setStateVariable(state, "current", get_initial_current());
}

void DCMotor::setPropertiesFromState(const SimTK::State& state)
{
	set_initial_current(getCurrent(state));
}


//-----------------------------------------------------------------------------
// GET AND SET STATE VARIABLES
//-----------------------------------------------------------------------------
double DCMotor::getCurrent(const SimTK::State& s) const
{
	return getStateVariable(s, "current");
}

void DCMotor::setCurrent(SimTK::State& s, double current) const
{
	setStateVariable(s, "current", current);
}

void DCMotor::setCurrentDeriv(const SimTK::State& s,
	double activeMotorUnitsDeriv) const
{
	setStateVariableDeriv(s, "current", activeMotorUnitsDeriv);
}

double DCMotor::getCurrentDeriv(const SimTK::State& s) const
{
	return getStateVariableDeriv(s, "current");
}

double DCMotor::
getStateVariableDeriv(const SimTK::State& s,
	const std::string &aStateName) const
{
	return getCacheVariable<double>(s, aStateName + "_deriv");
}

void DCMotor::
setStateVariableDeriv(const SimTK::State& s,
	const std::string &aStateName, double aValue) const
{
	double& cacheVariable = updCacheVariable<double>(s, aStateName + "_deriv");
	cacheVariable = aValue;
	markCacheVariableValid(s, aStateName + "_deriv");
}


//-----------------------------------------------------------------------------
// COMPUTE THE DERIVATIES OF STATE VARIABLES
//-----------------------------------------------------------------------------
SimTK::Vector DCMotor::
computeStateVariableDerivatives(const SimTK::State& s) const
{
	SimTK::Vector derivs(1, 0.0);

	double i_dot = 0.0; // rate of change of current
	double speed = 0.0;

	//get the speed of rotor
	speed = getSpeed(s);

	// equation of current derivative
	i_dot = (getControl(s)* getVoltageRatio() - getK() * speed - getR()*getCurrent(s)) / (getL());

	// check if the current is over the limit of the supplied source
	double i = getCurrent(s);
	if ((i >= get_maximum_current() && i_dot > 0) || (i<= -get_maximum_current() && i_dot < 0)) i_dot = 0;

	derivs[0] = i_dot;

	setCurrentDeriv(s, i_dot);

	return derivs;
}
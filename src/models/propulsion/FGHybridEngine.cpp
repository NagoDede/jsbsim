/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       FGHybridEngine.cpp
 Author:       V. Detroyat
 Date started: 01/10/20222
 Called by:    FGAircraft

 ------------- Copyright (C) 1999  Jon S. Berndt (jon@jsbsim.org) -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation; either version 2 of the License, or (at your option) any
 later version.

 This program is distributed in the hope that it will be useful, but WITHOUT ANY
 WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along
 with this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA 02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be
 found on the world wide web at http://www.gnu.org.

FUNCTIONAL DESCRIPTION
--------------------------------------------------------------------------------
See header file.

It provides an interface for an hybride engine.

HISTORY
--------------------------------------------------------------------------------


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include <iostream>
#include <sstream>

#include "FGFDMExec.h"
#include "FGHybridEngine.h"

#include "FGPropeller.h"
#include "input_output/FGXMLElement.h"

#include <string>
#include <stdexcept>

using namespace std;

namespace JSBSim {

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS IMPLEMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

FGHybridEngine::FGHybridEngine(FGFDMExec* exec, Element *el, int engine_number, FGEngine::Inputs& input)
  : FGEngine(engine_number, input)
{
      debug_lvl = 4;
      
      Load(exec,el);
      engineNumber = engine_number;

      PropertyManager = exec->GetPropertyManager();

        Type = etHybrid;

        cout << "\n    Hybrid Engine: " << Name << endl;

        base_property_name = CreateIndexedPropertyName(PropertyPath, engine_number);

        if (el->FindElement("piston_engine"))
        {
            Element *element = el->FindElement("piston_engine");

            ostringstream buf;
            buf << base_property_name << "/ice";
            enginesCnt++;
            iceEngineNumber = engineNumber + enginesCnt;
            pistonEngine = new FGPiston(exec, element, iceEngineNumber, in, Thruster, buf.str() );

            cout << "\n     - Piston Engine Name: " << pistonEngine->GetName() << endl;

        } else
        {
            cerr << el->ReadFrom() << " No piston_engine defined" << endl;
        }

        if (el->FindElement("electric_engine"))
        {
            Element *element = el->FindElement("electric_engine");

            ostringstream buf;
            buf << base_property_name << "/electric";
            enginesCnt++;
            elecEngineNumber = engineNumber + enginesCnt;
            elecEngine = new FGElectric(exec, element, elecEngineNumber, in,  Thruster, buf.str());
            cout << "\n     - Electric Engine Name: " << elecEngine->GetName() << endl;
        } else
        {
            cerr << el->ReadFrom() << " No electric defined" << endl;
        }

        if (el->FindElement("transmission"))
        {
          Element* elt = el->FindElement("transmission");
          LoadTransmission(exec,elt);
        } else
          cerr << el->ReadFrom() << " No electric defined" << endl;




        LoadElement(el);

        elecEngine->SetThruster(elecTransmission);
        pistonEngine->SetThruster(iceTransmission);

        SetPropertyTree(exec);

        Debug(1); // Call Debug() routine from constructor if needed

        debug_lvl = 1;
}

unsigned int FGHybridEngine::AddEnginesToPropulsionEngines(std::vector <FGEngine*>* Engines)
{
  //Organise the engines in order to have the ICE before the electrical engine
  if (iceEngineNumber > elecEngineNumber) {
    unsigned int tp = iceEngineNumber;
    iceEngineNumber = elecEngineNumber;
    elecEngineNumber = tp;
  }

  //Push the engines set in the Hybrid engine in the engines list.
  Engines->push_back(pistonEngine);
  Engines->push_back(elecEngine);

  return enginesCnt;
}

bool FGHybridEngine::LoadElement(Element* el) {
  MaxPower = ConfigValueConv(el, "maxpower", 340000, "W", true);
  MaxThrottle = ConfigValue(el, "maxthrottle", 1.0,false);
  MinThrottle = ConfigValue(el, "minthrottle", 1.0, false);
  return true;
}

double FGHybridEngine::ConfigValueConv(Element* el, const string& ename, double default_val,
  const string& unit, bool tell)
{

  Element* e = NULL;
  double val = default_val;

  string pname = "*No parent element*";

  if (el) {
    e = el->FindElement(ename);
    pname = el->GetName();
  }

  if (e) {
    if (unit.empty()) {
      val = e->GetDataAsNumber();
    }
    else {
      val = el->FindElementValueAsNumberConvertTo(ename, unit);
    }
  }
  else {
    if (tell) {
      cerr << pname << ": missing element '" << ename <<
        "' using estimated value: " << default_val << endl;
    }
  }

  return val;
}

/*
Load hybrid transmission data
*/
bool FGHybridEngine::LoadTransmission(FGFDMExec* exec, Element* el)
{
  if (el->FindElement("isparallel")) {
    isParallel = el->FindElementValueAsBoolean("isparallel");
  }
  else
    cout << "\n     isParallel not defined. Default is Serie. " << endl;

  if (el->FindElement("ice_side"))
  {
    Element* iceElt = el->FindElement("ice_side");
    ostringstream buf;
    buf << base_property_name << "/ice";
    iceTransmission = new FGHybridTransmission(exec, iceElt, 1, buf.str());
  }
  else
  {
    cerr << el->ReadFrom() << " No transmission defined on ICE side" << endl;
    return false;
  }

  if (el->FindElement("elec_side"))
  {
    Element* elecElt = el->FindElement("elec_side");
    ostringstream buf;
    buf << base_property_name << "/electric";
    elecTransmission = new FGHybridTransmission(exec, elecElt, 1, buf.str());
  }
  else
  {
    cerr << el->ReadFrom() << " No transmission defined on ELEC side" << endl;
    return false;
  }
  return true;  
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FGHybridEngine::~FGHybridEngine()
{
  Debug(1); // Call Debug() routine from constructor if needed
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGHybridEngine::SetPropertyTree(FGFDMExec* exec)
{

       string property_name;

        property_name = base_property_name + "/power-watt-cmd";
        PropertyManager->Tie(property_name.c_str(), this,
          &FGHybridEngine::GetPowerCmd, &FGHybridEngine::SetPowerCmd);

        property_name = base_property_name + "/h-factor-cmd";
        PropertyManager->Tie(property_name.c_str(), this,
          &FGHybridEngine::GetHFactorCmd, &FGHybridEngine::SetHFactorCmd);


        property_name = base_property_name + "/maxpower-watt";
        PropertyManager->Tie(property_name, &MaxPower);

        property_name = base_property_name + "/power-watt";
        PropertyManager->Tie(property_name, &OutputPower);


        property_name = base_property_name + "/h-factor";
        PropertyManager->Tie(property_name, &HFactor);

}


void FGHybridEngine::Calculate(void)
{
  RunPreFunctions();

  if (Thruster->GetType() == FGThruster::ttPropeller) {
    ((FGPropeller*)Thruster)->SetAdvance(in.PropAdvance[EngineNumber]);
    ((FGPropeller*)Thruster)->SetFeather(in.PropFeather[EngineNumber]);
  }

  RPM = Thruster->GetEngineRPM();

  OutputPower = MaxPower * in.ThrottlePos[EngineNumber];

  

  LoadThrusterInputs();

  // Filters out negative powers when the propeller is not rotating.
  double power = OutputPower * wtoftlbssec;
  if (RPM <= 0.1) power = max(power, 0.0);
  Thruster->Calculate(power);

  iceTransmission->SetThrusterRPM(RPM);
  elecTransmission->SetThrusterRPM(RPM);


  RunPostFunctions();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

double FGHybridEngine::CalcFuelNeed(void)
{
  return 0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string FGHybridEngine::GetEngineLabels(const string& delimiter)
{
  std::ostringstream buf;

  buf << Name << " HP (engine " << EngineNumber << ")" << delimiter
      << Thruster->GetThrusterLabels(EngineNumber, delimiter);

  return buf.str();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

string FGHybridEngine::GetEngineValues(const string& delimiter)
{
  std::ostringstream buf;

  buf << OutputPower << delimiter
     << Thruster->GetThrusterValues(EngineNumber, delimiter);

  return buf.str();
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//
//    The bitmasked value choices are as follows:
//    unset: In this case (the default) JSBSim would only print
//       out the normally expected messages, essentially echoing
//       the config files as they are read. If the environment
//       variable is not set, debug_lvl is set to 1 internally
//    0: This requests JSBSim not to output any messages
//       whatsoever.
//    1: This value explicity requests the normal JSBSim
//       startup messages
//    2: This value asks for a message to be printed out when
//       a class is instantiated
//    4: When this value is set, a message is displayed when a
//       FGModel object executes its Run() method
//    8: When this value is set, various runtime state variables
//       are printed out periodically
//    16: When set various parameters are sanity checked and
//       a message is printed out when they go out of bounds

void FGHybridEngine::Debug(int from)
{
  if (debug_lvl <= 0) return;

  if (debug_lvl & 1) { // Standard console startup message output
    if (from == 0) { // Constructor

      cout << "\n    Engine Name: "         << Name << endl;
      cout << "      Power Watts: "         << MaxPower << endl;

    }
  }
  if (debug_lvl & 2 ) { // Instantiation/Destruction notification
    if (from == 0) cout << "Instantiated: FGElectric" << endl;
    if (from == 1) cout << "Destroyed:    FGElectric" << endl;
  }
  if (debug_lvl & 4 ) { // Run() method entry print for FGModel-derived objects
  }
  if (debug_lvl & 8 ) { // Runtime state variables
  }
  if (debug_lvl & 16) { // Sanity checking

  }
  if (debug_lvl & 64) {
    if (from == 0) { // Constructor
    }
  }
}

} // namespace JSBSim

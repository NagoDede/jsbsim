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

        Type = etHybrid;

        cout << "\n    Hybrid Engine: " << Name << endl;

        base_property_name = CreateIndexedPropertyName(PropertyPath, engine_number);

        if (el->FindElement("piston_engine"))
        {
            Element *element = el->FindElement("piston_engine");

            ostringstream buf;
            buf << base_property_name << "/piston";
            pistonEngine = new FGPiston(exec, element, engine_number, in, Thruster, buf.str() );
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
            elecEngine = new FGElectric(exec, element, engine_number, in,  Thruster, buf.str());
            cout << "\n     - Electric Engine Name: " << elecEngine->GetName() << endl;
        } else
        {
            cerr << el->ReadFrom() << " No electric defined" << endl;
        }

        if (el->FindElement("maxhp"))
            MaxHP = el->FindElementValueAsNumberConvertTo("maxhp","HP");
        if (el->FindElement("maxthrottle"))
            MaxThrottle = el->FindElementValueAsNumber("maxthrottle");
        if (el->FindElement("minthrottle"))
            MinThrottle = el->FindElementValueAsNumber("minthrottle");

        SetPropertyTree(exec);

        Debug(1); // Call Debug() routine from constructor if needed

        debug_lvl = 1;
}


//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

FGHybridEngine::~FGHybridEngine()
{
  Debug(1); // Call Debug() routine from constructor if needed
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void FGHybridEngine::SetPropertyTree(FGFDMExec* exec)
{
        FGPropertyManager* PropertyManager = exec->GetPropertyManager();
        std::string property_name;
        property_name = base_property_name + "/power-hp";
        PropertyManager->Tie(property_name, &HP);
        property_name = base_property_name + "/maxhp";
        PropertyManager->Tie(property_name, &MaxHP);
        property_name = base_property_name + "/h-factor";
        PropertyManager->Tie(property_name, &HFactor);
        property_name = base_property_name + "/h-factor-cmd";
        PropertyManager->Tie(property_name, &HFactorCmd);
}


void FGHybridEngine::Calculate(void)
{
  RunPreFunctions();

  if (Thruster->GetType() == FGThruster::ttPropeller) {
    ((FGPropeller*)Thruster)->SetAdvance(in.PropAdvance[EngineNumber]);
    ((FGPropeller*)Thruster)->SetFeather(in.PropFeather[EngineNumber]);
  }

  RPM = Thruster->GetRPM() * Thruster->GetGearRatio();

  HP = PowerWatts * in.ThrottlePos[EngineNumber] / hptowatts;

  LoadThrusterInputs();
  // Filters out negative powers when the propeller is not rotating.
  double power = HP * hptoftlbssec;
  if (RPM <= 0.1) power = max(power, 0.0);
  Thruster->Calculate(power);

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

  buf << HP << delimiter
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
      cout << "      Power Watts: "         << MaxHP << endl;

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

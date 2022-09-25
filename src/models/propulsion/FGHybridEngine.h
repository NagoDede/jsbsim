/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       FGEngine.h
 Author:       Jon S. Berndt
 Date started: 01/21/99

 ------------- Copyright (C) 1999  Jon S. Berndt (jon@jsbsim.org) -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 Place - Suite 330, Boston, MA  02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be found on
 the world wide web at http://www.gnu.org.

FUNCTIONAL DESCRIPTION
--------------------------------------------------------------------------------

Based on Flightgear code, which is based on LaRCSim. This class simulates
a generic engine.

HISTORY
--------------------------------------------------------------------------------
01/21/99   JSB   Created

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SENTRY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef FGHYBRIDENGINE_H
#define FGHYBRIDENGINE_H

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "FGEngine.h"
#include "FGPiston.h"
#include "FGElectric.h"
#include "FGHybridTransmission.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FORWARD DECLARATIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

namespace JSBSim {

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DOCUMENTATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/** Base class for all engines.
    This base class contains methods and members common to all engines, such as
    logic to drain fuel from the appropriate tank, etc.
    <br>
    <h3>Configuration File Format:</h3>
@code
    <hybrid_engine>
    <hybrid_engine>
@endcode
<pre>
    NOTES:

  Not all thruster types can be matched with a given engine type.  See the class
  documentation for engine and thruster classes.
</pre>
    @author Jon S. Berndt
*/

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CLASS DECLARATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

class FGHybridEngine : public FGEngine
{public:
  /// Constructorma
  FGHybridEngine(FGFDMExec* exec, Element *el, int engine_number, FGEngine::Inputs& input);

  /// Destructor
  ~FGHybridEngine();

    void Calculate(void);
    double CalcFuelNeed(void);

    FGPiston* GetPistonEngine(void) {return pistonEngine;};
    FGElectric* GetElectricEngine(void) {return elecEngine;};

    double GetPowerAvailable(void) {return (HP * hptoftlbssec);}
    double getRPM(void) {return RPM;}

  std::string GetEngineLabels(const std::string& delimiter);
  std::string GetEngineValues(const std::string& delimiter);

private:
    FGPiston* pistonEngine;
    FGElectric* elecEngine;

    FGHybridTransmission *iceTransmission;
    FGHybridTransmission *elecTransmission;

  // constants
  double hptowatts;

  double PowerWatts;         // maximum engine power
  double RPM = 00;                // revolutions per minute
  double HP = 0.0;                 // engine output, in horsepower

  double MaxHP = 0.0;             //Maximum HP of the hybrid engine. Some of the RE and EE Max power

  double MaxThrottle = 1.0;       //full forward position
  double MinThrottle = 0.0;       //Idle

  double HFactorCmd = 0.5;
  double HFactor = 0.0;

  //Transmission parameters
  bool isParallel = false;
  //ICE transmission parameters
  bool iceFreewheel = false; //tied to   < ice_freewheel>1 < / ice_freewheel >
  bool iceClutch = false; // tied   < ice_clutch>0 < / ice_clutch >
  double iceRatio = 1.0; //tied to     < ice_ratio>0.46 < / ice_ratio >

  //Elec transmission parameters
  bool elecFreewheel = false; // < elec_freewheel>1 < / elec_freewheel >
  bool elecClutch = false; //  < elec_clutch>0 < / elec_clutch >
  double elecRatio = 1.0; // < elec_ratio>0.86 < / elec_ratio >


  std::string base_property_name = "";
  void SetPropertyTree(FGFDMExec* exec);
  void Debug(int from);
  bool LoadTransmission(FGFDMExec* exec, Element* el);

}; //end class
} //end namespace

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#endif

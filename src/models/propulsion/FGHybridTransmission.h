/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 JSBSim
 Author:       Jon S. Berndt
 Date started: 08/24/00
 ------------- Copyright (C) 2000  Jon S. Berndt (jon@jsbsim.org) -------------

 Header:       FGHybridTransmission
 Author:       Vincent DETROYAT
 Date started: 01/10/2022

 ------------- Copyright (C) 2022 V. Detroyat (t.kreitler@web.de) -------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free Software
 Foundation; either version 2 of the License, or (at your option) any later
 version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 59 Temple
 Place - Suite 330, Boston, MA  02111-1307, USA.

 Further information about the GNU Lesser General Public License can also be found on
 the world wide web at http://www.gnu.org.

HISTORY
--------------------------------------------------------------------------------
01/10/2022 VDT Creation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
SENTRY
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#ifndef FGHYBRIDTRANSMISSION_H
#define FGHYBRIDTRANSMISSION_H

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "FGJSBBase.h"
#include "FGThruster.h"
#include "input_output/FGXMLElement.h"
#include "FGFDMExec.h"
#include "input_output/FGPropertyManager.h"
#include <string>

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FORWARD DECLARATIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

namespace JSBSim {

  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  CLASS DOCUMENTATION
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  /** Utility class that handles power transmission in conjunction with Hybrid Engine.

    This class provides brake, clutch and free-wheel-unit (FWU) functionality on the 
    power axis set between the engine and the thruster.
    Also it is responsible for the RPM calculations.

    When the engine is off the brake could be used to slow/hold down a spinning
    thruster. The maximum brake power is defined in the transmission section of 
    the hybrid_engine config file.
    (Right now there is no checking if the input is in the [0..1] range.)

    The clutch operation is based on a heuristic approach. In the intermediate
    state the transfer is proportional to the clutch position. But equal RPM
    values are enforced on the thruster and rotor sides when approaching the
    closed state.

    The FWU inhibits that the rotor is driving the engine. To do so, the code
    just predicts the upcoming FWU state based on current torque conditions.

    Some engines won't work properly when the clutch is open. To keep them
    controllable some load must be provided on the engine side (EngineFriction,
    aka gear-loss). 

  <h3>Property tree</h3>

    The following properties are created (with x = your thruster number):
    <pre>
      propulsion/engine[x]/brake-ctrl-norm
      propulsion/engine[x]/free-wheel-transmission
      propulsion/engine[x]/clutch-ctrl-norm
    </pre>

  <h3>Notes</h3>

    <ul>
      <li> EngineFriction is assumed constant, so better orientate at low RPM
           values, because piston and turboprop engines don't 'like' high
           load at startup.</li>
      <li> The model doesn't support backward operation.</li>
      <li> And even worse, the torque calculations silently assume a minimal
           RPM value of approx. 1.</li>
    </ul>

  */


  /*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  CLASS DECLARATION
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

  class FGHybridTransmission : public FGThruster{

  public:
    FGHybridTransmission(FGFDMExec* exec, Element* elt, int num, std::string propertyPath ="propulsion/engine");

    /// Destructor for FGHybridTransmission
    ~FGHybridTransmission();

    void Calculate(double EnginePower, double ThrusterTorque, double dt);
    /// Retrieves the RPMs of the rotor.
    double GetRPM(void) const { return ThrusterRPM; }
    //normally not used, the engine or the propeller will fixe the RPM
    void SetRPM(double rpm) { ThrusterRPM = rpm; }
    
    double GetPowerRequired(void) { return 0.0; }



    void   SetEngineRPM(double x) { EngineRPM = x; }
    double GetEngineRPM() const { return EngineRPM; }

    void   SetThrusterRPM(double x) { ThrusterRPM = x; }
    double GetThrusterRPM() const { return ThrusterRPM; }

    void SetThruster(FGThruster* thr) { Thruster = thr; }

    virtual std::string GetThrusterLabels(int id, const std::string& delimeter);
    virtual std::string GetThrusterValues(int id, const std::string& delimeter);

    virtual void ResetToIC(void);


  private:

    //for JSBSIM integration
    std::string PropertyPath;
    FGPropertyManager* PropertyManager;
    std::string Name;
    double dt;


    bool Load(Element* elt);
    bool BindModel(int num);
    void Debug(int from);
    double Calculate(double EnginePower);

    //for modelisation
    bool HaveFreeWheel = false;
    bool HaveClutch = false;
    bool HaveBrake = false;
    double Ratio = 1.0;
    double GearLoss = 0.2;
    double Moment = 1.0; //Moment of inertia

    Filter FreeWheelLag;
    double FreeWheelTransmission; // state, 0: free, 1:locked

    double ThrusterMoment;
    double ThrusterTorque;
    double EngineMoment;   // estimated MOI of gear and engine, influences acceleration
    double EngineFriction; // estimated friction in gear and possibly engine

    double ClutchCtrlNorm;
    double BrakeCtrlNorm;
    double MaxBrakePower;

    double MaxPower;
    double EngineRPM;
    double ThrusterRPM;
    double CurrentPower; //Power available on the RE + EE at output shafts



    FGThruster *Thruster;


    inline double omega_to_rpm(double w) {
      return w * 9.54929658551372014613302580235; // omega/(2.0*PI) * 60.0
    }
    inline double rpm_to_omega(double r) {
      return r * 0.104719755119659774615421446109; // (rpm/60.0)*2.0*PI
    }
    double GetBrakeCtrlNorm() const { return BrakeCtrlNorm; }
    void   SetBrakeCtrlNorm(double x) { BrakeCtrlNorm = x; }
    double GetClutchCtrlNorm() const { return ClutchCtrlNorm; }
    void   SetClutchCtrlNorm(double x) { ClutchCtrlNorm = x; }
    double GetFreeWheelNorm() const { return FreeWheelTransmission; }
    
    double ConfigValueConv(Element* e, const std::string& ename, double default_val = 0.0,
      const std::string& unit = "", bool tell = false);

    double ConfigValue(Element* e, const std::string& ename, double default_val = 0.0,
      bool tell = false)
    {
      return ConfigValueConv(e, ename, default_val, "", tell);
    };



  };

}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
#endif


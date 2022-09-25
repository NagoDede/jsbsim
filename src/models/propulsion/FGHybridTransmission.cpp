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


/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
INCLUDES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include "FGHybridTransmission.h"

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FORWARD DECLARATIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

namespace JSBSim {
  FGHybridTransmission::FGHybridTransmission(FGFDMExec* exec, Element* elt, int num, std::string propertyPath)
    : FGThruster(exec, elt, num, propertyPath)
  {
    PropertyPath = propertyPath;
    PropertyManager = exec->GetPropertyManager();
    dt = in.TotalDeltaT;
    Load(elt);
    BindModel(num);
  }


  FGHybridTransmission::~FGHybridTransmission() {
    Debug(1);
  }


  double FGHybridTransmission::Calculate(double enginePower) {

    double coupling = 1.0, coupling_sq = 1.0;
    double fw_mult = 1.0;

    double d_omega = 0.0, engine_d_omega = 0.0, thruster_d_omega = 0.0; // relative changes

    double engine_omega = rpm_to_omega(EngineRPM);
    double safe_engine_omega = engine_omega < 1e-1 ? 1e-1 : engine_omega;
    double engine_torque = enginePower / safe_engine_omega;

    double thruster_omega = rpm_to_omega(Thruster->GetRPM());
    double safe_thruster_omega = thruster_omega < 1e-1 ? 1e-1 : thruster_omega;

    engine_torque -= EngineFriction / safe_engine_omega;
    ThrusterTorque += Constrain(0.0, BrakeCtrlNorm, 1.0) * MaxBrakePower / safe_thruster_omega;

    // would the FWU release ?
    engine_d_omega = engine_torque / EngineMoment * dt;
    thruster_d_omega = -ThrusterTorque / ThrusterMoment * dt;

    if (thruster_omega + thruster_d_omega > engine_omega + engine_d_omega) {
      // don't drive the engine
      FreeWheelTransmission = 0.0;
    }
    else {
      FreeWheelTransmission = 1.0;
    }

    fw_mult = FreeWheelLag.execute(FreeWheelTransmission);
    coupling = fw_mult * Constrain(0.0, ClutchCtrlNorm, 1.0);

    if (coupling < 0.999999) { // are the separate calculations needed ?
      // assume linear transfer 
      engine_d_omega =
        (engine_torque - ThrusterTorque * coupling) / (ThrusterMoment * coupling + EngineMoment) * dt;
      thruster_d_omega =
        (engine_torque * coupling - ThrusterTorque) / (ThrusterMoment + EngineMoment * coupling) * dt;

      EngineRPM += omega_to_rpm(engine_d_omega);
      ThrusterRPM += omega_to_rpm(thruster_d_omega);

      // simulate transit to static friction
      coupling_sq = coupling * coupling;
      EngineRPM = (1.0 - coupling_sq) * EngineRPM + coupling_sq * 0.02 * (49.0 * EngineRPM + ThrusterRPM);
      ThrusterRPM = (1.0 - coupling_sq) * ThrusterRPM + coupling_sq * 0.02 * (EngineRPM + 49.0 * ThrusterRPM);

      // enforce equal rpm
      if (fabs(EngineRPM - ThrusterRPM) < 1e-3) {
        EngineRPM = ThrusterRPM = 0.5 * (EngineRPM + ThrusterRPM);
      }
    }
    else {
      d_omega = (engine_torque - ThrusterTorque) / (ThrusterMoment + EngineMoment) * dt;
      EngineRPM = ThrusterRPM += omega_to_rpm(d_omega);
    }

    // nothing will turn backward
    if (EngineRPM < 0.0) EngineRPM = 0.0;
    if (ThrusterRPM < 0.0) ThrusterRPM = 0.0;

    EngineRPM = EngineRPM / Ratio;
    Thrust = Thruster->GetThrust();

    return 0.0;
  }
 
  bool FGHybridTransmission::Load(Element* el) {
    Name = el->GetAttributeValue("name");

    if (el->FindElement("freewheel"))
      HaveFreeWheel = el->FindElementValueAsBoolean("freewheel");
    else
      cout << "\n     freewheel not defined. Default is False (no freewheel). " << endl;

    if (el->FindElement("clutch"))
      HaveClutch = el->FindElementValueAsBoolean("clutch");
    else
      cout << "\n     clutch not defined. Default is False (no clutch). " << endl;

    if (el->FindElement("brake"))
      HaveBrake = el->FindElementValueAsBoolean("brake");
    else
      cout << "\n     brake not defined. Default is False (no brake). " << endl;

    Ratio = ConfigValue(el, "ratio", 1.0, true);
    GearLoss = ConfigValue(el, "gearloss", 0.02, true);
    Moment = ConfigValueConv(el, "moment", 1.0, "KG*M2", true);

    return true;
  }

  bool FGHybridTransmission::BindModel(int num)
  {
    string property_name, base_property_name;
    //base_property_name = CreateIndexedPropertyName(PropertyPath , num);
    base_property_name = PropertyPath;

    property_name = base_property_name + "/engine-rpm";
    PropertyManager->Tie(property_name.c_str(), this,
      &FGHybridTransmission::GetEngineRPM, &FGHybridTransmission::SetEngineRPM);

    property_name = base_property_name + "/thruster-rpm";
    PropertyManager->Tie(property_name.c_str(), this,
      &FGHybridTransmission::GetThrusterRPM, &FGHybridTransmission::SetThrusterRPM);


    if (HaveBrake)
    {
      property_name = base_property_name + "/brake-ctrl-norm";
      PropertyManager->Tie(property_name.c_str(), this,
        &FGHybridTransmission::GetBrakeCtrlNorm, &FGHybridTransmission::SetBrakeCtrlNorm);
    }

    if (HaveClutch) {
      property_name = base_property_name + "/clutch-ctrl-norm";
      PropertyManager->Tie(property_name.c_str(), this,
        &FGHybridTransmission::GetClutchCtrlNorm, &FGHybridTransmission::SetClutchCtrlNorm);
    }

    if (HaveFreeWheel)
    {
      property_name = base_property_name + "/free-wheel-norm";
      PropertyManager->Tie(property_name.c_str(), this,
        &FGHybridTransmission::GetFreeWheelNorm);
    }

    return true;
  }
  
  double FGHybridTransmission::ConfigValueConv(Element* el, const string& ename, double default_val,
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

  //%%%%1%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  string FGHybridTransmission::GetThrusterLabels(int id, const string& delimeter)
  {
    std::ostringstream buf;

    buf << Name << " Torque (engine " << id << ")" << delimeter;
    buf << Name << " RPM (engine " << id << ")";

    return buf.str();
  }

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  string FGHybridTransmission::GetThrusterValues(int id, const string& delimeter)
  {
    std::ostringstream buf;

    buf << ThrusterRPM;

    return buf.str();
  }

  void FGHybridTransmission::ResetToIC(void)
  {
    FGThruster::ResetToIC();
  }



  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

  void FGHybridTransmission::Debug(int from)
  {
    if (debug_lvl <= 0) return;

    if (debug_lvl & 1) { // Standard console startup message output
      if (from == 0) { // Constructor
      }
    }
    if (debug_lvl & 2) { // Instantiation/Destruction notification
      if (from == 0) cout << "Instantiated: FGTransmission" << endl;
      if (from == 1) cout << "Destroyed:    FGTransmission" << endl;
    }
    if (debug_lvl & 4) { // Run() method entry print for FGModel-derived objects
    }
    if (debug_lvl & 8) { // Runtime state variables
    }
    if (debug_lvl & 16) { // Sanity checking
    }
    if (debug_lvl & 64) {
      if (from == 0) { // Constructor
      }
    }
  }

}// end namespace
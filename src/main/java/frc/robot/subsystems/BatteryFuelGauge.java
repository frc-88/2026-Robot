// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.BattFuelGauge;
import com.playingwithfusion.BattFuelGauge.BatteryManufacturer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.StringPreferenceConstant;

public class BatteryFuelGauge extends SubsystemBase {
  /** Creates a new BatteryFuelGauge. */
  private final BattFuelGauge battFuelGauge = new BattFuelGauge(Constants.BATTERY_FUEL_GAUGE);

  private final StringPreferenceConstant batteryNickname =
      new StringPreferenceConstant("Battery Gauge/New Nickname", "Untitled Battery");

  private final SendableChooser<BatteryManufacturer> batteryManufacturer =
      new SendableChooser<BatteryManufacturer>();

  public BatteryFuelGauge() {
    SmartDashboard.putData("Battery Gauge/Set Battery Nickname", setBatteryNickname());
    SmartDashboard.putData("Battery Gauge/Set Battery Manufacturer", setBatteryManufacturer());
    SmartDashboard.putData("Battery Gauge/New Manufacturer", batteryManufacturer);

    batteryNickname.setValue(battFuelGauge.getNickname());

    batteryManufacturer.addOption("Duracell", BatteryManufacturer.Duracell);
    batteryManufacturer.addOption("Energizer", BatteryManufacturer.Energizer);

    updateDashboardNickname();
    updateDashboardManufacturer();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Battery Gauge/Voltage", battFuelGauge.getVoltage());
    SmartDashboard.putNumber("Battery Gauge/Current", battFuelGauge.getCurrent());
    SmartDashboard.putNumber(
        "Battery Gauge/Remaining Charge (%)", battFuelGauge.getRemainingChargePct());

    SmartDashboard.putNumber("Battery Gauge/Capacity (Ah)", battFuelGauge.getCapacityAh());
    SmartDashboard.putNumber(
        "Battery Gauge/Effective Capacity (Ah)", battFuelGauge.getEffectiveCapacityAh());
    SmartDashboard.putNumber(
        "Battery Gauge/Rated Capacity (Ah)", battFuelGauge.getRatedCapacityAh());
    SmartDashboard.putNumber("Battery Gauge/Cycles", battFuelGauge.getNumCycles());
    SmartDashboard.putNumber("Battery Gauge/Age (Days)", battFuelGauge.getBatteryAgeDays());
  }

  public void
      updateDashboardNickname() { // Get the nickname from the battery gauge and update the display
    // on the smart dashboard
    SmartDashboard.putString("Battery Gauge/Nickname", battFuelGauge.getNickname());
  }

  public void updateDashboardManufacturer() {
    // Get the manufacturer from the battery gauge and update the display on the smart dashboard
    String batteryManufacturerString;
    switch (battFuelGauge.getManufacturer()) {
      case Duracell:
        batteryManufacturerString = "Duracell";
        break;

      case Energizer:
        batteryManufacturerString = "Energizer";
        break;

      default:
        batteryManufacturerString = "Unknown";
        break;
    }
    SmartDashboard.putString("Battery Gauge/Manufacturer", batteryManufacturerString);
  }

  public void saveBatteryNickname() {
    // Function that saves the battery nickname to the battery gauge. This does NOT return a
    // command.
    battFuelGauge.setNickname(batteryNickname.getValue());
    updateDashboardNickname();
  }

  public void saveBatteryManufacturer() {
    // Function that saves the battery manufacturer to the battery gauge. This does NOT return a
    // command.
    battFuelGauge.setManufacturer(batteryManufacturer.getSelected());
    updateDashboardManufacturer();
  }

  public Command setBatteryNickname() {
    return new InstantCommand(() -> saveBatteryNickname());
  }

  public Command setBatteryManufacturer() {
    return new InstantCommand(() -> saveBatteryManufacturer());
  }
}

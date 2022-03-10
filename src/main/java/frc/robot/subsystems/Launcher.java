
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Launcher extends SubsystemBase {

  // Falcon 500 is controlled by TalonFX
  // These are the two motors for launching the cargo

  private final TalonFX backLauncherMotor = new TalonFX(Constants.BACK_LAUNCHER_MOTOR);
  private final TalonFX frontLauncherMotor = new TalonFX(Constants.FRONT_LAUNCHER_MOTOR);

  public Launcher() {
    // backLauncherMotor.configFactoryDefault();
    backLauncherMotor.setInverted(true);
    backLauncherMotor.setNeutralMode(NeutralMode.Coast);
    // frontLauncherMotor.configFactoryDefault();
    frontLauncherMotor.setNeutralMode(NeutralMode.Coast);
    frontLauncherMotor.setInverted(true);

    backLauncherMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    frontLauncherMotor.configNominalOutputForward(0, Constants.kTimeoutMs);
    backLauncherMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    frontLauncherMotor.configNominalOutputReverse(0, Constants.kTimeoutMs);
    
  }

  /** Launches the Cargo with speed set for low hub
    * Eventually the absolute value could potentially be replaced with sensor-driven values
    **/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLauncherSpeed(double speedFront, double speedBack) {
    frontLauncherMotor.set(ControlMode.PercentOutput, speedFront);
    backLauncherMotor.set(ControlMode.PercentOutput, speedBack);
  }
  
  // Set HIGH goal speed
  public void setLauncherHigh() {
    frontLauncherMotor.set(ControlMode.PercentOutput, 0.40);
    backLauncherMotor.set(ControlMode.PercentOutput, 0.35);
  }

  // Set LOW goal speed
  public void setLauncherLow() {
    backLauncherMotor.set(ControlMode.PercentOutput, 0.20);
    frontLauncherMotor.set(ControlMode.PercentOutput, 0.20);
  }
  
  public void stopLauncher() {
    backLauncherMotor.set(ControlMode.PercentOutput, 0);
    frontLauncherMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getIntakeSpeed() {
    return backLauncherMotor.getMotorOutputPercent(); // add another for Top if desired
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
} // End class
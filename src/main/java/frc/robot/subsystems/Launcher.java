
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Displaying data?

// CTRE imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Launcher extends SubsystemBase {

  // Falcon 500 is controlled by TalonFX
  // These are the two motors for launching the cargo
  // This might need to go in the subsystem below

  private final TalonFX bottomLauncherMotor = new TalonFX(Constants.BOTTOM_LAUNCHER_MOTOR);
  private final TalonFX topLauncherMotor = new TalonFX(Constants.TOP_LAUNCHER_MOTOR);

  // invert commands are not working for some reason 
  // final TalonFXInvertType topLaunchMotor = TalonFXInvertType.CounterClockwise;
  // final TalonFXInvertType bottomLaunchMotor = TalonFXInvertType.Clockwise;

    
  public Launcher() {
    bottomLauncherMotor.configFactoryDefault();
    bottomLauncherMotor.setInverted(false); // FIXME double check direction
    bottomLauncherMotor.setNeutralMode(NeutralMode.Coast);
    topLauncherMotor.configFactoryDefault();
    topLauncherMotor.setInverted(true); // FIXME double check direction
    topLauncherMotor.setNeutralMode(NeutralMode.Coast);
  }

  /** Launches the Cargo with speed set for low hub
    * Eventually the absolute value could be replaced with sensor-driven values
    **/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setLauncherSpeed(double speedTop, double speedBottom) {
    bottomLauncherMotor.set(ControlMode.PercentOutput, speedBottom);
    topLauncherMotor.set(ControlMode.PercentOutput, speedTop);
  }

  public void stopLauncher() {
    bottomLauncherMotor.set(ControlMode.PercentOutput, 0);
    topLauncherMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getIntakeSpeed() {
    return bottomLauncherMotor.getMotorOutputPercent(); // add another for Top if desired
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
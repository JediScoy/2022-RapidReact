// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LauncherSpeed;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

// private final TalonFX m_LauncherMotorTop = new TalonFX(Constants.topLaunchMotor);
// private final TalonFX m_LauncherMotorBottom = new TalonFX(Constants.bottomLaunchMotor);

// Change from m_subsystem references to m_launcher in command

public class LaunchCargo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Subsystem the command uses
  private final Launcher subsystem;
  private double speed; 
 


  public LaunchCargo(Launcher subsystem, double speed) {
    this.subsystem = subsystem;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      subsystem.setLauncherSpeed(speed); // FIXME Launcher or setLauncherSpeed?
    
    }
    
    /** Stops the launcher - maybe unnecessary now
    public void stopLaunch(){
      topLauncherMotor.set(ControlMode.PercentOutput, 0);
      bottomLauncherMotor.set(ControlMode.PercentOutput, 0);
    }
    */ 

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Connors Code - not sure about code location
    // Intersting that adding it here also works for the High cargo?!
    // m_launcherSubsystem.setDefaultCommand(new StopLaunch(m_launcherSubsystem));
    
    // This is more like the Hatchbot example online
    // LauncherSubsystem.setDefaultCommand(StopLaunch);

    // Try AFTER testing all of the other "new" stuff
    // EXAMPLE m_LauncherSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    // TalonFX m_LauncherSubsystem.set(new ControlMode.PercentOutput, 0);
    // m_LauncherMotorBottom.set(new ControlMode.PercentOutput, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
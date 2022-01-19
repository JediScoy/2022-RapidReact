// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;


// Change from m_subsystem references to m_launcher in command

public class LaunchCargoLow extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  // Subsystem the command uses
  private final LauncherSubsystem m_launcherSubsystem; 

  public LaunchCargoLow(LauncherSubsystem subsystem) {
    m_launcherSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launcherSubsystem.launchCargoLow();
    
    }
    

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
    // m_LauncherMotorTop.set(new ControlMode.PercentOutput, 0));
    // m_LauncherMotorBottom.set(new ControlMode.PercentOutput, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
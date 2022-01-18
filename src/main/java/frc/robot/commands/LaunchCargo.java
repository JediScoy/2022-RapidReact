// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

// Change from m_subsystem references to m_launcher in command

/** An example command that uses an example subsystem. */
public class LaunchCargo extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  // Subsystem the command uses
  private final LauncherSubsystem m_launcherSubsystem; 


  /**
   * Creates a new ExampleCommand.
   * The subsystem used by this command.
   */

  public LaunchCargo(LauncherSubsystem subsystem) {
    m_launcherSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_launcherSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_launcherSubsystem.launchCargo();
    
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

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

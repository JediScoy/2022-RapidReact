// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonBlue1 extends SequentialCommandGroup {
  
    public AutonBlue1(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotors, Launcher launcher) {
    addCommands(
      new IndexCommand(indexMotors, 0.2),
      new IntakeCommand(intakeMotors, 0.2)
      ); // End of commands
 
  }  
    
  } // end class
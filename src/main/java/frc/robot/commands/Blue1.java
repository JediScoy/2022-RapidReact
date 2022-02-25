// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Blue1 extends SequentialCommandGroup {
  
    public Blue1(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher, Launcher speedFront, Launcher speedBack) {
      addCommands(
        new LauncherSpeed(launcher, 0.2, 0.2).alongWith(
          new IndexCommand(indexMotors, 0.2).alongWith(
          new IntakeCommand(intakeMotor, 0.2)
      ))); // end of add commands
 
  }  
    
  } // end class
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;
import frc.robot.commands.IndexSpeed;
import frc.robot.commands.IntakeSpeed;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// FIXME Autonomous place holder for testing purposes
public class Red2 extends SequentialCommandGroup {

    public Red2(DrivetrainSubsystem drivetrain, Index indexMotors, Intake intakeMotor, Launcher launcher) {
      addCommands(
        new IndexSpeed(indexMotors, -0.2).alongWith(new IntakeSpeed(intakeMotor, -0.1)));
      
    }  
  
  } // end class
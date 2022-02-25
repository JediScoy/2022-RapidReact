// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Launcher;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonRed1 extends SequentialCommandGroup {

    public AutonRed1(Launcher launcher, Intake intakeMotor, Index indexMotors) {
    addCommands(
        new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(1),
          new LauncherSpeed(launcher, 0.35, 0.40),
          new IntakeCommand(intakeMotor, -0.5),
          new IndexCommand(indexMotors, 0.5)
    );
  }  
    
  } // end class
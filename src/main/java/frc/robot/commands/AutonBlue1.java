// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** A complex auto command that drives forward, releases a hatch, and then drives backward. */
public class AutonBlue1 extends SequentialCommandGroup {
  
  public AutonBlue1(DrivetrainSubsystem drive, Intake intake) {
    addCommands(
        // Drive forward the specified distance
        new DriveDistance(
            Constants.kAutoDriveDistanceInches, Constants.kAutoDriveSpeed, drive),

        // Release the hatch
        new ReleaseHatch(hatch),

        // Drive backward the specified distance
        new DriveDistance(
            Constants.kAutoBackupDistanceInches, -Constants.kAutoDriveSpeed, drive));
  }
}

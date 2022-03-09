// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


// FRC 3603 using from Alpha

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToDistance extends CommandBase {
  /** Creates a new DriveToDistanceCommand. */

  Drivetrain drivetrain;
  public double encoderClicks;
  public double desiredDistance;
  public double initialClicks;
  public double initialClicksAverage;
  public double desiredClicks;
  private double clicksTravelled = 0;

  public DriveToDistance(Drivetrain dts, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.

    drivetrain = dts;
    desiredDistance = distance;
    addRequirements(drivetrain);
    // ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    //   tab.addNumber("Left Module Clicks", ()->getClicksNeeded(distance));

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clicksTravelled = 0;
    drivetrain.resetEncoders();
    desiredClicks = getClicksNeeded(desiredDistance);
    while (drivetrain.getFrontRightEncoderValue() != 0) {
      drivetrain.resetEncoders();
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   SmartDashboard.putNumber("Desired Clicks", desiredClicks);

   if (desiredDistance > 0) {
    drivetrain.driveForward(3);
   }
   else {
     drivetrain.driveBackward(3);
   }

  // drivetrainSubsystem.drive(new ChassisSpeeds(0.2, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.drive(new ChassisSpeeds(0,0,0));
    drivetrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // this if statement checks the reading of the encoder used to check average encoder values.
    // we will now make it read the encoder values of only one encoder, after running reset encoder value method
    if (clicksTravelled >= desiredClicks) {
      SmartDashboard.putBoolean("Is Finished", true);
      return true;
    } else {
      SmartDashboard.putBoolean("Is Finished", false);
      clicksTravelled = Math.abs(drivetrain.getFrontRightEncoderValue());
      return false;
    }
  }

  public double getClicksNeeded(double desiredDistance) {
    // this is a function to convert desired distance in meters to the number of clicks
    // double clicksPerDegree = (2048/360);
    // double clicksPerRadians = Units.degreesToRadians(clicksPerDegree);
    // double wheelRadius = Units.inchesToMeters(2);
    // double clicks = (desiredDistance/wheelRadius)*clicksPerRadians;

    // this will deliver ticks needed given a desired distance in inches
    double tickin = ((2048*6.86)/(Math.PI*4));
    double clicks = tickin*desiredDistance;
    clicks = Math.abs(clicks);
    return clicks;
}
}

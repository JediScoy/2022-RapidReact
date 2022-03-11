// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** FRC 3603 Constants
 * 
 */
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Drivetrain;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.585; // FIXED Measure and set trackwidth
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
//public static final double rot2Meter =  Mk3SwerveModuleHelper.GearRatio.STANDARD * Math.PI * WHEEL_DIAMETER


    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.585; // FIXED Measure and set wheelbase
    // public static final int DRIVETRAIN_PIGEON_ID = 0; // Set Pigeon ID - We do not use Pigeon

      //moving this from DrivetrainSubsystem to see if it works
      public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
      );

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2; // Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 21; // Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(60.4); 
    // abs 1.9 - 59.4 = 57.5 offset on 2/16/22
    // 360 - 358.5 + 1.8 = 3.3
    // 357.5 - 360+2.1 = -0.4
    // 178; 182.4; 4.4; adding half
    // 179.5; 180.5

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4; // Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22; // Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(9.5); //
    // abs 8.3-1.9 = 6.4 offset on 2/16/22
    // 360 - 357.6 + 1.8 = 4.2
    // 357.4 - 360 + 4.4 = 1.8
    // 360 - 357.6 = 2.4
    // 177.6; 180.6; 4;
    // 178.0; 180.0;

    
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23; // Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(297.6); //
    // 177.1-1.7 = 175.4 ; no
    // 357.6 - 303.4 = 54.2
    // 357.6 - 180 = 177.6
    // 54.2 + 180 
    // 177 - 246 = 69 + (357-229) = 197
    // 107 + 180
    // 287: 181.4, 11.6 ()
    // 297.6-180 = 117.6 


    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8; // Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 24; // Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(269.85); // +2.75
    // abse 1.8-88.9 = 87.1 offset
    // backwards;87.1+180 = 267.1
    // adding 180 or -180 changes the wheel drive direction, but not the orientation
    // 245
    // 267.1: 359, 1.75
    

    // Constants for Launcher
    public static final class Launcher {
      // Two motors are used to launch the "Cargo"
      // Sets the bottom launcher motor ID value
      public static final int BACK_LAUNCHER_MOTOR = 9; 
      // Sets the top launcher motor ID value
      public static final int FRONT_LAUNCHER_MOTOR = 10; 
    }

    // Intake motors
    public static final int INTAKE_MOTOR = 11; // Sets the Intake Motor ID value

    // Index motors
    public static final int FRONT_INDEX_MOTOR = 13; // Grouped with the mid motor
    public static final int BACK_INDEX_MOTOR = 14; // Group with the high motor

    // Lift motors
    public static final int LEFT_LIFT_MOTOR = 15; // Grouped
    public static final int RIGHT_LIFT_MOTOR = 16; // Grouped
    
    // Rotational motors
    public static final int LEFT_PIVOT_MOTOR = 17; // Grouped
    public static final int RIGHT_PIVOT_MOTOR = 18; // Group

     // Constants for the Autonomous subsystem
     public static final class Auton {
    }
     //Autononomous values
     public static final double kPXController = 1.5; // default 1.5 from SeanSun; forum values, our last was 3
     public static final double kPYController = 1.5; // default 1.5; forum values
     public static final double kPThetaController = 0.0; // default 3.0; forum value, our last was 0.01
     public static final int kTimeoutMs = 30; // Added

 
     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
     new TrapezoidProfile.Constraints(
         Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
         Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    
    /**
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
      SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
    */
}

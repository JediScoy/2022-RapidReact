// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.DrivetrainSubsystem;

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

    // Motors are ID in the CTRE Phoenix Tuner need to match code
    // Values have been assigned from 2021 code 2021-11-30
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2; // Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 21; // Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(40.0); // Measure and set front right steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4; // Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22; // Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(19.5); // Measure and set back right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23; // Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(61.0); // Measure and set back left steer offset

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8; // Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 24; // Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(157.5); // Measure and set front left steer offset

    // Two motors are used to launch the "Cargo"
    public static final int BACK_LAUNCHER_MOTOR = 9; // Sets the bottom launcher motor ID value
    public static final int FRONT_LAUNCHER_MOTOR = 10; // Sets the top launcher motor ID value
    
   
    // FIXME Hypothetical motor IDs. These need to be set with Pheonix Tuner
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

     //Stuff for Auton
     public static final double kPXController = 3.0; // default 1.5 from SeanSun; forum values 
     public static final double kPYController = 0.0; // default 1.5; forum values
     public static final double kPThetaController = 0.01; // default 3.0; forum value
 
     public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
     new TrapezoidProfile.Constraints(
         DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
         DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

}

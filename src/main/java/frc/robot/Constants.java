// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.synth.Region;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //#region PID
    public static double kPTurning = .5;
//#endregion
   



    //trackwidth = length, base = width. There is no difference; this is a square.
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.585; // FIXED Measure and set trackwidth
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.585; // FIXED Measure and set wheelbase
    
    //#region motorconfig

    //------------VAGUELY IMPORTANT this is a region(custom collapsable section) that works with the extension "SantaCodes.santacodes-region-viewer." Further, you may choose to make a custom snippet(shorthand when you type), which I reccommend. -shaun
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
    public static final int BOTTOM_LAUNCHER_MOTOR = 9; // Sets the bottom launcher motor ID value
    public static final int TOP_LAUNCHER_MOTOR = 10; // Sets the top launcher motor ID value
    







    //#endregion
    
    public static final double MAX_VOLTAGE = 12.0;
    // Measure the drivetrain's maximum velocity or calculate the theoretical.
    //  The formula for calculating the theoretical maximum velocity is:
    //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
    //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
    //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
    //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * // FIXME Use our robots' velocity values
            SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
            SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
  
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );
  
    // Intake motors
    public static final int INTAKE = 11; // Sets the Intake Motor ID value; needs to match in Phoenix Tuner.

    // Index motors
    /** */

    
    // Already defined in DrivetrainSub as "m_navx"
    // public static AHRS gyro = new AHRS(Port.kMXP); // From 2021-Infinite Recharge

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

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
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
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
  public static final double MAX_ACCELERATION_METERS_SECOND_SQUARED = 5;//from (our) deprecated pathplanner code

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 * // FIXME Use our robots' velocity values
          SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
          SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
 
 
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  

  public class m_motionMagicConstants{
  //0 to 3 slots availble for PID "profiles"
  public static final int kDrivePIDSlot = 0;
  //the mode is binary; 0 is normal, and 1 is cascaded. We only want a basic normal loop(for now) 
  public static final int kDrivePIDLoop = 0;
//for the sake of easy reassignment
  public static final int kTimeoutMs = 0;
//gains for the motionmagic used in the auton
 public final Gains kDriveMotionMagicGains = new Gains(0, 0, 0, 0, 0, 0);
  
  }
  
  
  
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


      public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);



    // TODO Set the offsets using shuffleboard and a straight edge
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 1; // Set front right drive motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 2; // Set front right steer motor ID
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 21; // Set front right steer encoder ID
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(57.7); 
    // abs 1.9 - 59.4 = 57.5 offset on 2/16/22
    // 360 - 358.5 + 1.8 = 3.3
    // 357.5 - 360+2.1 = -0.4

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 3; // Set back right drive motor ID
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 4; // Set back right steer motor ID
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 22; // Set back right steer encoder ID
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(6.5); //
    // abs 8.3-1.9 = 6.4 offset on 2/16/22
    // 360 - 357.6 + 1.8 = 4.2
    // 357.4 - 360 + 4.4 = 1.8
    // 360 - 357.6 = 2.4
    // 

    
    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; // Set back left drive motor ID
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; // Set back left steer motor ID
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23; // Set back left steer encoder ID
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(297.6); // +10.2
    // 177.1-1.7 = 175.4 ; no
    // 357.6 - 303.4 = 54.2
    // 357.6 - 180 = 177.6
    // 54.2 + 180 
    // 177 - 246 = 69 + (357-229) = 197
    // 107 + 180
    // 287: 181.4, 11.6 ()


    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 7; // Set front left module drive motor ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 8; // Set front left module steer motor ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 24; // Set front left steer encoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(269.85); // +2.75
    // abse 1.8-88.9 = 87.1 offset
    // backwards;87.1+180 = 267.1
    // adding 180 or -180 changes the wheel drive direction, but not the orientation
    // 245
    // 267.1: 359, 1.75
    
    

    // Two motors are used to launch the "Cargo"
    public static final int BACK_LAUNCHER_MOTOR = 9; // Sets the bottom launcher motor ID value
    public static final int FRONT_LAUNCHER_MOTOR = 10; // Sets the top launcher motor ID value
    
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
         MAX_VELOCITY_METERS_PER_SECOND,
         MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);

}

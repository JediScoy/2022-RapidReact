// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** FRC 3603 Robot Container
 *
 */
package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Subsystem imports
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;

// Command imports
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IndexSpeed;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.LauncherSpeed;
import frc.robot.commands.Lift.AutoLiftCommandBar1;
import frc.robot.commands.Lift.AutoLiftCommandBar2;
import frc.robot.commands.Lift.LiftCommand;
import frc.robot.commands.Lift.LockLiftCommandBar1;
import frc.robot.commands.Lift.LockLiftCommandBar2;
import frc.robot.commands.PathPlannerBased.CG_2BalDrivePlus;
import frc.robot.commands.PathPlannerBased.CG_2BallDrive;
import frc.robot.commands.PathPlannerBased.Drive2m;
import frc.robot.commands.PathPlannerBased.DriveV;
import frc.robot.commands.auton.CG_1BallDriveNone;
import frc.robot.commands.auton.CG_1BallDriveStraight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
 
  // Main driver controller
  private final XboxController driverController = new XboxController(0);
  // Second operator controller
  private final XboxController operatorController = new XboxController(1);

  // The robot's subsystems and commands are defined here...
  //// Drivetrain
  private final Drivetrain m_drivetrain = new Drivetrain();
  
  //// Intake
  private final Intake intakeMotor = new Intake();
  
  //// Launcher
  private final Index indexMotors = new Index();
  private final Launcher launcher = new Launcher();
  
  //// Lift 
  private final Lift liftMotors = new Lift();
  private final Lift rightLiftMotor = new Lift();
  // private final Lift leftLiftMotor = new Lift();
  // private final LiftPivot liftPivotMotors = new LiftPivot();

  //// Limelight
  private final Limelight limelight = new Limelight();
  
  //// Autonomous
  SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** TODO Remove after testing the new chooser setup that contains these commands directly   
  private final Command autonLaunch1 = new CGLaunch1_DriveNone(indexMotors, intakeMotor, launcher);

  private final Command autonLaunch2Drive = new CGLaunch2_DriveStraight(m_drivetrain, indexMotors, intakeMotor, launcher);

  private final Command autonShortDrive = new DriveShort();
  
  private final Command autonStraight = new CGLaunch1_DriveStraight(m_drivetrain, launcher);
  */

  private final Command driveCommand; 
  // private final Command defaultDriveCommand;

  // private final Command PathStraight =
    // new PathStraight(m_drivetrainSubsystem);
  
  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    driveCommand = new DriveCommand(
      m_drivetrain,
      () -> -modifyAxis(driverController.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, // divide by 4 for slow mo
      () -> -modifyAxis(driverController.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, // divide by 4 for slow mo
      () -> -modifyAxis(driverController.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND); // divide by 4 for slow mo
    
    m_drivetrain.setDefaultCommand(driveCommand);
    
    // TODO add and test autonomous files here
    // AUTONOMOUS chooser

    // Launches high goal inside the tarmac, doesn't drive out

    // Launches high goal inside tarmac, drives out with launch sequence operating
    autonChooser.addOption("Launch 2",
      new CG_2BallDrive(m_drivetrain, indexMotors, intakeMotor, launcher));
    
    autonChooser.addOption("Launch 2, Pickup 1",
      new CG_2BalDrivePlus(m_drivetrain, indexMotors, intakeMotor, launcher));
    
    autonChooser.addOption("Drive 2m",
      new Drive2m(m_drivetrain));

    autonChooser.addOption("Drive V",
      new DriveV(m_drivetrain, indexMotors, intakeMotor, launcher));

    autonChooser.setDefaultOption("1 Ball, Drive straight",
      new CG_1BallDriveStraight(m_drivetrain, indexMotors, intakeMotor, launcher));
    
      // Drives out only. TODO Drivetrain testing purposes only -- would not use in competition
    // Tested, but it doesn't function at the moment
    // autonChooser.addOption("Drive Short test",
    // new CGLaunch0_DriveShort());

    // Puts the chooser on the dashboard
    Shuffleboard.getTab("Auton").add(autonChooser).withSize(2, 4);

    // DEBUGGING CODE:
    // System.out.println("subsystem requirements for autonShortDrive");
    // autonShortDrive.getRequirements().forEach((x) -> System.out.println(x));

    System.out.println("subsystem requirements for DriveCommand");
    driveCommand.getRequirements().forEach((x) -> System.out.println(x));
  }

  public void debugMethod () {
    // SmartDashboard.putBoolean("Short Drive", autonShortDrive.isScheduled());
    SmartDashboard.putBoolean("DriveCommand", driveCommand.isScheduled());
  }

  private void configureButtonBindings() {
    /// Declaring buttons on driver controller
    final JoystickButton d_backButton = new JoystickButton(driverController, Button.kBack.value);
    final JoystickButton d_ButtonA = new JoystickButton(driverController, Button.kA.value);
    final JoystickButton d_ButtonB = new JoystickButton(driverController, Button.kB.value);
    final JoystickButton d_ButtonX = new JoystickButton(driverController, Button.kX.value);
    final JoystickButton d_ButtonY = new JoystickButton(driverController, Button.kY.value);
    final JoystickButton d_RightBumper = new JoystickButton(driverController, Button.kRightBumper.value);
    final JoystickButton d_LeftBumper = new JoystickButton(driverController, Button.kLeftBumper.value);

    // Declaring buttons on the operator controller
    final JoystickButton op_ButtonA = new JoystickButton(operatorController, Button.kA.value);
    final JoystickButton op_ButtonB = new JoystickButton(operatorController, Button.kB.value);
    final JoystickButton op_ButtonX = new JoystickButton(operatorController, Button.kX.value);
    final JoystickButton op_ButtonY = new JoystickButton(operatorController, Button.kY.value);
    final JoystickButton op_StartButton = new JoystickButton(operatorController, Button.kStart.value);
    final JoystickButton op_BackButton = new JoystickButton(operatorController, Button.kBack.value);
    final JoystickButton op_RightBumper = new JoystickButton(operatorController, Button.kRightBumper.value);
    final JoystickButton op_LeftBumper = new JoystickButton(operatorController, Button.kLeftBumper.value);

    // Defining the actions associated with buttons

    // DRIVER Controller button commands

    // Resets the gyroscope to 0 degrees when back button is pressed
    d_backButton.whenPressed(m_drivetrain::zeroGyroscope);

      /**  LOW HOOP UP CLOSE LAUNCH SEQUENCE
       when A is held, run Launch motors by themselves for a second, then run Launch and Index motors for 0.5 seconds,
       then finally run all 3 motors at once. release to stop all motors */
      d_ButtonA.whenPressed(new SequentialCommandGroup(
        new LauncherSpeed(launcher, 0.20, 0.20).withTimeout(1),
          new SequentialCommandGroup(
            new LauncherSpeed(launcher, 0.20, 0.20).withTimeout(0.5).alongWith(
              new IndexSpeed(indexMotors, 0.5).withTimeout(0.5)),
                new ParallelCommandGroup (
                  new LauncherSpeed(launcher, 0.25, 0.25),
                  new IntakeSpeed(intakeMotor, 0.5),
                  new IndexSpeed(indexMotors, 0.5)))
      ));
      //stops all 3 motors when A button released
      d_ButtonA.whenReleased(new ParallelCommandGroup(
        new IntakeSpeed(intakeMotor, 0.0),
        new IndexSpeed(indexMotors, 0.0),
        new LauncherSpeed(launcher, 0.0, 0.0))
      );

     /**  HIGH HOOP EDGE OF TARMAC LAUNCH SEQUENCE
       when Y is held, run Launch motors by themselves for 0.75 seconds, then run Launch and Index motors for 0.25 seconds,
       then finally run all 3 motors at once. release button to stop all motors */
      d_ButtonY.whenPressed(new SequentialCommandGroup(
        new LauncherSpeed(launcher, 0.35, 0.40).withTimeout(0.75),
          new SequentialCommandGroup(
            new LauncherSpeed(launcher, 0.35, 0.40).withTimeout(0.25).alongWith(
              new IndexSpeed(indexMotors, 0.5).withTimeout(0.25)),
                new ParallelCommandGroup (
                  new LauncherSpeed(launcher, 0.36, 0.42),
                  new IntakeSpeed(intakeMotor, 0.5),
                  new IndexSpeed(indexMotors, 0.5))
      )));
      //stops all 3 motors when Y button released
      d_ButtonY.whenReleased(new ParallelCommandGroup(
        new IntakeSpeed(intakeMotor, 0.0),
        new IndexSpeed(indexMotors, 0.0),
        new LauncherSpeed(launcher, 0.0, 0.0))
      );

    // Hold right bumper to manually Intake cargo from the field, release to stop motors
    d_RightBumper.whenPressed(new IntakeSpeed(intakeMotor, -0.5));
    d_RightBumper.whenReleased(new IntakeSpeed(intakeMotor, 0.0));

    // Hold left bumper to manually Reverse cargo back to the field, release to stop motors
    d_LeftBumper.whenPressed(new IntakeSpeed(intakeMotor, 0.5));
    d_LeftBumper.whenReleased(new IntakeSpeed(intakeMotor, 0.0));

    // Hold X to manually Advance cargo to the launcher, release to stop motors
    d_ButtonX.whenPressed(new IndexSpeed(indexMotors, 0.5));
    d_ButtonX.whenReleased(new IndexSpeed(indexMotors, 0));

    //Hold B to drive at slower speed, release to drive normal
    d_ButtonB.whenPressed(new DriveCommand(
      m_drivetrain,
      () -> -modifyAxis(driverController.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND / 6,
      () -> -modifyAxis(driverController.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND / 6,
      () -> -modifyAxis(driverController.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 6
    ));
    d_ButtonB.whenReleased(new DriveCommand(
      m_drivetrain,
      () -> -modifyAxis(driverController.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driverController.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(driverController.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));


    // OPERATOR Controller commands

    //Hold X to rotationally align the robot (driver still has control of translational motion)
      op_ButtonX.whenPressed(new DriveCommand(
        m_drivetrain,
          () -> -modifyAxis(driverController.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driverController.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
          () -> {return -limelight.getX() * Constants.kPThetaLimelightController;}
      ));
      op_ButtonX.whenReleased(new DriveCommand(
          m_drivetrain,
          () -> -modifyAxis(driverController.getLeftY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driverController.getLeftX()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -modifyAxis(driverController.getRightX()) * Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      ));

    // hold left bumper to manually raise both climbing arms, release to stop motors
    op_LeftBumper.whenPressed(new LiftCommand(liftMotors, 0.5));
    op_LeftBumper.whenReleased(new LiftCommand(liftMotors, 0.0));

    // hold right bumper to manually lower both climbing arms, release to stop motors
    op_RightBumper.whenPressed(new LiftCommand(liftMotors, -0.5));
    op_RightBumper.whenReleased(new LiftCommand(liftMotors, 0.0));

    // press A to auto raise both climbing arms to the encoder value of bar #1
    op_ButtonA.whenPressed(new AutoLiftCommandBar1(liftMotors, 0.5));

    // press Back Button to auto lower both climbing arms to the encoder value of when the locking arms engage on bar #1
    op_BackButton.whenPressed(new LockLiftCommandBar1(liftMotors, -0.5));

    // press Y to auto raise both climbing arms to encoder value of bar #2
    op_ButtonY.whenPressed(new AutoLiftCommandBar2(liftMotors, 0.5));

    // press Start Button to auto lower both climbing arms to the encoder value of when the locking arms engage on bar #2
    op_StartButton.whenPressed(new LockLiftCommandBar2(liftMotors, -0.5));

    /** Did not work, no movement
    // Use left stick up and down to manually move ONLY left climbing arm up and down
    leftLiftMotor.setDefaultCommand(new DefaultDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(operatorController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(operatorController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(operatorController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    */

    // Use right stick up and down to manually move ONLY right climbing arm up and down
    rightLiftMotor.setDefaultCommand(new LiftCommand(
      rightLiftMotor, modifyAxis(operatorController.getRightY()) * Drivetrain.MAX_VELOCITY_METERS_PER_SECOND
      ));

  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */


  public Command getAutonomousCommand() {

    return autonChooser.getSelected();

  }; // end of getAutonomusCommand()

} // End of class

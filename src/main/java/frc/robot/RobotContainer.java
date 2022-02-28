// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton; //NewCommands vendordep
import static edu.wpi.first.wpilibj.XboxController.Button;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// Subsystem imports
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.commands.AutoLiftCommandBar1;
import frc.robot.commands.AutoLiftCommandBar2;
// Command imports
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IndexSpeed;
import frc.robot.commands.IntakeSpeed;
import frc.robot.commands.LauncherSpeed;
import frc.robot.commands.LiftCommand;

// Auton
import frc.robot.commands.auton.Blue1;
import frc.robot.commands.auton.Blue2;
import frc.robot.commands.auton.Blue3;
import frc.robot.commands.auton.Red1;
import frc.robot.commands.auton.Red2;
import frc.robot.commands.auton.Red3;
import frc.robot.commands.LockLiftCommandBar1;
import frc.robot.commands.LockLiftCommandBar2;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Index indexMotors = new Index();
  private final Intake intakeMotor = new Intake();
  private final Launcher launcher = new Launcher();
  private final Lift liftMotors = new Lift();
  private final Lift leftLiftMotor = new Lift();
  private final Lift rightLiftMotor = new Lift();
  // private final LiftPivot liftPivotMotors = new LiftPivot();

  // Main driver controller
  private final XboxController driverController = new XboxController(0);
  // Second operator controller
  private final XboxController operatorController = new XboxController(1);
  
  // Autononmous TODO Auton references. Update throughout season
  private final Command blueOne =
    new Blue1(m_drivetrainSubsystem, indexMotors, intakeMotor, launcher);

  private final Command blueTwo =
    new Blue2(m_drivetrainSubsystem, indexMotors, intakeMotor, launcher);

  private final Command blueThree =
    new Blue3(m_drivetrainSubsystem, indexMotors, intakeMotor, launcher);

  private final Command redOne =
    new Red1(m_drivetrainSubsystem, indexMotors, intakeMotor, launcher);
  
  private final Command redTwo =
    new Red2(m_drivetrainSubsystem, indexMotors, intakeMotor, launcher);

  private final Command redThree =
    new Red3(m_drivetrainSubsystem, indexMotors, intakeMotor, launcher);
  
  // A chooser for autonomous commands
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(driverController.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driverController.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driverController.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    
    // TODO Add commands to the autonomous command chooser
    m_chooser.setDefaultOption("Blue 1", blueOne);
    m_chooser.addOption("Blue 2", blueTwo);
    m_chooser.addOption("Blue 3", blueThree);
    m_chooser.addOption("Red 1", redOne);
    m_chooser.addOption("Red 2", redTwo);
    m_chooser.addOption("Red 3", redThree);

    // Puts the chooser on the dashboard
    Shuffleboard.getTab("Auton").add(m_chooser);
  }

  private void configureButtonBindings() {
    /// Declaring buttons on driver controller
    final JoystickButton d_backButton = new JoystickButton(driverController, Button.kBack.value);
    // final JoystickButton d_startButton = new JoystickButton(driverController, Button.kStart.value);
    final JoystickButton d_ButtonA = new JoystickButton(driverController, Button.kA.value);
    //final JoystickButton d_ButtonB = new JoystickButton(driverController, Button.kB.value);
    final JoystickButton d_ButtonX = new JoystickButton(driverController, Button.kX.value);
    final JoystickButton d_ButtonY = new JoystickButton(driverController, Button.kY.value);
    final JoystickButton d_RightBumper = new JoystickButton(driverController, Button.kRightBumper.value);
    final JoystickButton d_LeftBumper = new JoystickButton(driverController, Button.kLeftBumper.value);
    // final double d_LeftTrigger = driverController.getLeftTriggerAxis();
    // final double d_RightTrigger = driverController.getRightTriggerAxis();
    

    // Declaring buttons on the operator controller
    // final JoystickButton op_backButton = new JoystickButton(operatorController, Button.kBack.value);
    // final JoystickButton op_startButton = new JoystickButton(operatorController, Button.kStart.value);
    final JoystickButton op_ButtonA = new JoystickButton(operatorController, Button.kA.value);
    final JoystickButton op_ButtonB = new JoystickButton(operatorController, Button.kB.value);
    final JoystickButton op_ButtonX = new JoystickButton(operatorController, Button.kX.value);
    final JoystickButton op_ButtonY = new JoystickButton(operatorController, Button.kY.value);
    final JoystickButton op_RightBumper = new JoystickButton(operatorController, Button.kRightBumper.value);
    final JoystickButton op_LeftBumper = new JoystickButton(operatorController, Button.kLeftBumper.value);
    //final double op_LeftTrigger = operatorController.getLeftTriggerAxis();
    //final double op_RightTrigger = operatorController.getRightTriggerAxis();

    // Defining the actions associated with buttons
   

    //Driver Controller button commands

    // Resets the gyroscope to 0 degrees when back button is pressed
    d_backButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope); 

      /**  when A is held, run Launch motors by themselves for a second, then run Launch and Index motors for 0.5 seconds,
       then finally run all 3 motors at once. release to stop all motors */
        // Low shot up close
      d_ButtonA.whenPressed(new SequentialCommandGroup(
        new LauncherSpeed(launcher, 0.20, 0.20).withTimeout(1),
          new SequentialCommandGroup(
            new LauncherSpeed(launcher, 0.20, 0.20).withTimeout(0.5).alongWith(
              new IndexSpeed(indexMotors, 0.5).withTimeout(0.5)),
                new ParallelCommandGroup (
                  new LauncherSpeed(launcher, 0.25, 0.25),
                  new IntakeSpeed(intakeMotor, -0.5),
                  new IndexSpeed(indexMotors, 0.5)))
      ));
      d_ButtonA.whenReleased(new ParallelCommandGroup(
        new IntakeSpeed(intakeMotor, 0.0),
        new IndexSpeed(indexMotors, 0.0),
        new LauncherSpeed(launcher, 0.0, 0.0))
      );

     /**  when Y is held, run Launch motors by themselves for a second, then run Launch and Index motors for 0.5 seconds,
       then finally run all 3 motors at once. release to stop all motors */
        // High shot up close
      d_ButtonY.whenPressed(new SequentialCommandGroup(
        new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(1),
          new SequentialCommandGroup(
            new LauncherSpeed(launcher, 0.40, 0.45).withTimeout(0.5).alongWith(
              new IndexSpeed(indexMotors, 0.5).withTimeout(0.5)),
                new ParallelCommandGroup (
                  new LauncherSpeed(launcher, 0.35, 0.40),
                  new IntakeSpeed(intakeMotor, 0.5),
                  new IndexSpeed(indexMotors, 0.5))
      )));
      d_ButtonY.whenReleased(new ParallelCommandGroup(
        new IntakeSpeed(intakeMotor, 0.0),
        new IndexSpeed(indexMotors, 0.0),
        new LauncherSpeed(launcher, 0.0, 0.0))
      );

    // Hold right bumper to manually Intake cargo from the field, release to stop motors
    d_RightBumper.whenPressed(new IntakeSpeed(intakeMotor, 0.5)); 
    d_RightBumper.whenReleased(new IntakeSpeed(intakeMotor, 0.0)); 

    // Hold left bumper to manually Reverse cargo back to the field, release to stop motors
    d_LeftBumper.whenPressed(new IntakeSpeed(intakeMotor, -0.5)); 
    d_LeftBumper.whenReleased(new IntakeSpeed(intakeMotor, 0.0)); 

    // Hold X to manually Advance cargo to the launcher, release to stop motors
    d_ButtonX.whenPressed(new IndexSpeed(indexMotors, 0.5)); 
    d_ButtonX.whenReleased(new IndexSpeed(indexMotors, 0));

    //Operator Controller
      //lift button commands

    // hold left bumper to manually raise climbing arms, release to stop motors
    op_LeftBumper.whenPressed(new LiftCommand(liftMotors, 0.5)); 
    op_LeftBumper.whenReleased(new LiftCommand(liftMotors, 0.0));
    
    // hold right bumper to manually lower climbing arms, release to stop motors
    op_RightBumper.whenPressed(new LiftCommand(liftMotors, -0.5)); 
    op_RightBumper.whenReleased(new LiftCommand(liftMotors, 0.0));

    // press A to auto raise climbing arms to the encoder value of bar #1
    op_ButtonA.whenPressed(new AutoLiftCommandBar1(liftMotors, 0.5)); 

    // press B to auto lower climbing arms to the encoder value of when the locking arms engage on bar #1
    op_ButtonB.whenPressed(new LockLiftCommandBar1(liftMotors, -0.5)); 

    // press Y to auto raise climbing arms to encoder value of bar #2
    op_ButtonY.whenPressed(new AutoLiftCommandBar2(liftMotors, 0.5)); 

    // press B to auto lower climbing arms to the encoder value of when the locking arms engage on bar #2
    op_ButtonX.whenPressed(new LockLiftCommandBar2(liftMotors, -0.5));  

    // Use left stick up and down to manually move left climbing arm up and down
    leftLiftMotor.setDefaultCommand(new LiftCommand(
      leftLiftMotor, modifyAxis(operatorController.getLeftY()))); //FIXME did not work, no movement

    // Use right stick up and down to manually move left climbing arm up and down
    rightLiftMotor.setDefaultCommand(new LiftCommand(
      rightLiftMotor, modifyAxis(operatorController.getRightY()))); //FIXME did not work, no movement
      
    /** Removed the Pivot arms from the robot, commenting out these buttons 
    op_ButtonX.whenPressed(new LiftPivotCommand(liftPivotMotors, 0.5)); // Rotates the pivot-lift arms for the higher bar
    op_ButtonX.whenReleased(new LiftPivotCommand(liftPivotMotors, 0.0));
  
    op_ButtonY.whenPressed(new LiftPivotCommand(liftPivotMotors, -0.5)); // Rotates the pivot-lifts arms back towards the robot
    op_ButtonY.whenReleased(new LiftPivotCommand(liftPivotMotors, 0.0));
    */
    
  /** Use this to pass the autonomous command to the main {@link Robot} class.
  * @return the command to run in autonomous
 */

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
  
  // TODO Adjust Auton Chooser VS built in Auton
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();  

      /** 
    // Load the path
    Trajectory m_path = PathPlanner.loadPath("Straight", 5, 5);

    // 2. Defining PID Controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 3. Command to follow path from PathPlanner
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      m_path, 
      m_drivetrainSubsystem::getPose, 
      Constants.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    // 4. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
      new InstantCommand(()
        -> m_drivetrainSubsystem.resetOdometry(m_path.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() 
        -> m_drivetrainSubsystem.stop()));
        */
    
    // System.out.println(exampleState.velocityMetersPerSecond);
  
  };    
  
} // End of class
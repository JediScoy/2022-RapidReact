// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.buttons.Trigger;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.buttons.JoystickButton; // OldCommands vendorsdep
import edu.wpi.first.wpilibj2.command.button.JoystickButton; //NewCommands vendordep
import static edu.wpi.first.wpilibj.XboxController.Button;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

// Subsystem imports
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LiftPivot;

// Command imports
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IndexCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LauncherSpeed;
import frc.robot.commands.LiftCommand;
import frc.robot.commands.LiftPivotCommand;

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
  private final LiftPivot liftPivotMotors = new LiftPivot();

  // Main driver controller
  private final XboxController driverController = new XboxController(0);
  // Second operator controller
  private final XboxController operatorController = new XboxController(1);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(driverController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driverController.getLeftX()) *  Constants.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(driverController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));
    

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /// Declaring buttons on driver controller
    final JoystickButton d_backButton = new JoystickButton(driverController, Button.kBack.value);
    // final JoystickButton d_startButton = new JoystickButton(driverController, Button.kStart.value);
    final JoystickButton d_ButtonA = new JoystickButton(driverController, Button.kA.value);
    final JoystickButton d_ButtonB = new JoystickButton(driverController, Button.kB.value);
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
    //final JoystickButton op_RightBumper = new JoystickButton(operatorController, Button.kRightBumper.value);
    //final JoystickButton op_LeftBumper = new JoystickButton(operatorController, Button.kLeftBumper.value);
    //final double op_LeftTrigger = operatorController.getLeftTriggerAxis();
    //final double op_RightTrigger = operatorController.getRightTriggerAxis();

    // Defining the actions associated with buttons
    d_backButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope); // Shaun's gyro reset 

    d_ButtonA.whenPressed(new LauncherSpeed(launcher, 0.30, 030)); // High shot from a distance. No longer need negative values
    d_ButtonA.whenReleased(new LauncherSpeed(launcher, 0.0, 0.00));

    d_ButtonB.whenPressed(new LauncherSpeed(launcher, 0.30, 0.30)); // Low shot from a up close
    d_ButtonB.whenReleased(new LauncherSpeed(launcher, 0.0, 0.00));

    d_ButtonX.whenPressed(new LauncherSpeed(launcher, 0.30, 0.40)); // High shot up close
    d_ButtonX.whenReleased(new LauncherSpeed(launcher, 0.0, 0.00));

    d_RightBumper.whenPressed(new IntakeCommand(intakeMotor, 0.5)); // Intake cargo from the field
    d_RightBumper.whenReleased(new IntakeCommand(intakeMotor, 0.0)); 

    d_LeftBumper.whenPressed(new IntakeCommand(intakeMotor, -0.5)); // Reverse cargo back to the field
    d_LeftBumper.whenReleased(new IntakeCommand(intakeMotor, 0.0)); 

    d_ButtonY.whenPressed(new IndexCommand(indexMotors, 0.5)); // Advance cargo to the launcher
    d_ButtonY.whenReleased(new IndexCommand(indexMotors, 0));

    //lift
    op_ButtonA.whenPressed(new LiftCommand(liftMotors, 0.1)); // Releases the lift arms for extension
    op_ButtonA.whenReleased(new LiftCommand(liftMotors, 0.0));
    
    op_ButtonB.whenPressed(new LiftCommand(liftMotors, -0.1)); // Pulls the arms back down for climbing
    op_ButtonB.whenReleased(new LiftCommand(liftMotors, 0.0));
    
    op_ButtonX.whenPressed(new LiftPivotCommand(liftPivotMotors, 0.1)); // Rotates the pivot-lift arms for the higher bar
    op_ButtonX.whenReleased(new LiftPivotCommand(liftPivotMotors, 0.0));
  
    op_ButtonY.whenPressed(new LiftPivotCommand(liftPivotMotors, -0.1)); // Rotates the pivot-lifts arms back towards the robot
    op_ButtonY.whenReleased(new LiftPivotCommand(liftPivotMotors, 0.0));
  
    //op_backButton.whenPressed(new LiftCommand(liftMotors, 0.0, 0.0));
    //op_backButton.whenReleased(new LiftCommand(liftMotors, 0.0, 0.0));
    
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







   
  public Command getAutonomousCommand() {
   //#region trajectory
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.MAX_VELOCITY_METERS_PER_SECOND, //original called for max speed (just in case im making one of the dumb physics mistakes)
            Constants.MAX_ACCELERATION_METERS_SECOND_SQUARED)
                    .setKinematics(Constants.m_kinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            m_drivetrainSubsystem::getPose,
            Constants.m_kinematics,
            xController,
            yController,
            thetaController,
            m_drivetrainSubsystem::setModuleStates,//?
            m_drivetrainSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
            new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> m_drivetrainSubsystem.stop()));

   //#endregion






   /*  //#region pathplanner
    
   
    // new AutonSquare(m_drivetrainSubsystem);
    // This is from Prototype launcher
    // return AutonSquare;
    // This is from SDS Drive code base
    //return new InstantCommand();

    // 1. This will load the file "Square.path" from PathPlanner and generate it with a max velocity of 8 m/s 
    // and a max acceleration of 5 m/s^2

    Trajectory examplePath = PathPlanner.loadPath("BadPath",8,5);

    // 2. Defining PID Controllers for tracking trajectory
    PIDController xController = new PIDController(Constants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
    Constants.kPThetaController, 0, 0, Constants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 3. Command to follow path from PathPlanner
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      examplePath, 
      m_drivetrainSubsystem::getPose, 
      Constants.m_kinematics, 
      xController, 
      yController, 
      thetaController, 
      m_drivetrainSubsystem::setModuleStates, 
      m_drivetrainSubsystem);

    // 4. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> m_drivetrainSubsystem.resetOdometry(examplePath.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> m_drivetrainSubsystem.stop()));
       //#endregion */
  }
}

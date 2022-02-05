// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.buttons.Trigger;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.buttons.JoystickButton; // OldCommands vendorsdep
import edu.wpi.first.wpilibj2.command.button.JoystickButton; //NewCommands vendordep
// import edu.wpi.first.wpilibj2.command.button.Button;
import static edu.wpi.first.wpilibj.XboxController.Button;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleAuton;
// import frc.robot.commands.LaunchCargo;
import frc.robot.commands.LauncherSpeed;
import frc.robot.subsystems.Launcher;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.commands.IntakeSpeed;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Launcher launcher = new Launcher();

  // Main driver controller
  private final XboxController driverController = new XboxController(0);
  // Second operator controller
  private final XboxController operatorController = new XboxController(1);

  // private Intake intake = new Intake();
  
  // Robot Commands
  // private final LaunchCargoLow m_autoCommand = new LaunchCargoLow(m_launcherSubsystem);

  // private final XboxController m_joystick = new XboxController(0);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
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
    

    // Configure the button bindings
    configureButtonBindings();
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  // FIXME reset the gyro (navx) ideas
  /** Code from 2021 Skills bot of Enginerds 2337 
  public void resetDrivetrain (){
    swerveDrivetrain.resetAngleMotors();
    swerveDrivetrain.resetOdometry();
    swerveDrivetrain.resetDriveMotos();
  }
  // backButton.whenPressed(() -> swerveDrivetrain.resetDriveMotors());   // Using the Engingerds RobotContainer.java line 136


  */
  

  private void configureButtonBindings() {
    /// Declaring buttons on driver controller
    final JoystickButton d_backButton = new JoystickButton(driverController, Button.kBack.value);
    final JoystickButton d_startButton = new JoystickButton(driverController, Button.kStart.value);
    final JoystickButton d_ButtonA = new JoystickButton(driverController, Button.kA.value);
    final JoystickButton d_ButtonB = new JoystickButton(driverController, Button.kB.value);
    final JoystickButton d_ButtonX = new JoystickButton(driverController, Button.kX.value);
    final JoystickButton d_ButtonY = new JoystickButton(driverController, Button.kY.value);
    final JoystickButton d_RightBumper = new JoystickButton(driverController, Button.kRightBumper.value);
    final JoystickButton d_LeftBumper = new JoystickButton(driverController, Button.kLeftBumper.value);
    final double d_LeftTrigger = driverController.getLeftTriggerAxis();
    final double d_RightTrigger = driverController.getRightTriggerAxis();

    // Declaring buttons on the operator controller
    final JoystickButton op_backButton = new JoystickButton(operatorController, Button.kBack.value);
    final JoystickButton op_startButton = new JoystickButton(operatorController, Button.kStart.value);
    final JoystickButton op_ButtonA = new JoystickButton(operatorController, Button.kA.value);
    final JoystickButton op_ButtonB = new JoystickButton(operatorController, Button.kB.value);
    final JoystickButton op_ButtonX = new JoystickButton(operatorController, Button.kX.value);
    final JoystickButton op_ButtonY = new JoystickButton(operatorController, Button.kY.value);
    final JoystickButton op_RightBumper = new JoystickButton(operatorController, Button.kRightBumper.value);
    final JoystickButton op_LeftBumper = new JoystickButton(operatorController, Button.kLeftBumper.value);
    final double op_LeftTrigger = operatorController.getLeftTriggerAxis();
    final double op_RightTrigger = operatorController.getRightTriggerAxis();

    // Defining the actions associated with buttons cargo -- these are just suggested 1-30-22
    // Modeled after ENGINERDS "Intake" is a Command class, "intake" is a variable that makes a new IntakeSubsystem defined aboved
    op_ButtonA.whenPressed(new LauncherSpeed(launcher, 0.30, -030)); // High shot from a distance
    op_ButtonA.whenReleased(new LauncherSpeed(launcher, 0.0, 0.00));

    op_ButtonB.whenPressed(new LauncherSpeed(launcher, 0.30, -0.30)); // Low shot from a up close
    op_ButtonB.whenReleased(new LauncherSpeed(launcher, 0.0, 0.00));

    op_ButtonX.whenPressed(new LauncherSpeed(launcher, 0.30, -0.40)); // High shot up close
    op_ButtonX.whenReleased(new LauncherSpeed(launcher, 0.0, 0.00));

    op_ButtonY.whenPressed(new LauncherSpeed(launcher, 0.00, 0.00));
    op_ButtonY.whenReleased(new LauncherSpeed(launcher, 0.0, 0.00));
   
    // Connect the buttons to commands
    // Launch the Cargo when either left bumper or right bumper is pressed
    // We tried whileHeld command initially, but it only starts the motors, it does not stop the motors automatically
    // upon button release as it should
    // this is working to start falcon motors when button held
    
    // Cargo low shot on Hub
    // lBumper.whenPressed(new LaunchCargoLow(m_launcherSubsystem)); // replace "m_launcherSubsystem" with speed variable?
    // lBumper.whenReleased(new StopLaunch(m_launcherSubsystem));

    // Cargo high shot on Hub
    // rBumper.whenPressed(new LaunchCargoHigh(m_launcherSubsystem));
    // rBumper.whenReleased(new StopLaunch(m_launcherSubsystem));
    // added a when button released command until we have whileHeld working as it should
    

  /** Use this to pass the autonomous command to the main {@link Robot} class.
  * @return the command to run in autonomous
 */
  // Shaun's working code for reseting gyro
  d_backButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope);  

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
    // An ExampleCommand will run in autonomous
    // new LaunchCargo(m_launcherSubsystem);
    // This is from Prototype launcher
    // return m_autoCommand;

    // This is from SDS Drive code base
    //return new InstantCommand();

    // 1. Creating trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
      DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 
      DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND)
        .setKinematics();

    // 2. This will load the file "Square.path" from PathPlanner and generate it with a max velocity of 8 m/s 
    // and a max acceleration of 5 m/s^2
    Trajectory examplePath = PathPlanner.loadPath("Square", 8, 5);

    // 3. Defining PID Controllers for tracking trajectory
    PIDController xController = new PIDController(kp, ki, kd);

    // 4. Command to follow path from PathPlanner
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
      examplePath, 
      pose, 
      kinematics, 
      xController, 
      yController, 
      thetaController, 
      outputModuleStates, 
      requirements);
  }
}

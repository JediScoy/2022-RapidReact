// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.buttons.JoystickButton; // OldCommands vendorsdep
import edu.wpi.first.wpilibj2.command.button.JoystickButton; //NewCommands vendordep
// import edu.wpi.first.wpilibj2.command.button.Button;
import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
// import frc.robot.commands.LaunchCargoLow;
// import frc.robot.commands.LaunchCargoHigh;
// import frc.robot.commands.StopLaunch;
// import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystem;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  // Main driver controller
  private final XboxController driverController = new XboxController(0);
  // Second operator controller
  private final XboxController operatorController = new XboxController(1);

  // private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();

  // Robot Commands
  // private final LaunchCargoLow m_autoCommand = new LaunchCargoLow(m_launcherSubsystem);

  // private final XboxController m_joystick = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
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

    // Reset the navx
    
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  /** Code from 2021 Skills bot of Enginerds 2337 
  public void resetDrivetrain (){
    swerveDrivetrain.resetAngleMotors();
    swerveDrivetrain.resetOdometry();
    swerveDrivetrain.resetDriveMotos();
  }
  */
  private void configureButtonBindings() {
    //declaring buttons on controller

    // Doesn't work "Cannot instantiate the type ... final Button backButton = new Button(driverController, XboxController.Button.kBack.value);
    final JoystickButton backButton = new JoystickButton(driverController, XboxController.Button.kBack.value);


    final JoystickButton rBumper = new JoystickButton(operatorController, Button.kRightBumper.value);
    final JoystickButton lBumper = new JoystickButton(operatorController, Button.kLeftBumper.value);
    final JoystickButton greenA = new JoystickButton(operatorController, Button.kA.value);

    // FIXME rBumper.whePressed(new Intake(IntakeSubsystem, 0));

    /** COMMENTING OUT LAUNCHER CODE FOR PRACTICE BOT
    // Connect the buttons to commands
    // Launch the Cargo when either left bumper or right bumper is held
    // left bumper = low shot, right bumper = high shot
    // We tried whileHeld command initially, but it only starts the motors, it does not stop the motors automatically
    upon button release as it should
    //this is working to start falcon motors when button held
    lBumper.whenHeld(new LaunchCargoLow(m_launcherSubsystem));
    rBumper.whenHeld(new LaunchCargoHigh(m_launcherSubsystem));
    //added a when button released command until we have whileHeld working as it should
    lBumper.whenReleased(new StopLaunch(m_launcherSubsystem));
    rBumper.whenReleased(new StopLaunch(m_launcherSubsystem));
    **/

  /** Use this to pass the autonomous command to the main {@link Robot} class.
  * @return the command to run in autonomous
 */

  // Back button zeros the gyroscope
  // Shaun's previously working code (?) that we broke :)
  // new Button(driverController::getBackButton)
            // No requirements because we don't need to interrupt anything
  //          .whenPressed(m_drivetrainSubsystem::zeroGyroscope);    

  // Using the Engingerds RobotContainer.java line 136
  // backButton.whenPressed(() -> swerveDrivetrain.resetDriveMotors());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // new LaunchCargoLow(m_launcherSubsystem);
    // This is from Prototype launacher
    // return m_autoCommand;

    // This is from SDS Drive code base
    return new InstantCommand();
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
}

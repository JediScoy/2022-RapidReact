package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;


public class StopDrive extends CommandBase {
    private final DrivetrainSubsystem subsystem;
    
    /**
     * Sets the intake speed to a certain amount
     * @param subsystem The Intake subsystem used by this command
     * @param speed The speed to set the intake to (a value between -1 and 1)
     */
    public StopDrive(DrivetrainSubsystem subsystem) {
      this.subsystem = subsystem;
      addRequirements(subsystem);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    @Override
    public void end(boolean interrupted) {
      subsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

}

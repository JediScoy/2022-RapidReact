package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class XModeShaun extends CommandBase {
    private boolean isfin;



public XModeShaun(Drivetrain subsystem) {
    addRequirements(subsystem);
}



    @Override
    public void execute() {
        
        SwerveModuleState[] states = new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        };
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return isfin;
    }
    public void setDisabled()
   {
        isfin = true;
    }

}

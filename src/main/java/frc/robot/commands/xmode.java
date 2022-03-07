package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class xmode extends CommandBase {
    boolean isFin = false;
    Drivetrain driveTrain;
    public xmode(Drivetrain subsystem) {
        this.driveTrain = subsystem;
        addRequirements(subsystem);
    }
    
   
    @Override
    public void execute() {
        //sets the modules to unmoving, "x" shape
        //btw X-mode sounds better than anything else we'll come up with for this don't rename it im begging
        driveTrain.setModuleStates(
                new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45))
            });
        
    }

    @Override
    public boolean isFinished() {
        return isFin;
    }
    
    
    
    
 
    
}

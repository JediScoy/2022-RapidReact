package frc.robot.commands.auton;
import java.util.List;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.BotchAuton1Input;
import frc.robot.subsystems.DrivetrainSubsystem;



public class botchAuton extends CommandBase {
    /**the coords to for the robot to attempt to replicate; note this will be a shallow attempt, and may need tuning forward*/
    //#region properties
    List<BotchAuton1Input> coords;
    //this timer is the wpilib version, if it doesnt work its the build teams fault
    Timer timer;
    
    //#endregion
    private DrivetrainSubsystem driveSubsytem;
    private boolean isFin;
    

/**
 * @param subsystem The subsystem
 * @param coords the Inputs to use 
 * 
 */
 public botchAuton(DrivetrainSubsystem subsystem, List<BotchAuton1Input> coords) {

     addRequirements(subsystem);
     this.driveSubsytem = subsystem;
     this.coords = coords;//'this' keyword refers to the instance of the class, not the parameter. Technically 'lesser' practice 
 }
    
    @Override
    public void execute() 
        {
            for (BotchAuton1Input currentInput : coords) {
                timer.start();
                while (timer.get() > currentInput.interval)//measured in seccos 
                {
                    driveSubsytem.drive(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                                currentInput.x,
                                currentInput.y,
                                currentInput.theta,
                                driveSubsytem.getGyroscopeRotation()
                        )
                );


                }
            
            isFin = true;
            }
        }

    @Override
    public boolean isFinished() {
        
        return isFin;
    }

   //end code not currently in place, no need?
}

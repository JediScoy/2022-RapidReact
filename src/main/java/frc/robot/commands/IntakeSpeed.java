package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeLower;

/**
 * A simple command that grabs a hatch with the {@link HatchSubsystem}. Written explicitly for
 * pedagogical purposes. Actual code should inline a command this simple with {@link
 * edu.wpi.first.wpilibj2.command.InstantCommand}.
 */

public class IntakeSpeed extends CommandBase {
  
    
    // The subsystem the command runs on
    private final IntakeLower subsystem;
    private double speed;
  
    public IntakeSpeed(IntakeLower subsystem, double speed) {
      this.subsystem = subsystem;
      this.speed = speed;
      addRequirements(subsystem); // clever way to call in case you change the name of the subsystem
    }
  
    @Override
    public void initialize() {
      // FIXME subsystem.IntakeLower(speed);
    }
  
    @Override
    public void execute() {
      // FIXME subsystem.IntakeLower(speed);
    }
  
    @Override
    public void end(boolean interrupted){ 
      // FIXME subsystem.setSpeed(0.0);
    }
    
    
  }
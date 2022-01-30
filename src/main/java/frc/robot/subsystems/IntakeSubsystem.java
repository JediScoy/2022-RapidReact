
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Displaying data?

// CTRE imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import frc.robot.Constants;



public class IntakeSubsystem extends SubsystemBase {
  
  private TalonFX intakeMotor;


  /** Creates a Subsystem using Falcon 500s controlled by TalonFX.
    * These are the two motors for [ ]
**/

  /** Uncomment setup for each specific subystem with motors
  private final MotorController m_IntakeMotors = 
    /** new MotorControllerGroup(
        * new TalonFX(LauncherConstants.leftLaunchMotor),
        * new TalonFX(LauncherConstants.rightLaunchMotor));
  */
    
// final TalonFXInvertType rightLaunchMotor = TalonFXInvertType.CounterClockwise;

 
public IntakeSubsystem() {
  intakeMotor = new TalonFX(Constants.INTAKE);
  intakeMotor.setInverted(true);
  intakeMotor.setNeutralMode(NeutralMode.Coast);

}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Dashboard stuff would go here
  }

  // Sets the Intake system speed as a percentage between -1 and 1. Speed is given actual value in the RobotContrainer.java button code
  public void setIntakeSpeed(double speed){
    intakeMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


} // End of Class


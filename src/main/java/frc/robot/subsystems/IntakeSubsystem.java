
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



public class IntakeSubsystem extends SubsystemBase {
  
  // FIXME private TalonFX intakeMotor;


  /** Creates a Subsystem using Falcon 500s controlled by TalonFX.
    * These are the two motors for [ ]
**/

  /** Uncomment setup for each specific subystem with motors
  private final MotorController m_IntakeMotors = 
    /** new MotorControllerGroup(
        * new WPI_TalonFX(LauncherConstants.leftLaunchMotor),
        * new WPI_TalonFX(LauncherConstants.rightLaunchMotor));
  */
    
// final TalonFXInvertType rightLaunchMotor = TalonFXInvertType.CounterClockwise;
// FIXME

/** 
public Intake(){
  intakeMotor = new TalonFX(Constants.INTAKE);

}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSpeed(double speed){
    // intakeMotor.set(controlMode.PercentOutput, speed);
  }

  public void stopIntake(){
    return intakeMotor.set(ControlMode.PercentOutput, 0)
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
*/

} // End of Class


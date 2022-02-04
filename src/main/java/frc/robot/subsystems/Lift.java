
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import frc.robot.Constants;

// Shuffleboard imports
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard; // Displaying data?
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab; // Displaying data?

// CTRE imports
// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

public class Lift extends SubsystemBase {
/** Creates a Subsystem using Falcon 500s controlled by TalonFX.
    * These are the two motors for [ ]
**/
  /** 
  private final TalonFX liftMotor1;
  private final TalonFX liftMotor2;
  */

  /** Uncomment setup for each specific subystem with motors
  private final MotorController m_IntakeMotors = 
    /** new MotorControllerGroup(
        * new TalonFX(LauncherConstants.liftMotor1),
        * new TalonFX(LauncherConstants.liftMotor2));
  */
    
// final TalonFXInvertType rightLaunchMotor = TalonFXInvertType.CounterClockwise;
  /**
  public LiftSubsystem() {
    liftMOTOR = new TalonFX(Constants.LIFT_1, LIFT_2);
    liftMotor.setInverted(true);
    liftMotor.setNeutralMode(NeutralMode.Coast);
  */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

} // End of Class
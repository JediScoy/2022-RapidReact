package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//not needed?
//encoder class to use?

//import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CTREEncoder;
import frc.robot.Constants;


public class SwerveModule {

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;

    private final CTREEncoder driveEncoder;
    private final CTREEncoder turningEncoder;

    private final PIDController turningPidController;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed) {

        driveMotor = new WPI_TalonFX(driveMotorId);
        turningMotor = new WPI_TalonFX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        //NOTE: we dont actually know if this encoder will suffice. It does well as an implementation of an encoder, but two questions remain:
        /*1) does it properly init. to the motor?
        *2) can it be used where encoders are normally used, due to type issue? If not, can we find a way to convert the data to a compatible encoder class?
        *
        */
        driveEncoder =  new CTREEncoder(driveMotor, false, 0);
        turningEncoder =  new CTREEncoder(turningMotor, false, 0);

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(Constants.kPTurning, 0, 0);//the last 2 represent no adjustments on the last 2 terms
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
       // return driveEncoder.getPosition();
       return driveEncoder.getDistance();//are distance and velocity the same?
       //TODO: either determine use of an encoder or odometry
    }

    public double getTurningPosition() {
        return turningEncoder.getDistance();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    
    public void resetEncoders() {
       //quite possibly redundant
        driveEncoder.setPosition(0);
       
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.kPhysicalMaxSpeedMetersPerSecond);//TODO: Record max speed
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + turningEncoder.getChannel() + "] state", state.toString());//this originally called getchannel() for the absoluteencoder, rather than this
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}
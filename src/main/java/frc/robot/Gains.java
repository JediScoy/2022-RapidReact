package frc.robot;
/**
 * Wrapper for constants used in motionmagic, stolen
 */
public class Gains {
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
	
	public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
        //f-gain, gained by (out% * 1023/out%*max), where out% is the desired percentage of maximum power(i.e .50)
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}
}
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public class Interpolator {

    //This is an ordered list of "distance" measurements (this example is the y-angle reported by the Limelight).
    //This must be monotonically decreasing

    // FIXME Find these values from Limelight
    private static double angles[] = {20.00, 18.00, 11.75, 3.75};
    
    //This is an ordered list of "speed" outputs (this example is for a PID loop being fed raw encoder values)
    // FIXME Find through trial and error
    private static double speeds[] = {0.30, 0.35, 0.40, 0.45};
    
    // The front and back launch motors need to have different speeds,
    // so that's why there is a front and back speed
    // Speed of the front launcher motor
    private static double speedFront[] = {0.30, 0.35, 0.40, 0.30}; 
    // Speed of the back launcher motor
    private static double speedBack[] = {0.35, 0.40, 0.45, 0.60};
    
    //If it consistently shoots too low, this variable is used to correct that
    private static double m_offset = 0;

    /**
     * Static method that takes in the "distance" measurement and outputs "speed"
     * @param Distance measurement in "distance" units
     * @returns Speed in "speed" units
     */
    public static double getInterpolation(double angle) {

        //Search through the angles[] array to find the two values that the current
        //measurement is between (this is like finding two points on a graph)
        int i = 0;
        while(angles[i] > angle) {
            i++;

            //If it didn't find two points, return 0
            if(i >= angles.length) {
                return 0;
            }
        }
        
        if(i == 0) {
            return speeds[i];
        }

        //Make a line from the points (angles[i-1], speeds[i-1]) and (angles[i], speeds[i])
        //in the form y = mx+b
        int i1 = i-1;
        int i2 = i;
        double m = (speeds[i2]-speeds[i1])/(angles[i2]-angles[i1]);
        double b = speeds[i1] - m*angles[i1];

        //Find where the recorded point fits on that line, that's the speed!
        double interpolation = m*angle + b + m_offset;
        return interpolation;
    }

    /**
     * Static method used to set a constant offset (for chronic over or under shooting)
     * @param Offset in "speed" units
     */
    public static void setOffset(double offset) {
        m_offset = offset;
    }
}

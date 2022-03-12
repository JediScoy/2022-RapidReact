package frc.robot.commands;

import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class XLaunch extends ParallelDeadlineGroup{

    public XLaunch(int time, Command launchcommand, Command xmode) {
        
        super((Command)(new WaitCommand(time)), launchcommand, xmode);
    }


    public XLaunch(Command deadline, Command[] commands) {
        super(deadline, commands);

    }



    
    
}

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class IntakeRunFeeder extends Command {

	public boolean beamBreak;
	public boolean beamBreak2;
    
    public IntakeRunFeeder() {
        addRequirements(RobotContainer.feeder);
    }
	// Called just before this Command runs the first time
	public void initialize() {

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		//read sensor
		beamBreak = RobotContainer.feeder.readInput();
		beamBreak2 = RobotContainer.feeder.readInput2();
		
		//stop feeder if beam broken
		if (beamBreak == false || beamBreak2 == false) {
			RobotContainer.feeder.setSpeed(0.0);
			RobotContainer.candleSubsystem.setAnimate("Orange");
		}

		else {
			RobotContainer.feeder.setSpeed(Constants.Feeder.IntakeSpeed);
		}
        
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return beamBreak == false || beamBreak2 == false;
	}

	// Called once after isFinished returns true
	protected void end() {
		RobotContainer.feeder.setSpeed(0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}



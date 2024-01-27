package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class RunFeeder extends Command {

	private double shooterAngle = 0;
    
    public RunFeeder() {
        addRequirements(RobotContainer.feeder);
    }
	// Called just before this Command runs the first time
	public void initialize() {
		shooterAngle = RobotContainer.shooterWrist.getCurrentPosition();

	}

	// Called repeatedly when this Command is scheduled to run
	public void execute() {
		if (shooterAngle <= 76.5 && shooterAngle >= 74.5) {
			RobotContainer.feeder.setSpeed(-1);
		}
		
		else {
			RobotContainer.feeder.setSpeed(1);

		}
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}



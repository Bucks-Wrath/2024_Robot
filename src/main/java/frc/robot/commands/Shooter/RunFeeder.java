package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
		if (shooterAngle <= 75.0 && shooterAngle >= 74.4) {
			RobotContainer.feeder.setSpeed(Constants.Feeder.RearEjectSpeed);
			RobotContainer.candleSubsystem.setAnimate("Purple");
		}
		
		else {
			RobotContainer.feeder.setSpeed(Constants.Feeder.FeedShooterSpeed);
			RobotContainer.candleSubsystem.setAnimate("Purple");
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
			RobotContainer.feeder.setSpeed(0);
			RobotContainer.candleSubsystem.setAnimate("Purple");

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}



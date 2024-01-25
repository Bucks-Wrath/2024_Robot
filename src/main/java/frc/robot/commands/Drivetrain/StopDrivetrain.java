package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class StopDrivetrain extends Command {    
    private Swerve s_Swerve;    
    private Boolean robotCentricSup;

    public StopDrivetrain(Swerve s_Swerve, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {

    }
    
    @Override
    public void execute() {

        s_Swerve.drive(
        new Translation2d(0, 0.0).times(Constants.Swerve.maxSpeed), 
        0 * Constants.Swerve.maxAngularVelocity, 
        !robotCentricSup, 
        true
        );  

    }

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

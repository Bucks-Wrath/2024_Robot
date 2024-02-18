package frc.robot.commands.Intake;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.Drivetrain.StopDrivetrain;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class VisionAlignIntake extends Command {    
    private Swerve s_Swerve;    
    private Boolean robotCentricSup;

    private double tx;
    private double ty;
    private double ta;  // removed initial value

    private final PIDController angleController = new PIDController(0.005, 0, 0);
    private final PIDController slideController = new PIDController(0.007, 0, 0);  // added for better PID tuning
    private double targetAngle = 0;
    private final PIDController distanceController = new PIDController(0.025, 0, 0); // was 0.1
    private double targetArea = 14;  // what is the area when we pick up gp?
    private double targetTy = -10;

    public VisionAlignIntake(Swerve s_Swerve, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        addRequirements(RobotContainer.rearLimelight);
        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {
        tx = RobotContainer.rearLimelight.getX();
        ta = RobotContainer.rearLimelight.getArea();
        ty = RobotContainer.rearLimelight.getY();

        angleController.setTolerance(0.05);  // needs to be checked
        slideController.setTolerance(0.05);
        distanceController.setTolerance(0.05);
    }
    
    @Override
    public void execute() {
        // Find target location
        tx = RobotContainer.rearLimelight.getX();
        ta = RobotContainer.rearLimelight.getArea();
        ty = RobotContainer.rearLimelight.getY();


        if (ta > 1 && ta < 20) {
            double rotationVal = angleController.calculate(tx,targetAngle);
            double strafeVal = -slideController.calculate(tx,targetAngle);
            double translationVal = distanceController.calculate(ty,targetTy);

            /* Drive */
            if (ty > -8) { // ta < 11
                s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup, 
                true
                );  
            }
        
            else {

            }
        }

        else {

        }

    }

    public boolean isFinished() {
		return ty < -8;
	}

	// Called once after isFinished returns true
	protected void end() {
        new StopDrivetrain(s_Swerve, true);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
        end();
	}
}

package frc.robot.commands.Shooter;

import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoVisionShoot extends Command {    
    private double ty;
    private double ta;
    private double ySpeed;
    private double xSpeed;

    private double yShooterAngle = 0;
    private double aShooterAngle = 0;
    private double shooterAngle = 0;


    public AutoVisionShoot() {
        addRequirements(RobotContainer.frontLimelight);
        addRequirements(RobotContainer.leftShooter);
        addRequirements(RobotContainer.rightShooter);
        addRequirements(RobotContainer.shooterWrist);
    }

    public void initialize() {
        ty = RobotContainer.frontLimelight.getY();
        ta = RobotContainer.frontLimelight.getArea();
    }
    
    @Override
    public void execute() {
        // find target location
        ty = RobotContainer.frontLimelight.getY();
        ta = RobotContainer.frontLimelight.getArea();

        // Uses ta to set shooter angle
        // Caleb Numbers, just for comparison: -17.1136, 41.1436, -2.7548
        aShooterAngle = (-17.3601*ta*ta) + (41.5424*ta) - (2.82088);

        // use ty to calculate shooter angle
        // Caleb Numbers, just for comparison: -.0091, 0.7406, 18.3463
        yShooterAngle = (-0.009811884*ty*ty) + (0.740631*ty) + (18.3463);

        // average data from both equations
        shooterAngle = ((aShooterAngle + yShooterAngle) / 2) - xSpeed;  // is this going the right way and is it the right value

        // disallow any negative values
        if (shooterAngle <= 0) {
            shooterAngle = 0;
        }

        else{

        }

        // Sets Shooter angle and speed
        RobotContainer.leftShooter.setTargetVelocity(Shooter.DefaultShotVelocity.VelocityLeft);
        RobotContainer.rightShooter.setTargetVelocity(Shooter.DefaultShotVelocity.VelocityRight);

        // Controls shooter speed and angle
        RobotContainer.leftShooter.velocityControl();
        RobotContainer.rightShooter.velocityControl();

        if (ta > 0) { // only set shooter if has target
            RobotContainer.shooterWrist.setTargetPosition(shooterAngle);
            RobotContainer.shooterWrist.motionMagicControl();
        }

        else {

        }
    }
}

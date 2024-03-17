package frc.robot.commands.Shooter;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants.Shooter;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoVisionShoot extends Command {    
    private double ty;
    private double ta;
    private double ySpeed;
    private double xSpeed;
    private Swerve s_Swerve;    

    private double yShooterAngle = 0;
    private double aShooterAngle = 0;
    private double shooterAngle = 0;

    private double shooterAddValue = 0;

    public AutoVisionShoot(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;

        addRequirements(RobotContainer.frontLimelight);
        addRequirements(RobotContainer.leftShooter);
        addRequirements(RobotContainer.rightShooter);
        addRequirements(RobotContainer.shooterWrist);
    }

    public void initialize() {
        ty = RobotContainer.frontLimelight.getY();
        ta = RobotContainer.frontLimelight.getArea();
        shooterAddValue = RobotContainer.shooterWrist.getCValue();
    }
    
    @Override
    public void execute() {
        // find target location
        ty = RobotContainer.frontLimelight.getY();
        ta = RobotContainer.frontLimelight.getArea();

        xSpeed = s_Swerve.xSpeed()*0.85;  // does this need to be larger

        // Uses ta to set shooter angle
        // Eggo (-17.3601*ta*ta) + (41.5424*ta) - (4); // - 2.82088
        aShooterAngle = (-7.3996*ta*ta) + (34.3482*ta) - 0.55 + shooterAddValue; // - 1.38093  was -1.15 


        // use ty to calculate shooter angle
        // Eggo (-0.009811884*ty*ty) + (0.740631*ty) + (17);  // 18.3463
        //yShooterAngle = (-0.00280493*ty*ty) + (0.986053*ty) + (22.5) + shooterAddValue;  // 21.3444 was 22.5

        // average data from both equations
        shooterAngle = ((aShooterAngle + aShooterAngle) / 2) + xSpeed;
        
        // is this going the right way and is it the right value
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
        
        RobotContainer.shooterWrist.setTargetPosition(shooterAngle);
        RobotContainer.shooterWrist.motionMagicControl();
            

    }
}

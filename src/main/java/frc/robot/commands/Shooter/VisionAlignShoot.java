package frc.robot.commands.Shooter;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Shooter;
import frc.robot.subsystems.Swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class VisionAlignShoot extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private Boolean robotCentricSup;

    private double tx;
    private double ty;
    private double ta;
    private double tl;
    private double ts;
    private double ySpeed;
    private double xSpeed;
    private double rotationVal;

    private double slowSpeed = 0.5;
    private double shooterHeight = 0;

    private final PIDController angleController = new PIDController(0.0075, 0, 0);  //0.005
    private double targetAngle = 0;
    private double yShooterAngle = 0;
    private double aShooterAngle = 0;
    private double shooterAngle = 0;


    public VisionAlignShoot(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, Boolean robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        addRequirements(RobotContainer.frontLimelight);
        addRequirements(RobotContainer.leftShooter);
        addRequirements(RobotContainer.rightShooter);
        addRequirements(RobotContainer.shooterWrist);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.robotCentricSup = robotCentricSup;
    }

    public void initialize() {
        tx = RobotContainer.frontLimelight.getX();
        ty = RobotContainer.frontLimelight.getY();
        ta = RobotContainer.frontLimelight.getArea();
        tl = RobotContainer.frontLimelight.getLong();
        ts = RobotContainer.frontLimelight.getShort();
        angleController.setTolerance(0.05);  // needs to be tuned
    }
    
    @Override
    public void execute() {
        // Get y translation value
        ySpeed = s_Swerve.ySpeed()*9;
        xSpeed = s_Swerve.xSpeed()*1.5;  // does this need to be larger

        // adjust target x based on translation
        targetAngle = 0 - ySpeed;  // needs to be tuned

        // find target location
        tx = RobotContainer.frontLimelight.getX();
        ty = RobotContainer.frontLimelight.getY();
        ta = RobotContainer.frontLimelight.getArea();
        tl = RobotContainer.frontLimelight.getLong();
        ts = RobotContainer.frontLimelight.getShort();

        // Do math
        double diff = tl - ts;

        /* Get Values, Deadband*/
        double translationVal = -MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = -MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
 
        // Uses PID to point at target
        rotationVal = angleController.calculate(tx,targetAngle);

        // Uses ta to set shooter angle
        // Eggo (-17.3601*ta*ta) + (41.5424*ta) - (4); // - 2.82088
        aShooterAngle = (-7.3996*ta*ta) + (34.3482*ta) - 0.25; // - 1.38093  was -0.5

        // use ty to calculate shooter angle
        // Eggo (-0.009811884*ty*ty) + (0.740631*ty) + (17);  // 18.3463
        yShooterAngle = (-0.00280493*ty*ty) + (0.986053*ty) + (22.5);  // 21.3444 was 22.5

        // average data from both equations
        shooterAngle = ((aShooterAngle + yShooterAngle) / 2) + xSpeed;  // is this going the right way and is it the right value

        if (diff > 10) {
            shooterAngle = shooterAngle + 0.5;
        }

        else {

        }
        
        // disallow any negative values
        if (shooterAngle <= 0) {
            shooterAngle = 0;
        }

        else{

        }

        // calculate target error
        double targetError = Math.abs(targetAngle - tx);

        // signal to driver that we are ready to shoot
        if (targetError < 1 ) {  //&& RobotContainer.shooterWrist.isInPosition(shooterAngle)) {
            RobotContainer.candleSubsystem.setAnimate("Rainbow");
        }

        else {
            RobotContainer.candleSubsystem.setAnimate("Orange");
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup, 
            true
        );

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

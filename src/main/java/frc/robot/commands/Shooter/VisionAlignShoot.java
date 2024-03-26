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

    private double shooterHeight = 0;

    private final PIDController angleController = new PIDController(0.0065, 0, 0.0001);  //0.0065
    private double targetAngle = 0;
    private double yShooterAngle = 0;
    private double aShooterAngle = 0;
    private double shooterAngle = 0;
    private double shooterAddValue = 0;


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
        shooterAddValue = RobotContainer.shooterWrist.getCValue();
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
        //aShooterAngle = (-17.3601*ta*ta) + (41.5424*ta) - (4) + shooterAddValue; // eggo
        //aShooterAngle = (-7.3996*ta*ta) + (34.3482*ta) - 0.15 + shooterAddValue; // - 1.38093  was -0.75

        // use ty to calculate shooter angle
        //yShooterAngle = (-0.009811884*ty*ty) + (0.740631*ty) + (17) + shooterAddValue;  // eggo
        //yShooterAngle = (-0.00280493*ty*ty) + (0.986053*ty) + (21.6) + shooterAddValue;  // 21.3444 was 21 old dial shooter
        //yShooterAngle = (-0.00846863*ty*ty) + (0.783455*ty) + (17.7994) + shooterAddValue;  // v1
        yShooterAngle = (-0.0104986*ty*ty) + (0.80866*ty) + (18.8348) + shooterAddValue;  // v2


        // average data from both equations
        shooterAngle = ((yShooterAngle + yShooterAngle) / 2) + xSpeed;  // is this going the right way and is it the right value

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

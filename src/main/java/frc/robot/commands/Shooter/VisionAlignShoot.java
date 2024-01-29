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

    private double tx = 0;
    private double ty = 0;
    private double ta = 0;
    private double speed = 0;

    private double slowSpeed = 0.5;
    private double shooterHeight = 0;

    private final PIDController angleController = new PIDController(0.012, 0, 0);  //0.012
    private double targetAngle = 0;
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
        angleController.setTolerance(0.05);  // needs to be tuned
    }
    
    @Override
    public void execute() {
        // Get Shooter Height
        shooterHeight = RobotContainer.shooterWrist.getCurrentPosition();
        speed = s_Swerve.ySpeed();

        // find target location
        tx = RobotContainer.frontLimelight.getX(); //+ (10*speed);
        ty = RobotContainer.frontLimelight.getY();
        ta = RobotContainer.frontLimelight.getArea();

        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);

        // Uses PID to point at target
        double rotationVal = angleController.calculate(tx,targetAngle);

        // Uses ta and ty to set shooter angle
        if (ta > 0.11) {
            double input = Math.log(39.609*ta - 4.42716);
            double logE = Math.log(2.718281828);
            double output = input / logE;
            shooterAngle = (6.42789*output) - 4; // 1.76625
        }

        else {
            shooterAngle = 0;
        }

        // Make the robot slower if the shooter is up
        if (shooterHeight > 30){
            translationVal = translationVal*slowSpeed;
            strafeVal = strafeVal*slowSpeed;
            rotationVal = rotationVal*slowSpeed;
        }

        else {

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
        RobotContainer.shooterWrist.setTargetPosition(shooterAngle);

        // Controls shooter speed and angle
        RobotContainer.leftShooter.velocityControl();
        RobotContainer.rightShooter.velocityControl();
        RobotContainer.shooterWrist.motionMagicControl();
    }
}

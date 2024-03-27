package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Shooter.ShooterPose;
import frc.robot.auto.AutoSetGyro;
import frc.robot.auto.AutoZero;
import frc.robot.commands.Drivetrain.PIDTurnToAngle;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Intake.IntakeCommandGroup;
import frc.robot.commands.Intake.ManualIntakeCommandGroup;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.StopIntake;
import frc.robot.commands.Intake.StopIntakeCommandGroup;
import frc.robot.commands.LEDs.SetNeedNote;
import frc.robot.commands.Shooter.AutoRunFeeder;
import frc.robot.commands.Shooter.AutoVisionAlignShoot;
import frc.robot.commands.Shooter.AutoVisionShoot;
import frc.robot.commands.Shooter.CloseAutoVisionShoot;
import frc.robot.commands.Shooter.JoystickShooterWrist;
import frc.robot.commands.Shooter.RunFeeder;
import frc.robot.commands.Shooter.StopFeeder;
import frc.robot.commands.Shooter.VisionAlignShoot;
import frc.robot.commands.Shooter.CommandGroups.ShootFrom;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Autonomous Selector */
    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
    private final CommandXboxController DriverController = new CommandXboxController(0);
    private final CommandXboxController OperatorController = new CommandXboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Setting Bot to Field Centric */
    private final Boolean robotCentric = false;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton zeroFlipGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton faceLeftButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton faceRightButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton faceSourceButton = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton facePodiumButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton shootButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Operator Buttons */
    private final JoystickButton subwooferShotButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton podiumShotButton = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton ampShotButton = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton shooterDownButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton tallShotSubwooferButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton trapShotButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton PassButton = new JoystickButton(operator, XboxController.Button.kStart.value);


    /* Subsystems */
    private final Swerve swerve = new Swerve();
    public static LeftShooter leftShooter = new LeftShooter();
    public static RightShooter rightShooter = new RightShooter();
    public static ShooterWrist shooterWrist = new ShooterWrist();
    public static Intake intake = new Intake();
    public static Feeder feeder = new Feeder();
    public static FrontLimelight frontLimelight = new FrontLimelight();
    public static RearLimelight rearLimelight = new RearLimelight();
    public static CANdleSubsystem candleSubsystem = new CANdleSubsystem();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        registerNamedCommands();

        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");
        autoChooser.addOption("6 Note Auto V1", new PathPlannerAuto("6 Note Auto V1"));
        autoChooser.addOption("6 Note Auto V1B", new PathPlannerAuto("6 Note Auto V1B"));
        autoChooser.addOption("6 Note Auto V2", new PathPlannerAuto("6 Note Auto V2"));
        autoChooser.addOption("6 Note Auto V2B", new PathPlannerAuto("6 Note Auto V2B"));
        autoChooser.addOption("Long Side Skip Auto", new PathPlannerAuto("Long Side Skip Auto"));
        autoChooser.addOption("Long Side Skip AutoB", new PathPlannerAuto("Long Side Skip AutoB"));
        autoChooser.addOption("Long Side Auto V1", new PathPlannerAuto("Long Side Auto V1"));
        autoChooser.addOption("Long Side Auto V1B", new PathPlannerAuto("Long Side Auto V1B"));
        autoChooser.addOption("Long Side Auto V2", new PathPlannerAuto("Long Side Auto V2"));
        autoChooser.addOption("Long Side Auto V2B", new PathPlannerAuto("Long Side Auto V2B"));
        autoChooser.addOption("Short Side Auto V2", new PathPlannerAuto("Short Side Auto V2"));
        autoChooser.addOption("Short Side Auto V2B", new PathPlannerAuto("Short Side Auto V2B"));
        autoChooser.addOption("Short Side Auto V3", new PathPlannerAuto("Short Side Auto V3"));
        autoChooser.addOption("Short Side Auto V3B", new PathPlannerAuto("Short Side Auto V3B"));
        autoChooser.addOption("Short Side Auto V4", new PathPlannerAuto("Short Side Auto V4"));
        autoChooser.addOption("Short Side Auto V4B", new PathPlannerAuto("Short Side Auto V4B"));
        autoChooser.addOption("Short Side Skip Auto V1", new PathPlannerAuto("Short Side Skip Auto V1"));
        autoChooser.addOption("Short Side Skip Auto V1B", new PathPlannerAuto("Short Side Skip Auto V1B"));
        autoChooser.addOption("Short Side Skip Auto V2", new PathPlannerAuto("Short Side Skip Auto V2"));
        autoChooser.addOption("Short Side Skip Auto V2B", new PathPlannerAuto("Short Side Skip Auto V2B"));

        autoTab.add("Mode", autoChooser);

        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                robotCentric
            )
        );
        /* This section is for testing only
        // Shooter and wrists cannot both be controlled at the same time by joysticks */

        //leftShooter.setDefaultCommand(new JoystickShooter());
        //rightShooter.setDefaultCommand(new JoystickShooter());
        
        shooterWrist.setDefaultCommand(new JoystickShooterWrist());
        
        // Sets Default Commands for intake and feeder motors
        intake.setDefaultCommand(new StopIntake());
        feeder.setDefaultCommand(new StopFeeder());
 
        /* Configure the button bindings */
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroHeading()));    // Resets gyro
        zeroFlipGyro.onTrue(new InstantCommand(() -> swerve.autoFlipZeroGyro()));    // Resets gyro
        intakeButton.whileTrue(new IntakeCommandGroup(swerve));             // Runs intake w/ vision
        intakeButton.whileFalse(new StopIntakeCommandGroup());              // Stop intake w/ vision
        shootButton.whileTrue(new RunFeeder());  
        shootButton.whileFalse(new StopFeeder());                           // Stops shooter feeder
        DriverController.rightTrigger().onTrue(new ManualIntakeCommandGroup());


        faceLeftButton.whileTrue(new PIDTurnToAngle(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            () -> Constants.FieldAngle.getLeftAngle()));

        faceRightButton.whileTrue(new PIDTurnToAngle(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            () -> Constants.FieldAngle.getRightAngle()));

        
        facePodiumButton.whileTrue(new PIDTurnToAngle(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            //This is a temporary direction for testing out aliging with the source (as oriented from the Red Alliance)
            //We probably want to tie the source angle to a different button
            () -> Constants.FieldAngle.getPodiumAngle()));

        DriverController.leftTrigger().whileTrue(new VisionAlignShoot(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            robotCentric
        ));

        faceSourceButton.whileTrue(new PIDTurnToAngle(
            swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            robotCentric,
            //This is a temporary direction for testing out aliging with the source (as oriented from the Red Alliance)
            //We probably want to tie the source angle to a different button
            () -> Constants.FieldAngle.getSourceAngle()).alongWith(new SetNeedNote())); 
        
        /* Operator Buttons */
        subwooferShotButton.onTrue(new ShootFrom(ShooterPose.Subwoofer));
        podiumShotButton.onTrue(new ShootFrom(ShooterPose.Podium));
        ampShotButton.onTrue(new ShootFrom(ShooterPose.Amp));
        shooterDownButton.onTrue(new ShootFrom(ShooterPose.Home));//.alongWith(new ReverseFeeder()).withTimeout(1.5));    // Brings shooter back to start position and slows
        trapShotButton.onTrue(new ShootFrom(ShooterPose.TrapShot));
        tallShotSubwooferButton.onTrue(new ShootFrom(ShooterPose.SubwooferTall));
        OperatorController.leftTrigger().onTrue(new ShootFrom(ShooterPose.ClimbReady));
        OperatorController.rightTrigger().onTrue(new ShootFrom(ShooterPose.Climb));
        PassButton.onTrue(new ShootFrom(ShooterPose.PassShotPose));
    }

    /* Public access to joystick values */
    public Joystick getDriver() {
        return driver;
    }

    public Joystick getOperator() {
        return operator;
    }

    /* Sets Joystick Deadband */
    public static double stickDeadband(double value, double deadband, double center) {
        return (value < (center + deadband) && value > (center - deadband)) ? center : value;
    }
    
    /* Passes Along Joystick Values for Elevator and Wrist */
    public double getOperatorLeftStickY() {
        return stickDeadband(this.operator.getRawAxis(1), 0.05, 0.0);
    }
 
    public double getOperatorRightStickY() {
        return stickDeadband(this.operator.getRawAxis(5), 0.05, 0.0);
    }

    /* Runs the Autonomous Selector*/
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
        //return autonomousSelector.getCommand(swerve);
    }

    public void registerNamedCommands() {
        /* Command registration for PathPlanner */     
        NamedCommands.registerCommand("IntakeCommandGroup", new IntakeCommandGroup(swerve));
        NamedCommands.registerCommand("ManualIntakeCommandGroup", new ManualIntakeCommandGroup());
        NamedCommands.registerCommand("RunIntake", new RunIntake());
        NamedCommands.registerCommand("StopIntakeCommandGroup", new StopIntakeCommandGroup());
        NamedCommands.registerCommand("RunFeeder", new RunFeeder());
        NamedCommands.registerCommand("Shoot", new RunFeeder().withTimeout(0.4));
        NamedCommands.registerCommand("StopFeeder", new StopFeeder().withTimeout(0.1));
        NamedCommands.registerCommand("SubwooferShot", new ShootFrom(ShooterPose.Subwoofer).withTimeout(0.5));
        NamedCommands.registerCommand("AutoShoot", new AutoVisionShoot(swerve));
        NamedCommands.registerCommand("CloseAutoShoot", new CloseAutoVisionShoot(swerve));      
        NamedCommands.registerCommand("LongAutoShotPose", new ShootFrom(ShooterPose.LongAutoShotPose));
        NamedCommands.registerCommand("ShortSideAutoShotPose", new ShootFrom(ShooterPose.ShortSideAutoShotPose).withTimeout(1.75)); 
        NamedCommands.registerCommand("ShortSideAuto2ShotPose", new ShootFrom(ShooterPose.ShortSideAuto2ShotPose));
        NamedCommands.registerCommand("ShortSideAuto3ShotPose", new ShootFrom(ShooterPose.ShortSideAuto3ShotPose));
        NamedCommands.registerCommand("BlueShortSideAutoShotPose", new ShootFrom(ShooterPose.BlueShortSideAutoShotPose));
        NamedCommands.registerCommand("BlueShortSideAuto2ShotPose", new ShootFrom(ShooterPose.BlueShortSideAuto2ShotPose));   
        NamedCommands.registerCommand("BlueShortSideAuto3ShotPose", new ShootFrom(ShooterPose.BlueShortSideAuto3ShotPose));
        NamedCommands.registerCommand("BlueShortSideAuto4ShotPose", new ShootFrom(ShooterPose.BlueShortSideAuto4ShotPose)); 
        NamedCommands.registerCommand("PodiumAutoShotPose", new ShootFrom(ShooterPose.PodiumAutoShotPose));        
        NamedCommands.registerCommand("AutoZero", new AutoZero(swerve).withTimeout(0.1)); 
        NamedCommands.registerCommand("AutoHomeState", new ShootFrom(ShooterPose.AutoHome));
        NamedCommands.registerCommand("AutoRunFeeder", new AutoRunFeeder());
        NamedCommands.registerCommand("LSSZeroGyro", new AutoSetGyro(swerve, 120).withTimeout(0.1));
        NamedCommands.registerCommand("AutoVisionAlignShoot", new AutoVisionAlignShoot(swerve, true).withTimeout(0.7));
    }
}
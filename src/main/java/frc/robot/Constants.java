package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class FieldAngle {
        public static final double Left = 270;
        public static final double Right = 90;
        public static final double Front = 180;
        public static final double Rear = 360;
        public static final double SourceRed = 205;  // 
        public static final double SourceBlue = -205;
        public static final double PodiumRed = 25; // 
        public static final double PodiumBlue = -25;
        private static Alliance AllianceColor = null;

        public static final Alliance getAllianceColor(){
            if(AllianceColor == null && DriverStation.getAlliance().isPresent()){
                AllianceColor = DriverStation.getAlliance().get();
            }
            return AllianceColor;
        }

        public static final double getSourceAngle() {
            Alliance allianceColor = getAllianceColor();
            return allianceColor!=null ? 
                allianceColor==Alliance.Red ?
                    Constants.FieldAngle.SourceRed
                    : Constants.FieldAngle.SourceBlue
                :0.0;
        }

        public static final double getPodiumAngle() {
            Alliance allianceColor = getAllianceColor();
            return allianceColor!=null  ? 
                allianceColor==Alliance.Red ?
                    Constants.FieldAngle.PodiumRed
                    : Constants.FieldAngle.PodiumBlue
                :0.0;
        }


        public static final double getRightAngle() {
            Alliance allianceColor = getAllianceColor();
            return allianceColor!=null  ? 
                allianceColor==Alliance.Red ?
                    Constants.FieldAngle.Left
                    : Constants.FieldAngle.Left
                :0.0;
        }

        public static final double getLeftAngle() {
            Alliance allianceColor = getAllianceColor();
            return allianceColor!=null ? 
                allianceColor==Alliance.Red ?
                    Constants.FieldAngle.Right
                    : Constants.FieldAngle.Right
                :0.0;
        }
    }

    /// Shooter Position and Velocity Settings
    public static final class Shooter {
        public static final double DownPosition = 0;

        public static final class DefaultShotVelocity {
            public static final double VelocityLeft = 90;
            public static final double VelocityRight = 60; 
        }

        public static final class TrapShotVelocity {
            public static final double VelocityLeft = 20;  // 45
            public static final double VelocityRight = 20;  //35
        }

        public static final class SlowShotVelocity {
            public static final double VelocityLeft = 60;
            public static final double VelocityRight = 40; 
        }

        public static final class StopShotVelocity {
            public static final double VelocityLeft = 0;
            public static final double VelocityRight = 0; 
        }

        public static abstract class ShooterPose {
            protected double VelocityLeft;
            protected double VelocityRight;
            protected double Position;
            public double getVelocityLeft() { return VelocityLeft; }
            public double getVelocityRight() { return VelocityRight; }
            public double getPosition() { return Position; }

            public static final ShooterPose Subwoofer  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 20.4;
                }
            };

            public static final ShooterPose Podium = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 76.5;  // 76.86
                }
            };

            public static final ShooterPose Amp = new ShooterPose() {
                {
                    VelocityLeft = SlowShotVelocity.VelocityLeft;
                    VelocityRight = SlowShotVelocity.VelocityRight;
                    Position = 77;  // 74.1  // 77  
                }
            };

            public static final ShooterPose TrapShot = new ShooterPose() {
                {
                    VelocityLeft = TrapShotVelocity.VelocityLeft;
                    VelocityRight = TrapShotVelocity.VelocityRight;
                    Position = 61.1;

                }
            };

            public static final ShooterPose SubwooferTall = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 72.6;
                }
            };

            public static final ShooterPose Climb  = new ShooterPose() {
                {
                    VelocityLeft = StopShotVelocity.VelocityLeft;
                    VelocityRight = StopShotVelocity.VelocityRight;
                    Position = 25;
                }
            };

            public static final ShooterPose ClimbReady  = new ShooterPose() {
                {
                    VelocityLeft = StopShotVelocity.VelocityLeft;
                    VelocityRight = StopShotVelocity.VelocityRight;
                    Position = 99.5;
                }
            };

            public static final ShooterPose Home = new ShooterPose() {
                {
                    VelocityLeft = SlowShotVelocity.VelocityLeft;
                    VelocityRight = SlowShotVelocity.VelocityRight;
                    Position = 0.0;
                }
            };

            public static final ShooterPose AutoHome = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 0.0;
                }
            };

            public static final ShooterPose PodiumAutoShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 8.93;
                }
            };

            public static final ShooterPose LongAutoShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 2.3;
                }
            };

            public static final ShooterPose ShortSideAutoShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 4.1;  // was 3.8
                }
            };

            public static final ShooterPose ShortSideAuto2ShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 3.3;
                }
            };

            public static final ShooterPose ShortSideAuto3ShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 3.7;
                }
            };

            public static final ShooterPose BlueShortSideAutoShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 3.3;
                }
            };

            public static final ShooterPose BlueShortSideAuto2ShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 3.1;
                }
            };

            public static final ShooterPose BlueShortSideAuto3ShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 2.4;
                }
            };

            public static final ShooterPose BlueShortSideAuto4ShotPose  = new ShooterPose() {
                {
                    VelocityLeft = DefaultShotVelocity.VelocityLeft;
                    VelocityRight = DefaultShotVelocity.VelocityRight;
                    Position = 2.1;
                }
            };

            
            public static final ShooterPose PassShotPose  = new ShooterPose() {
                {
                    VelocityLeft = TrapShotVelocity.VelocityLeft;
                    VelocityRight = TrapShotVelocity.VelocityRight;
                    Position = 11.8;
                }
            };
        };

    }

    public static final class Feeder {
        public static final double IntakeSpeed = 0.7;
        public static final double FeedShooterSpeed = 1.0;
        public static final double RearEjectSpeed = -1.0;
    }

    public static final class Swerve {
        public static final int pigeonID = 12;

        public static final COTSTalonFXSwerveConstants chosenModule = 
        //COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);    // eggo
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(5.357);                                    // eggo
        //COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L3);    // dial

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.75);
        public static final double wheelBase = Units.inchesToMeters(20.75); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 30;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.01;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.01;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive kraken to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 5.2;
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-115.137-180);  //105.73236 eggo -115.137-180 dial
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 0;
            public static final int canCoderID = 8;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(74.443);  // 1.40616 + 180 eggo 74.443 dial
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(35.244+180);  //-125.68356 eggo 35.244+180 dial
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(170.895); //-73.91582 - 180 eggo 170.895 dial
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        //added for auto
        public static final double maxModuleSpeed = 5.2; // M/S
        public static final Translation2d flModuleOffset = new Translation2d(0.4, 0.4);
        public static final Translation2d frModuleOffset = new Translation2d(0.4, -0.4);
        public static final Translation2d blModuleOffset = new Translation2d(-0.4, 0.4);
        public static final Translation2d brModuleOffset = new Translation2d(-0.4, -0.4);

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(2.9, 0, 0), // Translation constants 
            new PIDConstants(5.0, 0, 0), // Rotation constants 
            maxModuleSpeed, 
            flModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig()
            );
    }

    /*public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 5.2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 8;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    */
}
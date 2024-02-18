package frc.robot.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutonomousSelector {

    private static SendableChooser<AutonomousMode> autonomousModeChooser;

    static {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto settings");

        autonomousModeChooser = new SendableChooser<>();
        autonomousModeChooser.addOption("5 Note Auto", AutonomousMode.Five_Note_Auto);
        autonomousModeChooser.addOption("Long Side Auto", AutonomousMode.Long_Side_Auto);
        autonomousModeChooser.addOption("Short Side Auto", AutonomousMode.Short_Side_Auto);
        autonomousModeChooser.addOption("Center Auto", AutonomousMode.Center_Auto);
        
        autoTab.add("Mode", autonomousModeChooser);
        
    }

    public Command getCommand(Swerve s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case Five_Note_Auto:
                return new PathPlannerAuto("5 Note Auto");
            case Long_Side_Auto:
                return new PathPlannerAuto("Long Side Auto");
            case Short_Side_Auto:
                return new PathPlannerAuto("Short Side Auto");
            case Center_Auto:
                return new PathPlannerAuto("Center Auto");
            default:
                return new PathPlannerAuto("5 Note Auto");
        }
    }

    public AutonomousSelector() {
        
    }

    private enum AutonomousMode {
        Five_Note_Auto,
        Long_Side_Auto,
        Short_Side_Auto,
        Center_Auto
    }

}

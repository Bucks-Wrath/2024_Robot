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
        autonomousModeChooser.addOption("4 Note Slow", AutonomousMode.Four_Note_Slow);
        autoTab.add("Mode", autonomousModeChooser);
        
    }

    public Command getCommand(Swerve s_Swerve){
        AutonomousMode mode = autonomousModeChooser.getSelected();

        switch (mode) {
            case Four_Note_Slow:
                return new PathPlannerAuto("4 Note Slow");
            default:
                return new PathPlannerAuto("4 Note Slow");
        }
    }

    public AutonomousSelector() {
        
    }

    private enum AutonomousMode {
        Four_Note_Slow,

    }

}

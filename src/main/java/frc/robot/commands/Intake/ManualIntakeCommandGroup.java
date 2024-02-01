package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.IntakeRunFeeder;

public class ManualIntakeCommandGroup extends SequentialCommandGroup {   
    
    public ManualIntakeCommandGroup() {
        addCommands(new IntakeRunFeeder().raceWith(new RunIntake()));  // use for testing
    }

}

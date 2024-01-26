package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.StopFeeder;

public class StopIntakeCommandGroup extends SequentialCommandGroup {
    
    public StopIntakeCommandGroup() {
        addCommands(new StopIntake().alongWith(new StopFeeder()));
    }

}

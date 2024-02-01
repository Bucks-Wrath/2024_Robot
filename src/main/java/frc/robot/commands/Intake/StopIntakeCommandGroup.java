package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.StopFeeder;

public class StopIntakeCommandGroup extends SequentialCommandGroup {
    
    public StopIntakeCommandGroup() {
        addCommands(new ReverseIntake().withTimeout(1.0).andThen(new StopIntake().alongWith(new StopFeeder())));
    }

}

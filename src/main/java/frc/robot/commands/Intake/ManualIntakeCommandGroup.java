package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.IntakeRunFeeder;
import frc.robot.subsystems.Swerve;

public class ManualIntakeCommandGroup extends SequentialCommandGroup {

    private Swerve s_Swerve;    
    
    public ManualIntakeCommandGroup(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addCommands(new IntakeRunFeeder().raceWith(new RunIntake()));  // use for testing
    }

}

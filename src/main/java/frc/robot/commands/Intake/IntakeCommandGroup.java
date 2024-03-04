package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class IntakeCommandGroup extends SequentialCommandGroup {

    private Swerve s_Swerve;    
    
    public IntakeCommandGroup(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addCommands(new ManualIntakeCommandGroup().deadlineWith(new VisionAlignIntake(s_Swerve, true)));  
    }

}

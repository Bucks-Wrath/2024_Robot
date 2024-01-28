package frc.robot.commands.Shooter.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Shooter.SetShooterPosition;
import frc.robot.commands.Shooter.SetShooterVelocity;
import frc.robot.Constants.Shooter.ShooterPose;
public class ShootFrom extends SequentialCommandGroup {

    public ShootFrom(ShooterPose shooterPose) {
        addCommands(new SetShooterVelocity(shooterPose.getVelocityLeft(), shooterPose.getVelocityRight())
            .alongWith(new SetShooterPosition(shooterPose.getPosition())));
    }

}

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

public class AutoSetGyro extends Command {

    private Swerve s_swerve;
    private double gValue;

    public AutoSetGyro(Swerve s_swerve, double angle) {
        this.s_swerve = s_swerve;
        this.gValue = angle;
        addRequirements(s_swerve);
    }

    public void initialize() {
        s_swerve.autoSetGyro(gValue);
    }

    public void execute() {
        
    }

    public boolean isFinished() {
        return false;
    }

    protected void end() {
        
    }

    protected void interrupted() {
        end();
    }
}
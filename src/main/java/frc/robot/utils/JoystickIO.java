package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class JoystickIO {
    private static final XboxController pilot = new XboxController(0);
    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

    public JoystickIO () {
        swerveSubsystem.setDefaultCommand(new DriveCommand(
            swerveSubsystem,
            () -> pilot.getLeftX(),
            () -> pilot.getLeftY(),
            () -> pilot.getRightX()
        ));
    }




}

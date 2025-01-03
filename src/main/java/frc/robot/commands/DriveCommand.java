package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class DriveCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotSupplier;

    public DriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedSupplier = xSpeed;
        this.ySpeedSupplier = ySpeed;
        this.rotSupplier = rot;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double xSpeed = applyDeadband(xSpeedSupplier.getAsDouble());
        double ySpeed = applyDeadband(ySpeedSupplier.getAsDouble());
        double rot = -applyDeadband(rotSupplier.getAsDouble());
        
        xSpeed *= SwerveConstants.Module.maxVelocity;
        ySpeed *= SwerveConstants.Module.maxVelocity;
        rot *= SwerveConstants.Module.maxAngularVelocity;

        swerveSubsystem.drive(xSpeed, ySpeed, rot);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.drive(0,0,0); // stop all movement
    }

    // prevent unintentional robot movement due to small joystick movements
    private double applyDeadband(double value) {
        return Math.abs(value) > Constants.ControllerConstants.deadBand ? value : 0.0;
    }

}

package frc.robot.subsystems;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModule;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveModule[] modules = new SwerveModule[4];

    public SwerveSubsystem() {
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(i);
        }
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        SwerveModuleState[] moduleStates = SwerveConstants.Drive.driveKinematics.toSwerveModuleStates(speeds);

        // normalize speeds to avoid exceeding motor limits
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            SwerveConstants.Module.maxVelocity
        );

        for (int i = 0; i < modules.length; i++) {
            modules[i].setDesiredState(moduleStates[i]);
        }
    }

    @Override
    public void periodic() {
        
    }
    
}

package frc.robot.subsystems;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.utils.ShuffleData;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveModule[] modules = new SwerveModule[4];

    //logging
    private ShuffleData<String> currentCommandLog = new ShuffleData<String>(this.getName(), "current command", "None");
    private ShuffleData<Double[]> realStatesLog = new ShuffleData<Double[]>(
      this.getName(),
      "real states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    private ShuffleData<Double[]> desiredStatesLog = new ShuffleData<Double[]>(
      this.getName(),
      "desired states",
      new Double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
    private ShuffleData<Double> velocityLog = new ShuffleData<Double>(
      this.getName(),
      "velocity",
      0.0);
    private ShuffleData<Double> rotationalVelocityLog = new ShuffleData<Double>(
      this.getName(),
      "rotational velocity",
      0.0);


    public SwerveSubsystem() {
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new SwerveModule(i);
        }
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("rot", rot);

        SwerveModuleState[] moduleStates = SwerveConstants.Drive.driveKinematics.toSwerveModuleStates(speeds);

        // normalize speeds to avoid exceeding motor limits
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            SwerveConstants.Module.maxVelocity
        );

    
            modules[0].setDesiredState(moduleStates[0]);
            modules[1].setDesiredState(moduleStates[1]);
            modules[2].setDesiredState(moduleStates[2]);
            modules[3].setDesiredState(moduleStates[3]);

        
    
    }


    private void logData() {
        Double[] realStates = {
            modules[0].getState().angle.getDegrees(),
            modules[0].getState().speedMetersPerSecond,
            modules[1].getState().angle.getDegrees(),
            modules[1].getState().speedMetersPerSecond,
            modules[2].getState().angle.getDegrees(),
            modules[2].getState().speedMetersPerSecond,
            modules[3].getState().angle.getDegrees(),
            modules[3].getState().speedMetersPerSecond
        };
    
        Double[] desiredStates = {
            modules[0].getDesiredState().angle.getDegrees(),
            modules[0].getDesiredState().speedMetersPerSecond,
            modules[1].getDesiredState().angle.getDegrees(),
            modules[1].getDesiredState().speedMetersPerSecond,
            modules[2].getDesiredState().angle.getDegrees(),
            modules[2].getDesiredState().speedMetersPerSecond,
            modules[3].getDesiredState().angle.getDegrees(),
            modules[3].getDesiredState().speedMetersPerSecond
        };
    
        realStatesLog.set(realStates);
        desiredStatesLog.set(desiredStates);
    }

    @Override
    public void periodic() {
        // periodic method for individual modules
    for (int i = 0; i < 4; i++) {
        modules[i].periodic();
      }
  
      logData();
    }
    
}

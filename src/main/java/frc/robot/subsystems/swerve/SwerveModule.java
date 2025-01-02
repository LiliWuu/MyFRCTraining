package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANcoder encoder;
    private final PIDController turnPID;
    
    public SwerveModule (int driveMotorPort, int turnMotorPort, int encoderID) {
        driveMotor = new CANSparkMax(driveMotorPort, CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorPort, CANSparkMax.MotorType.kBrushless);
        encoder = new CANcoder(encoderID);

        turnPID = new PIDController(SwerveConstants.Module.kP_TURN, SwerveConstants.Module.kI_TURN, SwerveConstants.Module.kD_TURN);
        turnPID.enableContinuousInput(0, 360);
    }

    public void setDesiredState (SwerveModuleState state) {
        double speed = state.speedMetersPerSecond;
        double angle = state.angle.getRadians();

        driveMotor.set(speed);
        turnMotor.set(turnPID.calculate(getAngle(), angle));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), Rotation2d.fromRadians(getAngle()));
    }

    public double getAngle() {
        double angleRotations = encoder.getAbsolutePosition().getValue();
        double angleDegrees = angleRotations * 360;
        return Units.degreesToRadians(angleDegrees);
    }

    
}

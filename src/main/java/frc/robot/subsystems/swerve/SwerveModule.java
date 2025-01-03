package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ShuffleData;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANcoder encoder;
    private final PIDController turnPID;

    private final ShuffleData<Double> driveSpeed;
    private final ShuffleData<Double> drivePosition;
    private final ShuffleData<Double> turnAngle;
    private final ShuffleData<Double> turnPIDError;

    private SwerveModuleState desiredState = new SwerveModuleState();
    private String name;

    public SwerveModule (int module) {
        driveMotor = new CANSparkMax(SwerveConstants.Module.driveMotorPorts[module], CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(SwerveConstants.Module.turnMotorPorts[module], CANSparkMax.MotorType.kBrushless);
        encoder = new CANcoder(SwerveConstants.Module.encoderPorts[module]);

        turnPID = new PIDController(SwerveConstants.Module.kP_TURN, SwerveConstants.Module.kI_TURN, SwerveConstants.Module.kD_TURN);
        turnPID.enableContinuousInput(0, 2 * Math.PI);

        driveMotor.getEncoder().setVelocityConversionFactor((1 / SwerveConstants.Module.driveMotorGearRatio) * Units.rotationsPerMinuteToRadiansPerSecond(1) * (SwerveConstants.Module.wheelDiameterMeters / 2));

        if (module == 0) {
            name = "FL module";
        } else if (module == 1) {
            name = "FR module";
        } else if (module == 2) {
            name = "BL module";
        } else if (module == 3) {
            name = "BR module";
        }

        // Initialize Shuffleboard data fields
        driveSpeed = new ShuffleData<>("Swerve/Module" + name, "Drive Speed", 0.0);
        drivePosition = new ShuffleData<>("Swerve/Module" + name, "Drive Position", 0.0);
        turnAngle = new ShuffleData<>("Swerve/Module" + name, "Turn Angle", 0.0);
        turnPIDError = new ShuffleData<>("Swerve/Module" + name, "Turn PID Error", 0.0);
    }

    public void setDesiredState (SwerveModuleState state) {
        // optimize state to ensure the shortest rotation path is taken
        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getAngle()));
        double speed = state.speedMetersPerSecond;
        double angle = state.angle.getRadians();

        // reduce unnecessary jittery movement when the speed nears zero
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }

        this.desiredState = state;


        driveMotor.set(speed);
        turnMotor.set(turnPID.calculate(getAngle(), angle));

       

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), Rotation2d.fromRadians(getAngle()));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    // calculate the current angle in radians
    public double getAngle() {
        double angleRotations = encoder.getAbsolutePosition().getValue();
        double angleDegrees = angleRotations * 360;
        return Units.degreesToRadians(angleDegrees);
    }

    private double getRotVelocity() {
        return Units.rotationsToRadians(encoder.getVelocity().getValueAsDouble());
    };

    // called within the swerve subsystem's periodic
    public void periodic() {
        driveSpeed.set(driveMotor.getEncoder().getVelocity());
        drivePosition.set(driveMotor.getEncoder().getPosition());
        turnAngle.set(getAngle());
        turnPIDError.set(turnPID.getPositionError());
    }

    
}

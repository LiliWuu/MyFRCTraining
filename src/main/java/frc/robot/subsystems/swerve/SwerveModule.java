package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;
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
    public double turnAbsolutePositionRad = 0.0;
    private double offset;
    private String name;
    private double moduleNumber;

    public SwerveModule(int module) {
        driveMotor = new CANSparkMax(SwerveConstants.Module.driveMotorPorts[module], CANSparkMax.MotorType.kBrushless);
        turnMotor = new CANSparkMax(SwerveConstants.Module.turnMotorPorts[module], CANSparkMax.MotorType.kBrushless);
        encoder = new CANcoder(SwerveConstants.Module.encoderPorts[module]);

        turnPID = new PIDController(SwerveConstants.Module.kP_TURN, SwerveConstants.Module.kI_TURN,
                SwerveConstants.Module.kD_TURN);
        turnPID.enableContinuousInput(0, 2 * Math.PI);

        driveMotor.getEncoder().setVelocityConversionFactor((1 / SwerveConstants.Module.driveMotorGearRatio)
                * Units.rotationsPerMinuteToRadiansPerSecond(1) * (SwerveConstants.Module.wheelDiameterMeters / 2));

        moduleNumber = module;
        if (module == 0) {
            name = "FL module";
        } else if (module == 1) {

            name = "FR module";
        } else if (module == 2) {
            name = "BL module";
        } else if (module == 3) {

            name = "BR module";
        }

        offset = Units.degreesToRadians(
                SwerveConstants.Module.absoluteEncoderOffsetDeg[module]);

        // Initialize Shuffleboard data fields
        driveSpeed = new ShuffleData<>("Swerve/Module" + name, "Drive Speed", 0.0);
        drivePosition = new ShuffleData<>("Swerve/Module" + name, "Drive Position", 0.0);
        turnAngle = new ShuffleData<>("Swerve/Module" + name, "Turn Angle", 0.0);
        turnPIDError = new ShuffleData<>("Swerve/Module" + name, "Turn PID Error", 0.0);
    }

    public void setDesiredState(SwerveModuleState state) {
        // optimize state to ensure the shortest rotation path is taken
        double speed = state.speedMetersPerSecond;
        double angle = state.angle.getRadians();
        while (angle < 0) {
            angle += 2 * Math.PI;
        }
        while (angle > 2 * Math.PI) {
            angle -= 2 * Math.PI;
        }

        // reduce unnecessary jittery movement when the speed nears zero
        if (Math.abs(speed) < 0.01) {
            speed = 0;
        }

        this.desiredState = state;

        // Get to later
        // driveMotor.set(speed);
        turnMotor.setVoltage(turnPID.calculate(getAngle(), angle));
        // turnMotor.setVoltage(1);

    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), new Rotation2d(getAngle()));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    private double getAngle() {
        double pos = Units.rotationsToRadians(encoder.getPosition().getValueAsDouble())
                - offset;
        while (pos < 0) {
            pos += 2 * Math.PI;
        }
        while (pos > 2 * Math.PI) {
            pos -= 2 * Math.PI;
        }
        if (moduleNumber == 2){
            return pos;
        }
        return -pos;
    };

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

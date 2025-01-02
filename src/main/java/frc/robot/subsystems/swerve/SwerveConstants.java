package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {

    public static final class Drive {
        public static final double trackWidth = 6.0;
        public static final double wheelBase = 6.0;

        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2), // FL
            new Translation2d(wheelBase / 2, -trackWidth / 2), // FR
            new Translation2d(-wheelBase / 2, trackWidth / 2), // BL
            new Translation2d(-wheelBase / 2, -trackWidth / 2) // BR
        );
    
    }

    public static final class Module {

        public static final double kP_TURN = 3.75; 
        public static final double kI_TURN = 0.0;
        public static final double kD_TURN = 0.0;

        public static final double driveMotorGearRatio = 6.75;
        public static final double wheelDiameterMeters = Units.inchesToMeters(4);

        // Module Settings: order is FL, FR, BL, BR
        public static final int[] driveMotorPorts = {3, 5, 7, 9};
        public static final int[] turnMotorPorts = {4, 6, 8, 10};
        public static final int[] encoderPorts = {11, 12, 13, 14};

        public static final double maxVelocity = 4.3;
    }
    
}

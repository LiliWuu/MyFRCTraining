package frc.robot.subsystems.swerve;

public class SwerveConstants {

    public static final class Drive {
        public static final double trackWidth = 6.0;
        public static final double wheelBase = 6.0;
    }

    public static final class Module {
        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 0;
        public static final int FRONT_LEFT_TURN_MOTOR_PORT = 1;
        public static final int FRONT_LEFT_ENCODER_PORT = 2;

        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 3;
        public static final int FRONT_RIGHT_TURN_MOTOR_PORT = 4;
        public static final int FRONT_RIGHT_ENCODER_PORT = 5;

        public static final double kP_TURN = 0.5; 
        public static final double kI_TURN = 0.0;
        public static final double kD_TURN = 0.05;
    }
    
}

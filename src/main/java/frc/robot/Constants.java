package frc.robot;

public class Constants {

    public static class ShooterConstants{
        public static final int shooterMotorRightID = 14; 
        public static final int shooterMotorLeftID = 15; 

        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 0.11;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static class IntakeConstants{
        public static final int intakeMotorID = 16;
        public static final double intakeMotorSpeed = 0.75;
        public static final double intakeMotorSpeed1 = 1;
    }

    public static class SpindexConstants{
        public static final int spindexMotorID = 21;
        public static final int kickerMotorID = 22;
        public static final double spindexMotorSpeed = 1;
        public static final double kickerMotorSpeed = 1;
        public static final double spindexTimer = 2;
        public static final double spindexCurrentLimit = 30;
        public static final double spindexReverseSpeed = -1;
    }

    public static class TurretConstants{
        public static final int turretMotorID = 17;
        public static final int turretEncoderID = 1;
        public static final double turretMinimumRotation = -90;
        public static final double turretMaximumRotation = 90;
        public static final double kS = 0.25;
        public static final double kV = 0.12;
        public static final double kA = 0.01;
        public static final double kP = 0.11;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static class ClimberConstants{
        public static final int climberMotorID = 23;
        public static final double climberMotorSpeed = 0.5;
    }
    
}

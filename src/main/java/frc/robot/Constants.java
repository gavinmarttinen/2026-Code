package frc.robot;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static class ShooterConstants{
        public static final int shooterMotorRightID = 14; 
        public static final int shooterMotorLeftID = 15; 
        public static final double shooterMotorVelocity = 58;
        public static final double shooterMotorVelocityMax = 100;
        public static final double shooterMotorVelocityHub = 45;
        public static final double kS = 0;
        public static final double kV = 0.12;
        public static final double kA = 0;
        public static final double kP = 1.2;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static class IntakeConstants{
        public static final int intakeMotorID = 16;
        public static final double intakeMotorSpeed = 1;
    }

    public static class SpindexConstants{
        public static final int spindexMotorID = 21;
        public static final int kickerMotorID = 22;
        public static final double spindexMotorSpeed = 1;
        public static final double spindexMotorSpeedSlow = 0.5;
        public static final double kickerMotorSpeed = 1;
        public static final double kickerMotorReverseSpeed = -0.05;
        public static final double spindexTimer = 2;
        public static final double spindexCurrentLimit = 30;
        public static final double spindexReverseSpeed = -1;
    }

    public static class TurretConstants{
        public static final int turretMotorID = 17;
        public static final int turretEncoderID = 1;
        public static final double turretMinimumRotation = 0;
        public static final double turretMaximumRotation = 155; //125;
        public static final double kS = 0.22;
        public static final double kV = 0.126;
        public static final double kA = 0.0;
        public static final double kP = 2.5;
        public static final double kI = 0;
        public static final double kD = 0.1;
        public static final double gearRatio = 0.1875;
        public static final double degreesPerRev = 360;
    }

    public static class ClimberConstants{
        public static final int climberMotorID = 23;
        public static final double climberMotorSpeed = 1;
    }

    public static class FieldConstants{
        public static final Translation2d redHub = new Translation2d(Units.inchesToMeters(469.11), Units.inchesToMeters(158.84));
        public static final Translation2d blueHub = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
        public static final Translation2d blueLeftZoneGoal = new Translation2d();
        public static final Translation2d blueRightZoneGoal = new Translation2d();
        public static final Translation2d redLeftZoneGoal = new Translation2d();
        public static final Translation2d redRightZoneGoal = new Translation2d();
        public static final Rectangle2d blueLeftZone = new Rectangle2d(new Translation2d(4.365,8),new Translation2d(16.5,4));
        public static final Rectangle2d blueRightZone = new Rectangle2d(new Translation2d(16.5,4),new Translation2d(4.365,0));
        public static final Rectangle2d redLeftZone = new Rectangle2d(new Translation2d(12.1,0),new Translation2d(0,4));
        public static final Rectangle2d redRightZone = new Rectangle2d(new Translation2d(0,4),new Translation2d(12.1,8));
        public static final Rectangle2d blueAllianceZone = new Rectangle2d(new Translation2d(0,0), new Translation2d(4.365,8));
        public static final Rectangle2d redAllianceZone = new Rectangle2d(new Translation2d(12.1,0), new Translation2d(16.5,8));
        public static final Translation2d blueClimbLeft = new Translation2d();
        public static final Translation2d blueClimbRight = new Translation2d();
        public static final Translation2d redClimbLeft = new Translation2d();
        public static final Translation2d redClimbRight = new Translation2d();
    
    }
}

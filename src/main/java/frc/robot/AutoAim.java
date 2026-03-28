package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAim {
    private static InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap shooterSpeedTable = new InterpolatingDoubleTreeMap();
    private CommandSwerveDrivetrain drivetrain;
    private Translation2d allianceHub;

public AutoAim(CommandSwerveDrivetrain drivetrain){
 //value is the time of flight for the fuel
 //key is the distance from hub

 hoodAngleTable.put(1.3, 0.0);
 hoodAngleTable.put(2.04, 0.43);
 hoodAngleTable.put(2.95, 0.47);
 hoodAngleTable.put(4.28, 0.52);
 hoodAngleTable.put(5.66, 0.52);

 shooterSpeedTable.put(1.3, 44.9);
 shooterSpeedTable.put(2.04, 46.79);
 shooterSpeedTable.put(2.95, 49.15);
 shooterSpeedTable.put(4.28, 56.61);
 shooterSpeedTable.put(5.66, 62.05);

 timeOfFlightTable.put(3.0, 1.44);
 timeOfFlightTable.put(3.5, 1.4);
 timeOfFlightTable.put(4.0, 1.32);
 timeOfFlightTable.put(5.25, 1.13);
 this.drivetrain = drivetrain;
}

public double calculateHoodAngle(){
    double hoodAngle = hoodAngleTable.get(getHubDistance());
    return hoodAngle;
}

public double calculateShooterSpeed(){
    double shooterSpeed = shooterSpeedTable.get(getHubDistance());
    return shooterSpeed;
}

public Translation2d goalPositionWithTOF(){
    ChassisSpeeds speeds= ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond, drivetrain.getState().Speeds.omegaRadiansPerSecond, drivetrain.getState().Pose.getRotation());
     allianceHub = drivetrain.getGoalPose();
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d turretPose = robotPose.transformBy(new Transform2d(Units.inchesToMeters(-6.25),Units.inchesToMeters(-5.25), new Rotation2d()));
    double distance = turretPose.getTranslation().getDistance(allianceHub);
    double TOF = timeOfFlightTable.get(distance);
    Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond,
    speeds.vyMetersPerSecond);
    Translation2d goalPoseWithTOF = allianceHub.minus(robotVelocity.times(TOF));
    double goalDistance = turretPose.getTranslation().getDistance(goalPoseWithTOF);
    double TOF1 = timeOfFlightTable.get(goalDistance);
    Translation2d goalPoseWithTOF1 = allianceHub.minus(robotVelocity.times(TOF1));
    double goalDistance1 = turretPose.getTranslation().getDistance(goalPoseWithTOF1);
    double TOF2 = timeOfFlightTable.get(goalDistance1);
    Translation2d goalPoseWithTOF2 = allianceHub.minus(robotVelocity.times(TOF2));
    return goalPoseWithTOF2;
}
public double getHubDistance(){
    Pose2d robotPose = drivetrain.getState().Pose;
   // Pose2d turretPose = robotPose.transformBy(new Transform2d(-0.1,-0.2, new Rotation2d())).rotateAround(robotPose.getTranslation(), robotPose.getRotation());
    return robotPose.getTranslation().getDistance(goalPositionWithTOF());
}

public double getHubRotation(){
    Pose2d robotPose = drivetrain.getState().Pose;
    Translation2d hub = goalPositionWithTOF();
    double setpoint = hub.minus(robotPose.getTranslation()).getAngle().getDegrees();
    // if(setpoint<0){
    //     setpoint += 360;
    // }
        return setpoint; 
}
}
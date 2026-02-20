package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAim {
    private static InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();
    private CommandSwerveDrivetrain drivetrain;
    private Translation2d allianceHub;

public AutoAim(CommandSwerveDrivetrain drivetrain){
 hoodAngleTable.put(2.21, 0.0);
 hoodAngleTable.put(3.55, 0.25);
 hoodAngleTable.put(5.4, 0.65);

 //value is the time of flight for the fuel
 //key is the distance from hub

 timeOfFlightTable.put(2.21, 1.43);
 timeOfFlightTable.put(3.55, 1.36);
 timeOfFlightTable.put(5.4, 1.11);
 this.drivetrain = drivetrain;
 allianceHub = drivetrain.isBlue() ? FieldConstants.blueHub:FieldConstants.redHub;
}

public double calculateHoodAngle(){
    double hoodAngle = hoodAngleTable.get(getHubDistance());
    return hoodAngle;
}

public Translation2d goalPositionWithTOF(){
    
    Pose2d robotPose = drivetrain.getState().Pose;
    //Pose2d turretPose = robotPose.transformBy(new Transform2d(-0.1,-0.2, new Rotation2d())).rotateAround(robotPose.getTranslation(), robotPose.getRotation());
    double distance = robotPose.getTranslation().getDistance(allianceHub);
    double TOF = timeOfFlightTable.get(distance);
    Translation2d robotVelocity = new Translation2d(drivetrain.getState().Speeds.vxMetersPerSecond,
    drivetrain.getState().Speeds.vyMetersPerSecond);
    Translation2d goalPoseWithTOF = allianceHub.minus(robotVelocity.times(TOF));
    double goalDistance = robotPose.getTranslation().getDistance(goalPoseWithTOF);
    double TOF1 = timeOfFlightTable.get(goalDistance);
    Translation2d goalPoseWithTOF1 = allianceHub.minus(robotVelocity.times(TOF1));
    double goalDistance1 = robotPose.getTranslation().getDistance(goalPoseWithTOF1);
    double TOF2 = timeOfFlightTable.get(goalDistance1);
    Translation2d goalPoseWithTOF2 = allianceHub.plus(robotVelocity.times(TOF2));
    return goalPoseWithTOF2;
}

public double getHubDistance(){
    Pose2d robotPose = drivetrain.getState().Pose;
   // Pose2d turretPose = robotPose.transformBy(new Transform2d(-0.1,-0.2, new Rotation2d())).rotateAround(robotPose.getTranslation(), robotPose.getRotation());
    return robotPose.getTranslation().getDistance(goalPositionWithTOF());
}
}
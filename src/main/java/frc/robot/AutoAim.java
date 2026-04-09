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
    private static InterpolatingDoubleTreeMap scoringHoodAngleTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap scoringTimeOfFlightTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap scoringShooterSpeedTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap passingHoodAngleTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap passingTimeOfFlightTable = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap passingShooterSpeedTable = new InterpolatingDoubleTreeMap();
    private CommandSwerveDrivetrain drivetrain;
    private Translation2d allianceHub;

public AutoAim(CommandSwerveDrivetrain drivetrain){
 //value is the time of flight for the fuel
 //key is the distance from hub
 double scoringDistance1 = 1.62;
 double scoringDistance2 = 2.06;
 double scoringDistance3 = 2.99;
 double scoringDistance4 = 3.49;
 double scoringDistance5 = 4.27;
 double scoringDistance6 = 4.45;
 double scoringDistance7 = 5.38;

 double passingDistance1 = 4.29;
 double passingDistance2 = 6.55;
 double passingDistance3 = 9.21;
 double passingDistance4 = 11.86;
 double passingDistance5 = 14.64;

 //scoring
 scoringHoodAngleTable.put(scoringDistance1, 0.41);
 scoringHoodAngleTable.put(scoringDistance2, 0.45);
 scoringHoodAngleTable.put(scoringDistance3, 0.46);
 scoringHoodAngleTable.put(scoringDistance4, 0.48);
 scoringHoodAngleTable.put(scoringDistance5, 0.49);
 scoringHoodAngleTable.put(scoringDistance6, 0.51);
 scoringHoodAngleTable.put(scoringDistance7, 0.53);

 scoringShooterSpeedTable.put(scoringDistance1, 43.53);
 scoringShooterSpeedTable.put(scoringDistance2, 44.88);
 scoringShooterSpeedTable.put(scoringDistance3, 47.09);
 scoringShooterSpeedTable.put(scoringDistance4, 49.5);
 scoringShooterSpeedTable.put(scoringDistance5, 52.59);
 scoringShooterSpeedTable.put(scoringDistance6, 57.02);
 scoringShooterSpeedTable.put(scoringDistance7, 59.53);

 scoringTimeOfFlightTable.put(scoringDistance1, 0.87);
 scoringTimeOfFlightTable.put(scoringDistance2, 1.07);
 scoringTimeOfFlightTable.put(scoringDistance3, 1.11);
 scoringTimeOfFlightTable.put(scoringDistance4, 1.08);
 scoringTimeOfFlightTable.put(scoringDistance5, 1.17);
 scoringTimeOfFlightTable.put(scoringDistance6, 1.24);
 scoringTimeOfFlightTable.put(scoringDistance7, 1.27);


 //passing
 passingHoodAngleTable.put(passingDistance1,0.45);
 passingHoodAngleTable.put(passingDistance2,0.47);
 passingHoodAngleTable.put(passingDistance3,0.52);
 passingHoodAngleTable.put(passingDistance4,0.55);
 passingHoodAngleTable.put(passingDistance5,0.59);

 passingShooterSpeedTable.put(passingDistance1,54.87);
 passingShooterSpeedTable.put(passingDistance2,63.6);
 passingShooterSpeedTable.put(passingDistance3,75.08);
 passingShooterSpeedTable.put(passingDistance4,86.56);
 passingShooterSpeedTable.put(passingDistance5,100.0);

 passingTimeOfFlightTable.put(passingDistance1,1.56);
 passingTimeOfFlightTable.put(passingDistance2,1.73);
 passingTimeOfFlightTable.put(passingDistance3,1.87);
 passingTimeOfFlightTable.put(passingDistance4,1.94);
 passingTimeOfFlightTable.put(passingDistance5,1.76);
 this.drivetrain = drivetrain;
}

public double calculateHoodAngle(){
    if(drivetrain.isPassing()){
        return passingHoodAngleTable.get(getHubDistance());
    }
        else{
            return scoringHoodAngleTable.get(getHubDistance());
        }
   // return scoringHoodAngleTable.get(getHubDistance());

}

public double calculateShooterSpeed(){
    if(drivetrain.isPassing()){
        return passingShooterSpeedTable.get(getHubDistance());
    }
    else{
        return scoringShooterSpeedTable.get(getHubDistance());
    }
  //  return scoringShooterSpeedTable.get(getHubDistance()+0.5);
}

public Translation2d goalPositionWithTOF(){
    ChassisSpeeds speeds= ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds.vxMetersPerSecond, drivetrain.getState().Speeds.vyMetersPerSecond, drivetrain.getState().Speeds.omegaRadiansPerSecond, drivetrain.getState().Pose.getRotation());
     allianceHub = drivetrain.getGoalPose();
    Pose2d robotPose = drivetrain.getVisionPose();
    Pose2d turretPose = robotPose.transformBy(new Transform2d(Units.inchesToMeters(-6.25),Units.inchesToMeters(5.25), new Rotation2d()));
    double distance = turretPose.getTranslation().getDistance(allianceHub);
    double TOF = drivetrain.isPassing()?passingTimeOfFlightTable.get(distance):scoringTimeOfFlightTable.get(distance);
    Translation2d robotVelocity = new Translation2d(speeds.vxMetersPerSecond,
    speeds.vyMetersPerSecond);
    Translation2d goalPoseWithTOF = allianceHub.minus(robotVelocity.times(TOF));
    double goalDistance = turretPose.getTranslation().getDistance(goalPoseWithTOF);
    double TOF1 = drivetrain.isPassing()?passingTimeOfFlightTable.get(goalDistance):scoringTimeOfFlightTable.get(goalDistance);
    Translation2d goalPoseWithTOF1 = allianceHub.minus(robotVelocity.times(TOF1));
    double goalDistance1 = turretPose.getTranslation().getDistance(goalPoseWithTOF1);
    double TOF2 = drivetrain.isPassing()?passingTimeOfFlightTable.get(goalDistance1):scoringTimeOfFlightTable.get(goalDistance1);
    Translation2d goalPoseWithTOF2 = allianceHub.minus(robotVelocity.times(TOF2));
    return goalPoseWithTOF2;
}
public double getHubDistance(){
    Pose2d robotPose = drivetrain.getVisionPose();
    Pose2d turretPose = robotPose.transformBy(new Transform2d(Units.inchesToMeters(-6.25),Units.inchesToMeters(5.25), new Rotation2d()));
    return turretPose.getTranslation().getDistance(goalPositionWithTOF());
}

public double getHubRotation(){
    Pose2d robotPose = drivetrain.getVisionPose();
    Translation2d hub = goalPositionWithTOF();
    double setpoint = hub.minus(robotPose.getTranslation()).getAngle().getDegrees();
    // if(setpoint<0){
    //     setpoint += 360;
    // }
        return setpoint; 
}
}
package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlockerConstants;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BlockerSubsystem extends SubsystemBase{
private final SparkMax leftLinearActuator = new SparkMax(BlockerConstants.blockerLeftSparkMaxID, MotorType.kBrushed);
private final SparkMax rightLinearActuator = new SparkMax(BlockerConstants.blockerRightSparkMaxID, MotorType.kBrushed);

public void extendBlocker(){
    rightLinearActuator.set(BlockerConstants.ExtendSpeed);
    leftLinearActuator.set(BlockerConstants.ExtendSpeed);
}

public void retractBlocker(){
    rightLinearActuator.set(BlockerConstants.RetractSpeed);
    leftLinearActuator.set(BlockerConstants.RetractSpeed);
}

public void stopBlocker(){
    rightLinearActuator.set(BlockerConstants.StopBlocker);
    leftLinearActuator.set(BlockerConstants.StopBlocker);
}
}

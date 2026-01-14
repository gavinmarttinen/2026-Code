package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
private final TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterMotorID);
private final TalonFX shooterMotor1 = new TalonFX(ShooterConstants.shooterMotor1ID);
private final double shooterMotorSpeed = 0;

private final VoltageOut m_voltReq = new VoltageOut(0.0);

private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         null,        // Use default ramp rate (1 V/s)
         Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> shooterMotor.setControl(m_voltReq.withOutput(volts.in(Volts))),
         null,
         this
      )
   );

public ShooterSubsystem() {   

shooterMotor1.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

// in init function
var talonFXConfigs = new TalonFXConfiguration();

// set slot 0 gains
var slot0Configs = talonFXConfigs.Slot0;
slot0Configs.kS = ShooterConstants.kS; // Add 0.25 V output to overcome static friction
slot0Configs.kV = ShooterConstants.kV; // A velocity target of 1 rps results in 0.12 V output
slot0Configs.kA = ShooterConstants.kA; // An acceleration of 1 rps/s requires 0.01 V output
slot0Configs.kP = ShooterConstants.kP; // An error of 1 rps results in 0.11 V output
slot0Configs.kI = ShooterConstants.kI; // no output for integrated error
slot0Configs.kD = ShooterConstants.kD; // no output for error derivative

// set Motion Magic Velocity settings
var motionMagicConfigs = talonFXConfigs.MotionMagic;
motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

shooterMotor.getConfigurator().apply(talonFXConfigs);
}

@Override
public void periodic() {

    SmartDashboard.putNumber("shooterMotorSpeed",shooterMotorSpeed);
    
    shooterMotor.set(shooterMotorSpeed);
}

public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.dynamic(direction);
}

}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// 16 25 13/16
package frc.robot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SpindexConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.SpindexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private final CommandPS5Controller driverController = new CommandPS5Controller(0);
    private final CommandPS5Controller operatorController = new CommandPS5Controller(1);

    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
    private final SpindexSubsystem spindexSubsystem = new SpindexSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();


    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final AutoAim autoAim = new AutoAim(drivetrain);
    private final HoodSubsystem hoodSubsystem = new HoodSubsystem(autoAim);

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
      //  turretSubsystem.setDefaultCommand(Commands.run(()->turretSubsystem.setTurretSpeed(operatorController.getLeftX()), turretSubsystem));
        //turretSubsystem.setDefaultCommand(Commands.run(()->turretSubsystem.setTurretPosition(50),turretSubsystem));
        //turretSubsystem.setDefaultCommand(Commands.run(()->turretSubsystem.stopTurretMotor(), turretSubsystem));
        turretSubsystem.setDefaultCommand(Commands.run(()->turretSubsystem.setTurretPosition(autoAim.getHubRotation().getDegrees() - drivetrain.getState().Pose.getRotation().getDegrees()), turretSubsystem));
        climberSubsystem.setDefaultCommand(Commands.run(()->climberSubsystem.setClimberSpeed(0), climberSubsystem));
        spindexSubsystem.setDefaultCommand(Commands.run(()->spindexSubsystem.reverseKickerStopSpindex(),spindexSubsystem));
        intakeSubsystem.setDefaultCommand(Commands.run(()->intakeSubsystem.setIntakeSpeed(0), intakeSubsystem));
        hoodSubsystem.setDefaultCommand(Commands.run(()->hoodSubsystem.setHoodPosition(), hoodSubsystem));



       // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
       // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //    point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        //));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
       // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
       // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
       // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
       // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        driverController.L1().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        operatorController.povUp().whileTrue(Commands.run(()->climberSubsystem.setClimberSpeed(ClimberConstants.climberMotorSpeed), climberSubsystem));
        operatorController.povDown().whileTrue(Commands.run(()->climberSubsystem.setClimberSpeed(-ClimberConstants.climberMotorSpeed), climberSubsystem));
        operatorController.square().whileTrue(Commands.run(()->spindexSubsystem.setSpindexSpeed(SpindexConstants.spindexMotorSpeed, SpindexConstants.kickerMotorSpeed),spindexSubsystem));
      //  operatorController.L2().whileTrue(Commands.run(()->intakeSubsystem.setIntakeSpeed(-IntakeConstants.intakeMotorSpeed), intakeSubsystem));
       // operatorController.R2().whileTrue(Commands.run(()->intakeSubsystem.setIntakeSpeed(-IntakeConstants.intakeMotorSpeed1), intakeSubsystem));
        //operatorController.L1().whileTrue(Commands.run(()->intakeSubsystem.setIntakeSpeed(IntakeConstants.intakeMotorSpeed), intakeSubsystem));
        operatorController.R1().whileTrue(Commands.run(()->intakeSubsystem.setIntakeSpeedWithCurrent(), intakeSubsystem));
        operatorController.triangle().whileTrue(Commands.run(()-> shooterSubsystem.setShooterVelocity()));
        operatorController.cross().whileTrue(Commands.run(()->turretSubsystem.setTurretPosition(200),turretSubsystem));
        //operatorController.circle().whileTrue(Commands.run(()-> shooterSubsystem.setShooterVelocity(100)));
        drivetrain.registerTelemetry(logger::telemeterize);
    }
    
    public Command getAutonomousCommand() {
          return autoChooser.getSelected();
    }
}

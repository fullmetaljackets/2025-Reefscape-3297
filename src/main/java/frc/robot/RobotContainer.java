// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.IntakeToggle;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.intake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.WristRot;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Intake s_Intake = new Intake();
    private final IntakeJaws s_IntakeJaws = new IntakeJaws();
    private final ArmRot s_ArmRot = new ArmRot();
    private final ArmExtend s_ArmExtend = new ArmExtend();
    private final WristRot s_WristRot = new WristRot();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        
        joystick.povUp().whileTrue(drivetrain.applyRequest(() -> 
        drive.withVelocityX(0.25 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
        .withVelocityY(0 * MaxSpeed ) // Drive left with negative X (left)
        .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
        joystick.povDown().whileTrue(drivetrain.applyRequest(() -> 
        drive.withVelocityX(-0.25 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
        .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
        joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> 
        drive.withVelocityX(0 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
        .withVelocityY(0.25 * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
        joystick.povRight().whileTrue(drivetrain.applyRequest(() -> 
        drive.withVelocityX(0 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
        .withVelocityY(-0.25 * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Intake controlls 
        joystick.rightTrigger().whileTrue(new intake(1, s_Intake));
        joystick.leftTrigger().whileTrue(new intake(-1, s_Intake));

        //IntakeJaws toggle
        joystick.y().toggleOnTrue(new IntakeToggle(s_IntakeJaws));

        //ArmRot/ArmExtend setpoint controlls
        joystick.a().onTrue(new ArmRotToSetpoint(0, s_ArmRot));
        joystick.b().onTrue(new ArmExtendToSetpoint(0, s_ArmExtend));

        //WristRot setpoint controlls
        joystick.x().onTrue(new WristRotToSetpoint(0, s_WristRot));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

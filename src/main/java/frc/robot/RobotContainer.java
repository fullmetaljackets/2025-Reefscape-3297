// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.grouped.BackFloorIntake;
import frc.robot.commands.grouped.BallIntake;
import frc.robot.commands.grouped.Barge;
import frc.robot.commands.grouped.LV3Algee;
import frc.robot.commands.grouped.ReefLV4;
import frc.robot.commands.grouped.Middle;
import frc.robot.commands.grouped.ReefLV3;
import frc.robot.commands.grouped.ReefLV2;
import frc.robot.commands.grouped.ReefLV1;
import frc.robot.commands.ArmExtendRun;
import frc.robot.commands.ArmExtendToSetpoint;
import frc.robot.commands.ArmRotRun;
import frc.robot.commands.ArmRotToSetpoint;
import frc.robot.commands.AutoAlignReef;
import frc.robot.commands.AutoAlignToAprilTag;
import frc.robot.commands.IntakeToggle;
import frc.robot.commands.WristRotRun;
import frc.robot.commands.WristRotToSetpoint;
import frc.robot.commands.intake;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.WristRot;



public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxspeedTest = TunerConstants.kMaxSpeed;
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 1/4 of a rotation per second max angular velocity

    private final CommandXboxController DriveStick = new CommandXboxController(0);
    private final CommandXboxController CopilotStick = new CommandXboxController(1);

    //  Setting up bindings for necessary control of the swerve drive platform 
     private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
           .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
             .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
             .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    // SlewRateLimiter speedLimiter = new SlewRateLimiter(2); // 2 m/s/s


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Limelight s_Limelight = new Limelight();
    private final Intake s_Intake = new Intake();
    private final IntakeJaws s_IntakeJaws = new IntakeJaws();
    private final ArmRot s_ArmRot = new ArmRot();
    private final ArmExtend s_ArmExtend = new ArmExtend();
    private final WristRot s_WristRot = new WristRot();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        SmartDashboard.putData(s_ArmExtend);
        SmartDashboard.putData(s_ArmRot);
        SmartDashboard.putData(s_Intake);
        SmartDashboard.putData(s_WristRot);


        configureBindings();

        NamedCommands.registerCommand("AutoL4", new ReefLV4(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        NamedCommands.registerCommand("place coral", new intake(.18, s_Intake) );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {

    // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-DriveStick.getLeftY() * MaxspeedTest) // Drive forward with negative Y (forward)
                    .withVelocityY(-DriveStick.getLeftX() * MaxspeedTest) // Drive left with negative X (left)
                    .withRotationalRate(-DriveStick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // DriveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // DriveStick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-DriveStick.getLeftY(), -DriveStick.getLeftX()))
        // ));
        
        // DriveStick.povUp().whileTrue(drivetrain.applyRequest(() -> 
        // drive.withVelocityX(0.25 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
        // .withVelocityY(0 * MaxSpeed ) // Drive left with negative X (left)
        // .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        // ));
        // DriveStick.povDown().whileTrue(drivetrain.applyRequest(() -> 
        // drive.withVelocityX(-0.25 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
        // .withVelocityY(0 * MaxSpeed) // Drive left with negative X (left)
        // .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        // ));
        // DriveStick.povLeft().whileTrue(drivetrain.applyRequest(() -> 
        // drive.withVelocityX(0 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
        // .withVelocityY(0.25 * MaxSpeed) // Drive left with negative X (left)
        // .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        // ));
        // DriveStick.povRight().whileTrue(drivetrain.applyRequest(() -> 
        // drive.withVelocityX(0 * MaxSpeed) // Drive forward 25% of MaxSpeed (forward)
        // .withVelocityY(-0.25 * MaxSpeed) // Drive left with negative X (left)
        // .withRotationalRate(0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DriveStick.back().and(DriveStick.y()).onTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriveStick.back().and(DriveStick.x()).onTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriveStick.start().and(DriveStick.y()).onTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriveStick.start().and(DriveStick.x()).onTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        // DriveStick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        //Limelight AutoAlignReef
        // DriveStick.rightBumper().onTrue(new AutoAlignReef(s_Limelight, drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        // DriveStick.start().onTrue(new AutoAlignReef(s_Limelight, drivetrain));
        // DriveStick.start().onTrue(drivetrain.applyRequest(() -> drive.withVelocityX(AutoAlignReef.LLRange)
        // .withVelocityY(AutoAlignReef.LLStrafe)
        // .withRotationalRate(0)).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        DriveStick.rightBumper().whileTrue(new AutoAlignToAprilTag(drivetrain, s_Limelight));

        //DriveStick controls
        DriveStick.rightTrigger().whileTrue(new intake(0.18, s_Intake));
        DriveStick.leftTrigger().whileTrue(new intake(-0.2, s_Intake));
        DriveStick.rightBumper().onFalse(new IntakeToggle(s_IntakeJaws));
        
        DriveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DriveStick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // DriveStick.x().onTrue(new LV3Algee(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));

        //manualy adjust arm extend
        DriveStick.x().whileTrue(new ArmExtendRun(-0.1, s_ArmExtend));
        DriveStick.y().whileTrue(new ArmExtendRun(0.1, s_ArmExtend));

        //manualy adjust arm rot
        DriveStick.povUp().whileTrue(new ArmRotRun(0.4, s_ArmRot));
        DriveStick.povDown().whileTrue(new ArmRotRun(-0.4, s_ArmRot));

        //manualy adjust wrist rot
        DriveStick.povLeft().whileTrue(new WristRotRun(-0.1, s_WristRot));
        DriveStick.povRight().whileTrue(new WristRotRun(0.1, s_WristRot));


        //CopilotStick controls
        CopilotStick.a().onTrue(new ReefLV1(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws).alongWith(new PrintCommand("Hello World 1")));
        CopilotStick.b().onTrue(new ReefLV2(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws).alongWith(new PrintCommand("Hello World 2")));
        CopilotStick.x().onTrue(new ReefLV3(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws).alongWith(new PrintCommand("Hello World 3")));
        CopilotStick.y().onTrue(new ReefLV4(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws).alongWith(new PrintCommand("Hello World 4")));
        //CopilotStick.y().onTrue(new PrintCommand("Y Pressed"));
        
        CopilotStick.rightTrigger().onTrue(new BackFloorIntake(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        CopilotStick.leftTrigger().onTrue(new BallIntake(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));

        CopilotStick.rightBumper().onTrue(new Middle(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));

        CopilotStick.povUp().onTrue(new Barge(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
    






        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        } else {
            return value;
        }
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");

        return autoChooser.getSelected();

    }

}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ArmExtendRun;
import frc.robot.commands.ArmRotRun;
import frc.robot.commands.AutoAlignToAprilTagLeftLV3;
import frc.robot.commands.AutoAlignToAprilTagLeftLV4;
import frc.robot.commands.AutoAlignToAprilTagRightLV3;
import frc.robot.commands.AutoAlignToAprilTagRightLV4;
import frc.robot.commands.AutoDriveToCoral;
import frc.robot.commands.AutoStrafeToCoral;
import frc.robot.commands.ClimberRun;
import frc.robot.commands.IntakeClose;
import frc.robot.commands.IntakeOpen;
import frc.robot.commands.IntakeRun;
import frc.robot.commands.IntakeToggle;
import frc.robot.commands.SetPipelineLeft;
import frc.robot.commands.SetPipelineRight;
import frc.robot.commands.WristRotRun;
import frc.robot.commands.grouped.AutoAlignToCoral;
import frc.robot.commands.grouped.AutoBackFloorIntake;
import frc.robot.commands.grouped.AutoReefLV4Over;
import frc.robot.commands.grouped.BackBallIntake;
import frc.robot.commands.grouped.BackFloorIntake;
import frc.robot.commands.grouped.BallIntake;
import frc.robot.commands.grouped.Barge;
import frc.robot.commands.grouped.FeedIntake;
import frc.robot.commands.grouped.Middle;
import frc.robot.commands.grouped.ReefLV1;
import frc.robot.commands.grouped.ReefLV3Over;
import frc.robot.commands.grouped.ReefLV4Over;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmRot;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeJaws;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.WristRot;




public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 1/4 of a rotation per second max angular velocity

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
    private final LimelightHelpers LimelightHelpers = new LimelightHelpers();
    private final Limelight s_Limelight = new Limelight();
    private final Intake s_Intake = new Intake();
    private final IntakeJaws s_IntakeJaws = new IntakeJaws();
    private final ArmRot s_ArmRot = new ArmRot();
    private final ArmExtend s_ArmExtend = new ArmExtend();
    private final WristRot s_WristRot = new WristRot();
    private final Climber s_Climber = new Climber();
    private final LEDSubsystem s_LedSubsystem = new LEDSubsystem();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        SmartDashboard.putData(s_ArmExtend);
        SmartDashboard.putData(s_ArmRot);
        SmartDashboard.putData(s_Intake);
        SmartDashboard.putData(s_WristRot);


        configureBindings();

        NamedCommands.registerCommand("L4", new ReefLV4Over(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        NamedCommands.registerCommand("AutoL4", new AutoReefLV4Over(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));

        // NamedCommands.registerCommand("AutoL2", new ReefLV2(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));

        NamedCommands.registerCommand("place coral", new IntakeRun(.18, s_Intake).withTimeout(0.5));
        NamedCommands.registerCommand("floor intake", new AutoBackFloorIntake(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        NamedCommands.registerCommand("Close Intake", new IntakeClose(s_IntakeJaws));
        NamedCommands.registerCommand("open intake", new IntakeOpen(s_IntakeJaws));

        //limelight auto alignment 
        NamedCommands.registerCommand("align left", new AutoAlignToAprilTagLeftLV4(drivetrain, s_Limelight));
        NamedCommands.registerCommand("align right", new AutoAlignToAprilTagRightLV4(drivetrain, s_Limelight));

        NamedCommands.registerCommand("pipeline right", new SetPipelineRight(LimelightHelpers).withTimeout(.2));
        NamedCommands.registerCommand("pipeline left", new SetPipelineLeft(LimelightHelpers).withTimeout(.2));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {

    // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-DriveStick.getLeftY() * 2) // Drive forward with negative Y (forward)
                    .withVelocityY(-DriveStick.getLeftX() * 2) // Drive left with negative X (left)
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
        DriveStick.leftBumper().and(DriveStick.povLeft()).whileTrue(new AutoAlignToAprilTagLeftLV4(drivetrain, s_Limelight).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        DriveStick.leftBumper().and(DriveStick.povRight()).whileTrue(new AutoAlignToAprilTagRightLV4(drivetrain, s_Limelight).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        
        DriveStick.leftBumper().and(DriveStick.povDown()).whileTrue(new AutoAlignToAprilTagLeftLV3(drivetrain, s_Limelight).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        DriveStick.leftBumper().and(DriveStick.povUp()).whileTrue(new AutoAlignToAprilTagRightLV3(drivetrain, s_Limelight).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

        DriveStick.leftBumper().and(DriveStick.y()).whileTrue(new AutoAlignToCoral(s_Limelight, drivetrain));

        

        DriveStick.start().onTrue(new FeedIntake(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        //DriveStick controls
        // DriveStick.x().whileTrue(new intake(0.18, s_Intake));
        // DriveStick.y().whileTrue(new intake(-0.2, s_Intake));
        DriveStick.rightTrigger().whileTrue(new IntakeRun(0.18, s_Intake));
        DriveStick.rightTrigger().whileFalse(new IntakeRun(-0.03, s_Intake));

        DriveStick.leftTrigger().whileTrue(new IntakeRun(-0.2, s_Intake));
        DriveStick.leftTrigger().whileFalse(new IntakeRun(-0.03, s_Intake));

        DriveStick.rightBumper().onFalse(new IntakeToggle(s_IntakeJaws));
        
        DriveStick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DriveStick.b().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // DriveStick.x().onTrue(new LV3Algee(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));

        //manualy adjust arm extend
        DriveStick.y().and(DriveStick.povUp()).whileTrue(new ArmExtendRun(0.2, s_ArmExtend));
        DriveStick.y().and(DriveStick.povDown()).whileTrue(new ArmExtendRun(-0.2, s_ArmExtend));
        // DriveStick.x().whileTrue(new ArmExtendRun(-0.2, s_ArmExtend));
        // DriveStick.y().whileTrue(new ArmExtendRun(0.2, s_ArmExtend));

        //Climber
        DriveStick.y().and(DriveStick.povRight()).whileTrue(new ClimberRun(-1, s_Climber));
        DriveStick.y().and(DriveStick.povLeft()).whileTrue(new ClimberRun(1, s_Climber));

        // //manualy adjust arm rot
        DriveStick.x().and(DriveStick.povUp()).whileTrue(new ArmRotRun(0.4, s_ArmRot));
        DriveStick.x().and(DriveStick.povDown()).whileTrue(new ArmRotRun(-0.4, s_ArmRot));
        // DriveStick.povUp().whileTrue(new ArmRotRun(0.4, s_ArmRot));
        // DriveStick.povDown().whileTrue(new ArmRotRun(-0.4, s_ArmRot));

        // //manualy adjust wrist rot
        DriveStick.x().and(DriveStick.povRight()).whileTrue(new WristRotRun(0.4, s_WristRot));
        DriveStick.x().and(DriveStick.povLeft()).whileTrue(new WristRotRun(-0.4, s_WristRot));
        // DriveStick.povLeft().whileTrue(new WristRotRun(-0.1, s_WristRot));
        // DriveStick.povRight().whileTrue(new WristRotRun(0.1, s_WristRot).withInterruptBehavior(InterruptionBehavior.kCancelSelf));


        //CopilotStick controls
        CopilotStick.b().onTrue(new ReefLV1(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws).alongWith(new PrintCommand("Hello World 1")));
        // CopilotStick.a().onTrue(new ReefLV2(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws).alongWith(new PrintCommand("Hello World 2")));
        CopilotStick.x().onTrue(new ReefLV3Over(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws).alongWith(new PrintCommand("Hello World 3")));
        CopilotStick.y().onTrue(new ReefLV4Over(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws).alongWith(new PrintCommand("Hello World 4")));
        //CopilotStick.y().onTrue(new PrintCommand("Y Pressed"));
        
        CopilotStick.rightTrigger().onTrue(new BackFloorIntake(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        CopilotStick.leftTrigger().onTrue(new Middle(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));

        CopilotStick.rightBumper().and(CopilotStick.y()).onTrue(new AutoReefLV4Over(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        // CopilotStick.leftBumper().whileTrue(new intake(-0.5, s_Intake));

        CopilotStick.povDown().onTrue(new BackBallIntake(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        CopilotStick.povUp().onTrue(new Barge(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        // CopilotStick.start().onTrue(new Barge(s_ArmRot, s_ArmExtend, s_WristRot, s_IntakeJaws));
        CopilotStick.leftBumper().whileTrue(new BallIntake(s_Intake, s_IntakeJaws));

        // CopilotStick.povLeft().whileTrue(new ClimberRun(1, s_Climber));
        // CopilotStick.povRight().whileTrue(new ClimberRun(-1, s_Climber));


        // Joystick controls for arm motor
        new WristRotRun(CopilotStick.getLeftY(), s_WristRot);


    






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

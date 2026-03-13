// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controllers.CommandGameSirT3Lite;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.OverBumper;
import frc.robot.subsystems.Flywheel.FlywheelSetpoint;
import frc.robot.subsystems.Intake.IntakeSetpoint;
import frc.robot.vision.LimelightHelpers;
import frc.robot.vision.LimelightVisionSystem;
import frc.robot.vision.LoggableRobotPose;
//import frc.robot.vision.Commands.*;
//import frc.robot.vision.AutoAlign;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.SwerveDriveBrake speedChange = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.FieldCentricFacingAngle targetHub = new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(10, 0, 0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandGameSirT3Lite joystick = new CommandGameSirT3Lite(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Flywheel flywheel = new Flywheel();
    public final Intake intake = new Intake();
    public final Climb climb = new Climb();
    public final OverBumper overBumper = new OverBumper();
    public final LimelightVisionSystem vision = new LimelightVisionSystem(this::consumePhotonVisionMeasurement, () -> drivetrain.getState().Pose);
    //public final 

    private final AngularVelocity SpinUpThreshold = RotationsPerSecond.of(3); // Tune to increase accuracy while not sacrificing throughput
    /* The flywheel is ready to shoot when it's near the target or when the driver overrides it with the X button */
    private final Trigger isFlywheelReadyToShoot = flywheel.getTriggerWhenNearTarget(SpinUpThreshold).or(joystick.x());

    Trigger downButton = new Trigger(
        () ->  joystick.getHID().getPOV() == 180 && joystick.getHID().getPOV() != 270 && joystick.getHID().getPOV() != 90
    );
    Trigger upButton = new Trigger(
        () ->  joystick.getHID().getPOV() == 0 && joystick.getHID().getPOV() != 270 && joystick.getHID().getPOV() != 90
    );
    Trigger leftButton = new Trigger(
        () ->  joystick.getHID().getPOV() == 270 && joystick.getHID().getPOV() != 0 && joystick.getHID().getPOV() != 180
    );
    Trigger rightButton = new Trigger(
        () ->  joystick.getHID().getPOV() == 90 && joystick.getHID().getPOV() != 0 && joystick.getHID().getPOV() != 180
    );
    Trigger abxy = new Trigger(
        () -> joystick2.getHID().getAButton() || joystick2.getHID().getBButton() || joystick2.getHID().getXButton() || joystick2.getHID().getYButton()
    );

    Command alignToHubCommand = drivetrain.applyRequest(()-> {
            if (!vision.isHubTargetValid()) {
                /* Do typical field-centric driving since we don't have a target */
                return drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            } else {
                /* Use the hub target to determine where to aim */
                  return targetHub.withTargetDirection(vision.getHeadingToHubFieldRelative())
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed); // Drive left with negative X (left) 
            }
        });

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("Stop Shooting", flywheel.coastFlywheel().alongWith(intake.coastIntake()));
        /* Shoot commands need a bit of time to spool up the flywheel before feeding with the intake */
        NamedCommands.registerCommand("VisShoot", flywheel.setDistance(() -> vision.getHubDistance()));
        NamedCommands.registerCommand("Shoot Near", flywheel.setTarget(() -> FlywheelSetpoint.AutoNear));
        NamedCommands.registerCommand("Shoot Mid", flywheel.setTarget(() -> FlywheelSetpoint.AutoMid));
        NamedCommands.registerCommand("Shoot Far", flywheel.setTarget(() -> FlywheelSetpoint.Far));
        NamedCommands.registerCommand("Stop Intake", intake.coastIntake().alongWith(flywheel.coastFlywheel()));
        NamedCommands.registerCommand("Intake Fuel", intake.setTarget(() -> IntakeSetpoint.Intake).alongWith(flywheel.setTarget(() -> FlywheelSetpoint.Intake)));
        NamedCommands.registerCommand("Outtake Fuel", intake.setTarget(() -> IntakeSetpoint.Outtake));
        // NamedCommands.registerCommand("Go to Pos", camera.setGoal(() -> ShootPoints.Middle).andThen(AutoAlign(() -> )));
        NamedCommands.registerCommand("Climb", climb.climb());
        NamedCommands.registerCommand("UnClimb", climb.unclimb());
        NamedCommands.registerCommand("Align", alignToHubCommand);
        NamedCommands.registerCommand("Shoot", Commands.waitUntil(isFlywheelReadyToShoot).andThen(intake.setTarget(() ->IntakeSetpoint.FeedToShoot)));


        autoChooser = AutoBuilder.buildAutoChooser("Only Score");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {
                Translation2d requestedVelocity = new Translation2d(
                    -joystick.getLeftY() * MaxSpeed, // Drive forward with negative Y (forward)
                    -joystick.getLeftX() * MaxSpeed // Drive left with negative X (left)
                );

                requestedVelocity = SwerveUtils.driverDivot(requestedVelocity, drivetrain.getState().Pose.getRotation());

                return drive.withVelocityX(requestedVelocity.getX())
                    .withVelocityY(requestedVelocity.getY())
                     .withRotationalRate(-joystick.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left) 
            }
            )
        );

        climb.setDefaultCommand(climb.run(()-> {
            double climbY = joystick2.getLeftY();
            if (climbY > 0.1 || climbY < -0.1) {
                climb.driveOpenLoop(climbY);
            } else{
                climb.driveOpenLoop(0);
            }
        }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.rightBumper().whileTrue(alignToHubCommand.alongWith(
            flywheel.coastFlywheel()
            // flywheel.setDistance(() -> {
            //     return SwerveUtils.findDistanceToTarget(drivetrain.getState().Pose);
            // })
        ));


        //over bumper intake controls
        // joystick.x().onTrue(overBumper.run(() -> overBumper.overBumperToPos(90)));
        // joystick.b().onTrue(overBumper.run(() -> overBumper.overBumperToPos(20)));
        // overBumper.setDefaultCommand(overBumper.run(()-> {
        //     double overBumperY = joystick2.getRightY();
        //     if (overBumperY > 0.1 || overBumperY < -0.1) {
        //         overBumper.driveOpenLoop(overBumperY);
        //     } else{
        //         overBumper.driveOpenLoop(0);
        //     }
        // }));

        final double StraightSpeed = 1;

        // joystick.povRight().whileTrue(drivetrain.applyRequest(() -> 
        //     forwardStraight.withVelocityX(0).withVelocityY(0.5))
        // );
        rightButton.whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(0).withVelocityY(-StraightSpeed))
        );
        // joystick.povLeft().whileTrue(drivetrain.applyRequest(() -> 
        //     forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        // );
        leftButton.whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(0).withVelocityY(StraightSpeed))
        );
        // joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        upButton.whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(StraightSpeed).withVelocityY(0))
        );
        // joystick.povDown().whileTrue(drivetrain.applyRequest(() -> 
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))  
        // );
        downButton.whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(-StraightSpeed).withVelocityY(0))  
        );

         joystick.povDownLeft().whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(-StraightSpeed).withVelocityY(StraightSpeed))
        );
         joystick.povDownRight().whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(-StraightSpeed).withVelocityY(-StraightSpeed))
        );
         joystick.povUpLeft().whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(StraightSpeed).withVelocityY(StraightSpeed))
        );
         joystick.povUpRight().whileTrue(drivetrain.applyRequest(() -> 
            forwardStraight.withVelocityX(StraightSpeed).withVelocityY(-StraightSpeed))
        );

        // Bind the start button to set the field-centric forward in case it's lost for whatever reason.
        joystick.screenshot().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Bind left bumper/trigger to our intake/outtake
        joystick.leftTrigger().whileTrue(intake.setTarget(()->IntakeSetpoint.Intake).alongWith(flywheel.setTarget(()->FlywheelSetpoint.Intake)));
        joystick.capture().whileTrue(intake.setTarget(()->IntakeSetpoint.Outtake));

        // Bind right bumper/trigger to our near/far shots
       abxy.whileTrue(
        flywheel.setTarget(() -> {
            if (joystick2.getHID().getAButton()) {
                return FlywheelSetpoint.Far;
            }
            if (joystick2.getHID().getBButton()) {
                return FlywheelSetpoint.Mid;
            }
            if (joystick2.getHID().getYButton()) {
                return FlywheelSetpoint.Near;
            }
            if (joystick2.getHID().getXButton()) {
                return FlywheelSetpoint.Pass;
            }
            return FlywheelSetpoint.Nothing;
        })
       );
    //    abxy.negate().and(joystick.rightTrigger()).and(joystick.povDown()).whileTrue(
    //     flywheel.setDistanceBack(vision::getHubDistance)
    //    );
       abxy.negate().and(joystick.rightTrigger())/*.and(joystick.povDown().negate())*/.whileTrue(
        flywheel.setDistance(vision::getHubDistance)
       );

       
        
       joystick.rightTrigger().whileTrue(
        Commands.waitUntil(isFlywheelReadyToShoot).andThen(intake.setTarget(()->IntakeSetpoint.FeedToShoot))
       );
        // Bind right bumper/trigger to prep flywheeel(operator)
         joystick2.rightBumper().toggleOnTrue(
            flywheel.setTarget(()->FlywheelSetpoint.Near) //spin up the flywheel
           // .alongWith(Commands.waitUntil(isFlywheelReadyToShoot).andThen(intake.setTarget(()->IntakeSetpoint.FeedToShoot)))
        );
        joystick2.rightTrigger().toggleOnTrue(
            flywheel.setTarget(()->FlywheelSetpoint.Far) // spin up the flywheel
          //  .alongWith(Commands.waitUntil(isFlywheelReadyToShoot).andThen(intake.setTarget(()->IntakeSetpoint.FeedToShoot)))
        );
        
        // make x + y button change speed
        joystick.y().whileTrue(
            Commands.runOnce(()->{
                if (MaxSpeed > 2.5) {
                   MaxSpeed = 2.5; 
                } else{
                    MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
                }
                
            })
        );
        joystick.leftBumper().onTrue(
            Commands.runOnce(()->{
                   MaxSpeed = 2.5; 
                } 
        ));
        joystick.leftBumper().onFalse(
            Commands.runOnce(()->{
                   MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); 
                } 
        ));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    private Matrix<N3, N1> defaultDevs = new Matrix<N3, N1>(N3.instance, N1.instance, new double[]{10, 10, Double.POSITIVE_INFINITY});
    public void consumePhotonVisionMeasurement(LoggableRobotPose pose) {
        /* Super simple, should modify to support variable standard deviations */
        // System.out.println("Pose: " + pose.estimatedPose.getX() + " - " + pose.estimatedPose.getY() + " - " + pose.estimatedPose.getZ());
        drivetrain.setVisionMeasurementStdDevs(defaultDevs);
        drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }


    public void periodic() {
        vision.periodic();

        // SmartDashboard.putNumber("Max Speed", MaxSpeed);
    }

    public void simulationPeriodic() {
        var drivetrainPose = drivetrain.m_simOdometry.getPoseMeters();
        vision.simPeriodic(drivetrainPose);

        // var debugField = vision.getSimDebugField();
        // debugField.getObject("EstimatedRobot").setPose(drivetrainPose);
    }
}

package frc.robot.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SwerveUtils;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Fiducial;

public class LimelightVisionSystem {
    final AprilTagFieldLayout TagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    /* IDs 3,4 and 19,20 are on the side of the hub that we can't shoot from, so don't include them */
    private final int[] RedHubApriltagIds = new int[]{
        2, /* 3, 4, */ 5, 8, 9, 10, 11
    };
    private final int[] BlueHubApriltagIds = new int[]{
        18, /* 19, 20, */ 21, 24, 25, 26, 27
    };

    private final ApriltagTarget RedHub = new ApriltagTarget(Map.of(
        2, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        5, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        8, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
        9, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
        10, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        11, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))
    ));

    private final Translation2d BlueHubTarget = new Translation2d(4.6256194, 4.0346376);
    private final Translation2d RedHubTarget = new Translation2d(11.915419, 4.0346376);
    // private final Translation2d RedLeftCorner = new Translation2d(11.915419, 4.0346376); //make real values
    // private final Translation2d RedRightCorner = new Translation2d(11.915419, 4.0346376);
    // private final Translation2d BlueRightCorner = new Translation2d(11.915419, 4.0346376);
    // private final Translation2d BlueLeftCorner = new Translation2d(11.915419, 4.0346376);

    private final ApriltagTarget BlueHub = new ApriltagTarget(Map.of(
        18, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        21, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        24, new Translation3d(Inches.of(-23.5), Inches.of(-14), Inches.of(33)),
        25, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33)),
        26, new Translation3d(Inches.of(-23.5), Inches.of(0), Inches.of(33)),
        27, new Translation3d(Inches.of(-23.5), Inches.of(14), Inches.of(33))
    ));

    Transform3d robotToCamera = new Transform3d(
        /* X, Y, Z */
        new Translation3d(Meters.of(-0.5), Meters.of(0), Meters.of(0.5)),
        /* Roll, Pitch, Yaw */
        /* A pitch of -40 degrees appears to see the apriltags  */
        new Rotation3d(Degrees.of(0), Degrees.of(-40), Degrees.of(180)));
    
    /* Use the current robot heading to keep track of where to target when aiming for the hub */
    Supplier<Pose2d> currentRobotPose;
    Supplier<ChassisSpeeds> currentRobotSpeed;
    Supplier<Rotation2d> currentRobotHeading;

    Consumer<LoggableRobotPose> poseConsumer;

    LoggableRobotPose[] allPoses = new LoggableRobotPose[0];
    public double timeOfLastTrackedHubTarget = 0;
    Pose3d hubTarget = Pose3d.kZero;
    Rotation2d hubHeading = Rotation2d.kZero;
    double hubDistance = 0;
    Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
    public double robotY = 0;

    private final NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("CameraDetails");
    private final StructPublisher<Pose3d> hubTargetPublisher = cameraTable.getStructTopic("HubTarget", Pose3d.struct).publish();
    private final StructPublisher<Rotation2d> hubHeadingPublisher = cameraTable.getStructTopic("HubHeading", Rotation2d.struct).publish();
    private final SwerveRequest.FieldCentricFacingAngle targetHub = new SwerveRequest.FieldCentricFacingAngle();
    
    public LimelightVisionSystem(Consumer<LoggableRobotPose> poseConsumer, Supplier<Pose2d> currentRobotPose, 
                                 Supplier<ChassisSpeeds> currentRobotSpeed, Supplier<Rotation2d> currentRobotHeading) {
        this.poseConsumer = poseConsumer;
        this.currentRobotPose = currentRobotPose;
        this.currentRobotSpeed = currentRobotSpeed;
        this.currentRobotHeading = currentRobotHeading;
    }

    public void periodic() {
        Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Red);
        if (!Utils.isReplay()) {
            /* If this is not replay, get the hardware/simulated results from the camera */

            int[] hubTargetIds;
            /* Figure out if we should use red alliance hub ids or blue alliance hub ids */
            if (currentAlliance == Alliance.Red) {
                hubTargetIds = RedHubApriltagIds;
            } else {
                hubTargetIds = BlueHubApriltagIds;
            }

            var results = LimelightHelpers.getLatestResults("limelight");
            
            // LimelightTarget_Fiducial bestTarget = null;
            // for (LimelightTarget_Fiducial target : results.targets_Fiducials) {
            //     /* Check that the apriltag id is a hub ID */
            //     if (Arrays.stream(hubTargetIds).anyMatch(x -> x == (int)target.fiducialID)) {
            //         /* If we've never assigned the best target, use this one */
            //         if (bestTarget == null) {
            //             bestTarget = target;
            //         }
            //         /* Otherwise only update the target if this is a better ambiguity */
            //         else if (target.tx < bestTarget.tx) {
            //             bestTarget = target;
            //         }
            //     }
            // }
            // if (bestTarget != null) {
                /* Update our timestamp when we decide to use this target */
                timeOfLastTrackedHubTarget = Utils.getCurrentTimeSeconds();

                /* Process them */
                var cameraRobotPose = results.getBotPose2d_wpiBlue();
                /* Limelight always assumes 0,0 is your DS corner, it isn't always blue alliance */

                var hubTarget = currentAlliance == Alliance.Red ? RedHubTarget : BlueHubTarget;
                Pose2d robotPos = currentRobotPose.get();
                ChassisSpeeds robotSpeeds = currentRobotSpeed.get();
                Rotation2d robotHeading = currentRobotHeading.get();

                Translation2d robotTarget = hubTarget;
                for(int i = 0; i < 5; ++i) {
                    double timeOfFlightToTarget = SwerveUtils.timeOfFlight(robotPos, robotTarget);
                    robotTarget = SwerveUtils.getShiftedHubPose(robotSpeeds, robotHeading, timeOfFlightToTarget, robotTarget);
                }

                if (false) {
                    /* Set this to true to enable shoot on the move */
                    hubTarget = robotTarget;
                }

                var targetDelta = hubTarget.minus(cameraRobotPose.getTranslation());
                if (targetDelta.getX() == 0 && targetDelta.getY() == 0) {
                    /* Don't do anything */
                }
                else {
                    var angleToTarget = targetDelta.getAngle().rotateBy(DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red ? Rotation2d.kZero : Rotation2d.k180deg);
                    // var robotPose = currentRobotPose.get();
                    hubHeading = angleToTarget;
                    hubDistance = targetDelta.getNorm();

                    SmartDashboard.putNumber("Hub Heading", angleToTarget.getDegrees());
                    SmartDashboard.putNumber("Hub Distance", Units.metersToInches(hubDistance));
                }
                // Transform3d tagRelativeToRobot = bestTarget.getTargetPose_RobotSpace().minus(new Pose3d());
                // var transformToHub = currentAlliance == Alliance.Red ? RedHub.getHubPose((int)bestTarget.fiducialID) :
                //                                         BlueHub.getHubPose((int)bestTarget.fiducialID);
                // hubTarget = new Pose3d(robotPose).transformBy(tagRelativeToRobot).transformBy(transformToHub);
                // var hubRelativeToRobot = hubTarget.relativeTo(new Pose3d(robotPose));

                // double offsetX = bestTarget.tx;
            // }
            var robotPose = results.getBotPose3d_wpiBlue();
            if (robotPose != null) {
                poseConsumer.accept(new LoggableRobotPose(robotPose, results.timestamp_RIOFPGA_capture));
            }
        }
        robotY = currentRobotPose.get().getY();

        hubTargetPublisher.accept(hubTarget);
        hubHeadingPublisher.accept(hubHeading);
    }

    public void simPeriodic(Pose2d simPose) {
        // visionSim.update(simPose);
    }
    /** A Field2d for visualizing our robot and objects on the field. */
    // public Field2d getSimDebugField() {
    //     if (!Robot.isSimulation()) return null;
    //     return visionSim.getDebugField();
    // }
    public boolean isHubTargetValid() {
        return Utils.getCurrentTimeSeconds() - timeOfLastTrackedHubTarget < 0.2;
    }
    public Pose3d getHubPoseRelativeToRobot() {
        return hubTarget;
    }
    public Rotation2d getHeadingToHubFieldRelative() {
        return hubHeading;
    }
    public double getHubDistance() {
        return hubDistance;
    }
    // public Command PassToCorners() {
    //     if(currentAlliance == Alliance.Red){
    //         if (robotY > 4.0346376) {
    //             hubTarget = RedLeftCorner;
    //         } else{
    //             hubTarget = RedRightCorner;
    //         }
    //     } else if (currentAlliance == Alliance.Blue) {
    //         if (robotY > 4.0346376) {
    //             hubTarget = BlueRightCorner;
    //         } else{
    //             hubTarget = BlueLeftCorner;
    //         }
    //     }
    // }
}

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Flywheel.FlywheelSetpoint;

public class SwerveUtils {
    static final Translation2d redTarget = new Translation2d(0, 0);
    static final Translation2d blueTarget = new Translation2d(0, 0);

    static Angle DivotRange = Degrees.of(10);

    static double findDistanceToTarget(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

        double distanceToTarget = robotPose.getTranslation().getDistance(
            isRed ? redTarget : blueTarget
        );

        return Units.metersToInches(distanceToTarget);
    }

    static Translation2d driverDivot(Translation2d driverRequestedTranslation, Rotation2d Heading) {
        if (driverRequestedTranslation.getNorm() < 0.1) { return driverRequestedTranslation; }
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
        Rotation2d realForward = Heading.plus(isRed ? Rotation2d.k180deg : Rotation2d.kZero);
        var delta = driverRequestedTranslation.getAngle().minus(realForward).getMeasure();
        
        if (delta.isNear(Degrees.of(0), DivotRange)) {
            var speed = driverRequestedTranslation.getNorm();
            driverRequestedTranslation = new Translation2d(
                speed * realForward.getCos(),
                speed * realForward.getSin()
            );
        }

        return driverRequestedTranslation;
    }

    static private final InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
    static {
        table.put(57.0, 1.0); //in to sec
        table.put(97.0, 1.3);
        table.put(112.0, 1.45);
        table.put(132.0, 1.7);
        table.put(191.0, 2.4);
      //  table.put(, 70.0);
    };

    public static double timeOfFlight(Pose2d robotPose, Translation2d hubPose) {
        double robotX = robotPose.getX();
        double hubX = hubPose.getX();
        double robotY = robotPose.getY();
        double hubY = hubPose.getY();
        double XSquared = (robotX - hubX)*(robotX - hubX);
        double YSquared = (robotY - hubY)*(robotY - hubY);
        double distance = Math.sqrt(XSquared + YSquared);

        // robotPose.minus(hubPose).getTranslation().getNorm(); does same as above 6 lines
        return table.get(Units.metersToInches(distance));
    }

    public static Translation2d getShiftedHubPose(ChassisSpeeds robotSpeed, Rotation2d robotHeading, double timeOfFlight, Translation2d hubPose) {
        /* Get the velocity of the robot relative to the field */
        ChassisSpeeds fieldRelativeVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeed, robotHeading);

        /* Take the velocity and figure out a translation2d that covers the distance given the time of flight */
        Twist2d changeInPose = fieldRelativeVelocity.toTwist2d(timeOfFlight);
        Translation2d changeInPosition = new Translation2d(-changeInPose.dx, -changeInPose.dy);

        /* Apply that changeInPosition to the hub's pose to figure out where to aim */
        Translation2d shiftedHub = hubPose.plus(changeInPosition);

        return shiftedHub;
    }
}

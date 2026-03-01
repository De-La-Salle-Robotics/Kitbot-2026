package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
}

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class SwerveUtils {
    static final Translation2d redTarget = new Translation2d(0, 0);
    static final Translation2d blueTarget = new Translation2d(0, 0);

    static double findDistanceToTarget(Pose2d robotPose) {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

        double distanceToTarget = robotPose.getTranslation().getDistance(
            isRed ? redTarget : blueTarget
        );

        return Units.metersToInches(distanceToTarget);
    }
}

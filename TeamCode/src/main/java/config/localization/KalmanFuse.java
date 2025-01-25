package config.localization;

import static config.localization.Limelight.LX;
import static config.localization.Limelight.LY;
import static config.localization.Limelight.fiducialResults;
import config.localization.Limelight;
import java.util.List;

import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/** KalmanFuse class for fusing Pedro Pathing and Limelight poses with Kalman filter.
 * Credit to 20278 Neurobots for their original class this was based on
 */
public class KalmanFuse {

    private Pose fusedPose;  // The final fused pose using Kalman filter
    private Pose pedroPose;  // Pedro Pathing pose
    private static Pose weighPose = new Pose(0,0,0);

    public static Pose rawPedroPose;

    private KalmanFilter kalmanFilter;

    public void KalmanInit() {
        this.fusedPose = new Pose(0, 0, 0);  // Initialize fused pose
        this.pedroPose = new Pose(0, 0, 0);  // Initialize Pedro Pathing pose
        this.kalmanFilter = new KalmanFilter();
    }

    /**
     * Updates the localization by fusing the Pedro Pathing pose and Limelight pose.
     */
    public void updateLocalization(Pose followerPose, Limelight LimeClass) {
        pedroPose = new Pose(followerPose.getX(), followerPose.getY(), followerPose.getHeading());

        LimeClass.getPose();
        // Convert Limelight bot position to Pose2d
        //Make sure to take the Follower Heading as the Limelight does not have one
        Pose LimelightPose2d = new Pose(LX, LY, 0);
        if (LimelightPose2d.getX() == 72 && LimelightPose2d.getY() == 72
        || LimelightPose2d.getX() < 0 || LimelightPose2d.getY() < 0) {
            //If limelight is blocked only use the OTOS/PedroPathing pose
            //fusedPose will be the new followerPose which should already incorporate the new OTOS changes
            fusedPose = followerPose;
            return;
        }

        // Fuse the poses using Kalman filter
        fusedPose = kalmanFilter.fuse(pedroPose, LimelightPose2d);
    }

    /**
     * Returns the fused pose.
     *
     * @return The fused Pose2d.
     */
    public Pose getFusedPose() {
        return fusedPose;
    }

    /**
     * Kalman Filter for fusing the poses.
     */
    private static class KalmanFilter {


        /**
         * Fuses the Pedro and Limelight poses.
         *
         * @param pedroPose    Pose from Pedro Pathing.
         * @param LimelightPose Pose from Limelight localization.
         * @return The fused Pose2d.
         */
        public Pose fuse(Pose pedroPose, Pose LimelightPose) {

            if (weighPose.getX() > 144 || weighPose.getY() > 144){
                return pedroPose;
            } else if (fiducialResults.size() > 1){
            weighPose = new Pose((pedroPose.getX() * 0.95) + (LimelightPose.getX() * 0.05),
                    (pedroPose.getY() * 0.95) + (LimelightPose.getY() * 0.05),
                    pedroPose.getHeading());
            } else {
                return pedroPose;
            }


            return weighPose;
        }
    }
}

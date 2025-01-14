package config.localization;

import static config.localization.Limelight.LX;
import static config.localization.Limelight.LY;
import config.localization.Limelight;

import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/** KalmanFuse class for fusing Pedro Pathing and Limelight poses with Kalman filter.
 * Credit to 20278 Neurobots for their original class this was based on
 */
public class KalmanFuse {

    private Pose fusedPose;  // The final fused pose using Kalman filter

    private KalmanFilter kalmanFilter;

    public void KalmanInit() {
        this.fusedPose = new Pose(0, 0, 0);  // Initialize fused pose
        this.kalmanFilter = new KalmanFilter();
    }

    /**
     * Updates the localization by fusing the Pedro Pathing pose and Limelight pose.
     */
    public void updateLocalization(Pose followerPose, Limelight LimeClass) {

        // Get the current pose from Pedro Pathing
        Pose pedroPose = followerPose;  // Retrieve pose from Pedro Pathing

        LimeClass.getPose();
        // Convert Limlight bot position to Pose2d
        //Make sure to take the Follower Heading as the Limelight does not have one
        Pose LimelightPose2d = new Pose(LX, LY, 0);

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

        // Kalman filter state variables
        private double[] x = new double[3];  // [x, y, heading]
        private double[][] P = new double[3][3];  // Covariance matrix

        //Adjust weight for the Kalman Filter here
        private double R = 0.07;  // Measurement noise (Limelight Pose)
        private double Q = 0.2;  // Process noise (Pedro Pose)

        public KalmanFilter() {
            // Initialize covariance matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    P[i][j] = 0.1;  // Initial estimate covariance
                }
            }
        }

        /**
         * Fuses the Pedro and Limelight poses.
         *
         * @param pedroPose    Pose from Pedro Pathing.
         * @param LimelightPose Pose from Limelight localization.
         * @return The fused Pose2d.
         */
        public Pose fuse(Pose pedroPose, Pose LimelightPose) {
            // Prediction step (assuming Pedro Pose is the prediction model)
            double[] z = new double[]{pedroPose.getX(), pedroPose.getY(), pedroPose.getHeading()};
            double[] measurement = new double[]{LimelightPose.getX(), LimelightPose.getY(), 0};

            // Innovation
            double[] y = new double[3];
            for (int i = 0; i < 3; i++) {
                y[i] = measurement[i] - z[i];
            }

            // Kalman Gain
            double[][] K = new double[3][3];
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    K[i][j] = P[i][j] / (P[i][i] + R);
                }
            }

            // Update estimate with innovation
            for (int i = 0; i < 3; i++) {
                x[i] = x[i] + K[i][0] * y[0] + K[i][1] * y[1] + K[i][2] * y[2];
            }

            // Update covariance matrix
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    P[i][j] = P[i][j] - K[i][j] * P[j][i];
                }
            }

            // Return the fused pose
            return new Pose(x[0], x[1], x[2]);
        }
    }
}

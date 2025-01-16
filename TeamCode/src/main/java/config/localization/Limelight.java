package config.localization;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limelight {

    private Limelight3A limelight;
    private Follower follower;
    public static double LX;
    public static double LY;
    public static double LZ;
    public static double MT2Heading;
    public static double startHeading;
    /**
     * Initialize the Limelight
     */
    public void LimelightInit(Limelight3A limelight, Follower follower, Pose startPose) {
        this.limelight = limelight;
        this.follower = follower;
        startHeading = startPose.getHeading();
        limelight.start();
        limelight.pipelineSwitch(0);
    }

    /** MegaTag2 requires we provide the Limelight with the robot heading
     *  It assumes we are providing it a heading in degrees that comes from the IMU starting at 0
     *  We are collecting the BotPose from the Limelight
     *  Then we multiply to convert from meters to inches and add 72 to match with PedroPathing
     */
    public void getPose() {
        MT2Heading = follower.getTotalHeading(); // follower.getPose().getHeading(); is bounded 0-2 Pi
        limelight.updateRobotOrientation(Math.toDegrees(MT2Heading - startHeading));
        Pose3D botpose = limelight.getLatestResult().getBotpose_MT2();

        LX = botpose.getPosition().x * 39.37007874 + 72;
        LY = botpose.getPosition().y * 39.37007874 + 72;
        LZ = botpose.getPosition().z * 39.37007874 + 72;
    }
}

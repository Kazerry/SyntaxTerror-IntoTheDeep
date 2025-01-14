package config.localization;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;



public class Limelight {

    private Limelight3A limelight;
    public static double LX;
    public static double LY;
    public static double LZ;

    /**
     * Initialize the Limelight
     */
    public void LimelightInit(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.start();
    }

    /** We are collecting the BotPose from the Limelight
     *  Then we multiply to convert from meters to inches and add 72 to match with PedroPathing
     */
    public void getPose() {
        Pose3D botpose = limelight.getLatestResult().getBotpose();

        LX = botpose.getPosition().x * 39.37 + 72;
        LY = botpose.getPosition().y * 39.37 + 72;
        LZ = botpose.getPosition().z * 39.37 + 72;
    }
}

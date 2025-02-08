package config;

import com.acmerobotics.dashboard.config.Config;

/** Everything that we want to store globally, for example positions of servos, motors, etc. goes in here. **/
@Config
public class RobotHardware {

    /** Variables are positions for the claw servos. **/
    public static double closeClaw = 0.170;
    public static double openClaw = 0.025;

    /** Variables are positions for the Arm Motors. **/
    public static int armUp = 0;
    public static int armDown = 0;

    /** Variables are positions for the Slide Motors. **/
    public static int slideUp = 120;
    public static int slideDown = 0;

    /** Variables are color values for the color sensor **/
    public static int yellowBlueMin = 200, yellowBlueMax = 700;
    public static int yellowRedMin = 1000, yellowRedMax = 2030;
    public static int yellowGreenMin = 1000, yellowGreenMax = 2550;


    /** TEST position **/
    public static int PosTestExtension = -100;
    public static int PosTestPivot = 300;
    public static double PosTestBicep = 0.5;
    public static double PosTestForearm = 0.5;
    public static double PosTestRotation = 0.3;

    //River Pivot things
    public static boolean inThresh(double val, double val2, double tol) {
        return Math.abs(val - val2) < tol;
    }

}
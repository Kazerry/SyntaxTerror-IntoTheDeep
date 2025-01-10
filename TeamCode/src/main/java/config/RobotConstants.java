package config;

/** Everything that we want to store globally, for example positions of servos, motors, etc. goes in here. **/
public class RobotConstants {

    /** Variables are positions for the claw servos. **/
    public static double closeClaw = 0.237;
    public static double openClaw = 0.03;

    /** Variables are positions for the Arm Motors. **/
    public static int armUp = 0;
    public static int armDown = 0;

    /** Variables are positions for the Slide Motors. **/
    public static int slideUp = 500;
    public static int slideDown = 0;

    /** Variables are color values for the color sensor **/
    public static int yellowBlueMin = 200, yellowBlueMax = 700;
    public static int yellowRedMin = 1000, yellowRedMax = 2030;
    public static int yellowGreenMin = 1000, yellowGreenMax = 2550;

}
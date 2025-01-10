package config.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import config.RobotConstants;

public class ClawSubsystem {
    // Servo for the claw
    private final Servo clawServo;
    private final ColorSensor colorSensor;

    // Claw state
    private boolean isClawClosed = false;

    // Timing and detection thresholds
    private final ElapsedTime clawTimer = new ElapsedTime();
    private static final double CLAW_HOLD_TIME = 1.0; // 1 second hold time
    private int yellowDetectionCount = 0;


    // Detection thresholds
    private final int yellowDetectionThreshold = 3;

    public ClawSubsystem(Servo clawServo, ColorSensor colorSensor) {
        this.clawServo = clawServo;
        this.colorSensor = colorSensor;

        // Initialize the claw as open
        this.clawServo.setPosition(RobotConstants.openClaw);
    }

    /**
     * Checks if the current color sensor readings indicate yellow.
     * @return true if yellow is detected
     */
    private boolean isYellowDetected() {
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        int red = colorSensor.red();

        return (blue > RobotConstants.yellowBlueMin && blue < RobotConstants.yellowBlueMax &&
                green > RobotConstants.yellowGreenMin && green < RobotConstants.yellowGreenMax &&
                red > RobotConstants.yellowRedMin && red < RobotConstants.yellowRedMax);
    }

    /**
     * Automatically manages the claw's state based on yellow detection.
     */
    public void manageClaw() {
        if (isYellowDetected()) {
            yellowDetectionCount++;

            if (yellowDetectionCount >= yellowDetectionThreshold && !isClawClosed) {
                clawServo.setPosition(RobotConstants.closeClaw);
                isClawClosed = true;
                clawTimer.reset();
            }
        } else {
            yellowDetectionCount = 0;

            if (isClawClosed && clawTimer.seconds() > CLAW_HOLD_TIME) {
                clawServo.setPosition(RobotConstants.openClaw);
                isClawClosed = false;
            }
        }
    }

    /**
     * Overrides to manually close the claw.
     */
    public void closeClaw() {
        clawServo.setPosition(RobotConstants.closeClaw);
        isClawClosed = true;
        clawTimer.reset();
    }

    /**
     * Overrides to manually open the claw.
     */
    public void openClaw() {
        clawServo.setPosition(RobotConstants.openClaw);
        isClawClosed = false;
    }
}
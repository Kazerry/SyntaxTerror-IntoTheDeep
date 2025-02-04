package config.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import config.RobotHardware;

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
        this.clawServo.setPosition(RobotHardware.openClaw);
    }

    /**
     * Checks if the current color sensor readings indicate yellow.
     * @return true if yellow is detected
     */
    private boolean isYellowDetected() {
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        int red = colorSensor.red();

        return (blue > RobotHardware.yellowBlueMin && blue < RobotHardware.yellowBlueMax &&
                green > RobotHardware.yellowGreenMin && green < RobotHardware.yellowGreenMax &&
                red > RobotHardware.yellowRedMin && red < RobotHardware.yellowRedMax);
    }

    /**
     * Automatically manages the claw's state based on yellow detection.
     */
    public void manageClaw() {
        if (isYellowDetected()) {
            yellowDetectionCount++;

            if (yellowDetectionCount >= yellowDetectionThreshold && !isClawClosed) {
                clawServo.setPosition(RobotHardware.closeClaw);
                isClawClosed = true;
                clawTimer.reset();
            }
        } else {
            yellowDetectionCount = 0;

            if (isClawClosed && clawTimer.seconds() > CLAW_HOLD_TIME) {
                clawServo.setPosition(RobotHardware.openClaw);
                isClawClosed = false;
            }
        }
    }

    /**
     * Overrides to manually close the claw.
     */
    public void closeClaw() {
        clawServo.setPosition(RobotHardware.closeClaw);
        isClawClosed = true;
        clawTimer.reset();
    }

    /**
     * Overrides to manually open the claw.
     */
    public void openClaw() {
        clawServo.setPosition(RobotHardware.openClaw);
        isClawClosed = false;
    }
}
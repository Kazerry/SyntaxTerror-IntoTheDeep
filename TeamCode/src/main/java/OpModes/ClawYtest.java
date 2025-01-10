package OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import pedroPathing.constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * @version 1.1, 12/13/2024
 * Enhanced Claw Intelligence OpMode
 */
@Disabled
@Config
@TeleOp(name = "ClawYtest", group = "Test")
public class ClawYtest extends OpMode {
    private Follower follower;

    // Motor and Servo Declarations
    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private Servo clawServo, testServo;

    // Sensor and Dashboard
    private ColorSensor colorSensor;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    // Claw and Movement Configuration
    public static double closeClaw = 0.237;
    public static double openClaw = 0.03;

    // Yellow Detection Thresholds
    public static int YELLOW_BLUE_MIN = 200;
    public static int YELLOW_BLUE_MAX = 700;
    public static int YELLOW_GREEN_MIN = 1000;
    public static int YELLOW_GREEN_MAX = 2550;
    public static int YELLOW_RED_MIN = 1000;
    public static int YELLOW_RED_MAX = 2030;

    // Claw State Management
    private boolean speedToggle = false;
    private boolean isClawClosed = false;
    private ElapsedTime clawTimer = new ElapsedTime();
    private static final double CLAW_HOLD_TIME = 1.0; // 1 second hold time

    // Advanced Claw Detection
    private int yellowDetectionCount = 0;
    private static final int YELLOW_DETECTION_THRESHOLD = 3; // Require multiple consecutive detections

    @Override
    public void init() {
        // Initialize Follower and Drive Motors
        follower = new Follower(hardwareMap);

        // Initialize Servos and Sensors
        clawServo = hardwareMap.get(Servo.class, "Cservo");
        testServo = hardwareMap.get(Servo.class, "testservo");
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Set Motor Behaviors
        setMotorZeroPowerBehavior();

        // Start Teleoperation Drive
        follower.startTeleopDrive();
        follower.setMaxPower(1);

        // Open Claw Initially
        clawServo.setPosition(openClaw);
    }

    /**
     * Set zero power behavior for all drive motors
     */
    private void setMotorZeroPowerBehavior() {
        DcMotor[] motors = {leftFront, leftRear, rightRear, rightFront};
        for (DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Check if current color reading represents yellow
     * @return true if yellow is detected
     */
    private boolean isYellowDetected() {
        int blue = colorSensor.blue();
        int green = colorSensor.green();
        int red = colorSensor.red();

        return (blue > YELLOW_BLUE_MIN && blue < YELLOW_BLUE_MAX &&
                green > YELLOW_GREEN_MIN && green < YELLOW_GREEN_MAX &&
                red > YELLOW_RED_MIN && red < YELLOW_RED_MAX);
    }

    /**
     * Manage claw state based on color detection and timing
     */
    private void manageClaw() {
        if (isYellowDetected()) {
            // Reset detection counter and timer
            yellowDetectionCount++;
            clawTimer.reset();

            // Only close if we've had multiple consecutive yellow detections
            if (yellowDetectionCount >= YELLOW_DETECTION_THRESHOLD && !isClawClosed) {
                clawServo.setPosition(closeClaw);
                isClawClosed = true;
            }
        } else {
            // Reset detection counter
            yellowDetectionCount = 0;

            // If claw is closed, check how long it's been closed
            if (isClawClosed) {
                if (clawTimer.seconds() > CLAW_HOLD_TIME) {
                    // Open claw if no yellow detected for more than specified time
                    clawServo.setPosition(openClaw);
                    isClawClosed = false;
                }
            }
        }
    }

    @Override
    public void loop() {
        // Manage intelligent claw behavior
        manageClaw();

        // Telemetry for Debugging
        telemetry.addLine("Advanced Claw Intelligence:");
        telemetry.addData("Claw State", isClawClosed ? "Closed" : "Open");
        telemetry.addData("Yellow Detection Count", yellowDetectionCount);
        telemetry.addData("Claw Hold Time", clawTimer.seconds());

        telemetry.addLine("\nColor Sensor Readings:");
        telemetry.addData("Red", colorSensor.red());
        telemetry.addData("Blue", colorSensor.blue());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Servo Position", clawServo.getPosition());

        // Speed Toggle Telemetry
        telemetry.addLine(speedToggle ? "Robot Speed: Half" : "Robot Speed: Full");
        telemetry.update();

        // Manual Claw and Servo Control
        handleManualControls();

        // Dashboard and Follower Update
        dashboard.updateConfig();
        follower.setTeleOpMovementVectors(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
        follower.update();
    }

    /**
     * Handle manual controls for claw and other servos
     */
    private void handleManualControls() {
        // Manual Claw Control
        if (gamepad1.y) {
            clawServo.setPosition(closeClaw);
            isClawClosed = true;
            clawTimer.reset();
        }

        if (gamepad1.a) {
            clawServo.setPosition(openClaw);
            isClawClosed = false;
        }

        // Test Servo Controls
        if (gamepad1.dpad_down) {
            testServo.setPosition(0.35);
        }
        if (gamepad1.dpad_up) {
            testServo.setPosition(0.6);
        }
    }
}
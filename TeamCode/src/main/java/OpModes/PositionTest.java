package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import config.subsystems.Pivot;
import config.subsystems.Extension;
import config.subsystems.Wrist;

@Config
@TeleOp(name = "PositionTest", group = "Test")
public class PositionTest extends OpMode {
    private Extension extension;
    private Pivot pivot;
    private Wrist wrist;

    private DcMotorEx rightExtension;
    private DcMotorEx leftExtension;
    private DcMotorEx rightPivot;
    private DcMotorEx leftPivot;
    private TouchSensor extLimit;

    private Servo bicepLeft;
    private Servo bicepRight;
    private Servo forearm;
    private Servo rotation;

    // Track current test positions
    private String currentPivotPos = "";
    private String currentExtensionPos = "";
    private String currentBicepPos = "";
    private String currentForearmPos = "";

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    @Override
    public void init() {
        // Initialize motors
        rightExtension = hardwareMap.get(DcMotorEx.class, "rightExtension");
        leftExtension = hardwareMap.get(DcMotorEx.class, "leftExtension");
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightPivot");
        leftPivot = hardwareMap.get(DcMotorEx.class, "leftPivot");
        extLimit = hardwareMap.get(TouchSensor.class, "extLimit");

        // Initialize servos
        bicepLeft = hardwareMap.get(Servo.class, "bicepLeft");
        bicepRight = hardwareMap.get(Servo.class, "bicepRight");
        forearm = hardwareMap.get(Servo.class, "forearm");
        rotation = hardwareMap.get(Servo.class, "rotation");

        // Reset encoders
        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize subsystems
        extension = new Extension(leftExtension, rightExtension, extLimit);
        pivot = new Pivot(hardwareMap, rightPivot, leftPivot);
        wrist = new Wrist(bicepLeft, bicepRight, forearm, rotation);

        //initialize dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        // Update subsystems
        extension.update();
        pivot.update();
        wrist.update();

        // Continuously update positions if buttons are pressed
        if (gamepad1.y) {
            currentPivotPos = "PosTestPivot";
            pivot.setPos(currentPivotPos);
        }

        if (gamepad1.b) {
            currentExtensionPos = "PosTestExtension";
            extension.setPos(currentExtensionPos);
        }

        if (gamepad1.x) {
            currentBicepPos = "PosTestBicep";
            wrist.setBicepPos(currentBicepPos);
        }

        if (gamepad1.a) {
            currentForearmPos = "PosTestForearm";
            wrist.setForearmPos(currentForearmPos);
        }

        // If a position is active, continuously update it to get new dashboard values
        if (!currentPivotPos.isEmpty()) {
            pivot.setPos(currentPivotPos);
        }
        if (!currentExtensionPos.isEmpty()) {
            extension.setPos(currentExtensionPos);
        }
        if (!currentBicepPos.isEmpty()) {
            wrist.setBicepPos(currentBicepPos);
        }
        if (!currentForearmPos.isEmpty()) {
            wrist.setForearmPos(currentForearmPos);
        }

        // Manual adjustments using D-pad
        if (gamepad1.dpad_up && !lastDpadUp) {
            // Increment the current test position
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            // Decrement the current test position
        }

        // Update button states
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        // Telemetry
        telemetry.addData("=== Active Positions ===", "");
        telemetry.addData("Current Pivot Position", currentPivotPos);
        telemetry.addData("Current Extension Position", currentExtensionPos);
        telemetry.addData("Current Bicep Position", currentBicepPos);
        telemetry.addData("Current Forearm Position", currentForearmPos);

        telemetry.addData("=== Motors ===", "");
        telemetry.addData("Right Extension Pos", rightExtension.getCurrentPosition());
        telemetry.addData("Left Extension Pos", leftExtension.getCurrentPosition());
        telemetry.addData("Right Pivot Pos", rightPivot.getCurrentPosition());
        telemetry.addData("Left Pivot Pos", leftPivot.getCurrentPosition());

        telemetry.addData("=== Servos ===", "");
        telemetry.addData("Bicep Left Pos", bicepLeft.getPosition());
        telemetry.addData("Bicep Right Pos", bicepRight.getPosition());
        telemetry.addData("Forearm Pos", forearm.getPosition());
        telemetry.addData("Rotation Pos", rotation.getPosition());

        telemetry.addData("=== Controls ===", "");
        telemetry.addLine("Y - Test Pivot Position");
        telemetry.addLine("B - Test Extension Position");
        telemetry.addLine("X - Test Bicep Position");
        telemetry.addLine("A - Test Forearm Position");

        telemetry.update();
    }
}
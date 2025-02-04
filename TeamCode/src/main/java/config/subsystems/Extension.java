package config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.HashMap;

import config.RobotConstants;

@Config
public class Extension {

    private DcMotorEx leftExtension, rightExtension;
    private TouchSensor limitSwitch;
    private boolean wasPressed = false; // Track previous limit switch state
    private int pos;
    private int curLeft;
    private int manualPos;
    private boolean isManualMode = false;

    public static double kP = 0.0075, kI = 0, kD = 0;
    PIDController pidController = new PIDController(kP, kI, kD);
    public static int PIDTol = 10, PIDThresh = 10;

    public static HashMap<String, Integer> positions = new HashMap<String, Integer>();

    public Extension(DcMotorEx leftExtension, DcMotorEx rightExtension, TouchSensor limitSwitch) {
        this.leftExtension = leftExtension;
        this.rightExtension = rightExtension;
        this.limitSwitch = limitSwitch;

        leftExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        // Stop motors
        leftExtension.setPower(0);
        rightExtension.setPower(0);

        // Set run mode
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior to brake
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pidController.setTolerance(PIDTol);

        positions.put("Intake", -1000);
        positions.put("Idle",   -50);
        positions.put("Place",   -1350);
        positions.put("downPlace",   -150);
        positions.put("Basket", -2300);
        positions.put("Hang", -2500);
        positions.put("Retract", -1400);
        positions.put("Specified", -1000); // Added for fine-tuning when under bars
    }

    private void configureMotors() {
        // Stop motors
        leftExtension.setPower(0);
        rightExtension.setPower(0);

        // Reset encoders
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait briefly for reset to complete
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            // Handle interruption if needed
        }

        // Set run mode
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior to brake
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update() {
        // Check for limit switch press transition (was not pressed, now is pressed)
        boolean isPressed = limitSwitch.isPressed();
        if (isPressed && !wasPressed) {
            // Stop motors immediately
            applyPower(0);

            // Reset encoders to 0 when limit switch is pressed
            leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Reconfigure motors
            configureMotors();

            // Reset position to 0
            pos = 0;
        }
        wasPressed = isPressed;

        // Get current position
        curLeft = leftExtension.getCurrentPosition();

        // Apply PID control
        if (RobotConstants.inThresh(curLeft, pos, PIDThresh)) {
            applyPower(0);
        } else {
            double pidPower = pidController.calculate(curLeft, pos);
            // Limit power when extending (moving away from limit switch)
            if (curLeft < pos) {
                pidPower = Math.min(pidPower, 0.7); // Adjust max power as needed
            }
            applyPower(pidPower);
        }
    }

    public void setManualPos(int manualPosition) {
        // Ensure the manual position is within safe bounds
        // 0 is fully retracted (limit switch), -2000 is fully extended
        this.manualPos = Math.min(0, Math.max(-2000, manualPosition));
        this.pos = manualPos;
        this.isManualMode = true;
    }

    public void setPos(String pos) {
        this.pos = positions.get(pos);
        this.isManualMode = false;
    }

    public void applyPower(double power) {
        // Add safety check for limit switch
        if (limitSwitch.isPressed() && power < 0) {
            power = 0; // Prevent moving further into limit switch
        }
        leftExtension.setPower(power);
        rightExtension.setPower(power);
    }

    // Add getters for debugging if needed
    public int getCurrentPosition() {
        return curLeft;
    }

    public boolean isLimitPressed() {
        return limitSwitch.isPressed();
    }
}
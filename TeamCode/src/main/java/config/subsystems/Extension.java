package config.subsystems;

import static config.RobotHardware.PosTestExtension;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.HashMap;

import config.RobotHardware;

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
        positions.put("Idle", -50);
        positions.put("Place", -1350);
        positions.put("downPlace", -150);
        positions.put("Basket", -2300);
        positions.put("Hang", -2500);
        positions.put("Retract", -1400);
        positions.put("Specified", -50);
        positions.put("Observe", -2000);
        positions.put("PosTestExtension", PosTestExtension); // Added for fine-tuning when under bars
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

        // Apply PID control or manual control
        if (isManualMode) {
            // Calculate manual power based on the difference between target and current position
            double power = calculateManualPower();
            applyPower(power);
        } else if (RobotHardware.inThresh(curLeft, pos, PIDThresh)) {
            applyPower(0); // Stop if within threshold
        } else {
            // Use PID control
            double pidPower = pidController.calculate(curLeft, pos);
            // Limit power when extending (moving away from limit switch)
            if (curLeft < pos) {
                pidPower = Math.min(pidPower, 0.7); // Adjust max power as needed
            }
            applyPower(pidPower);
        }
    }

    private double calculateManualPower() {
        double power = 0;
        double maxPower = 0.7; // Increase max power for better responsiveness
        int error = manualPos - curLeft;

        if (Math.abs(error) > 10) { // Deadband to prevent jitter
            power = error * 0.001; // Proportional control
            power = Math.max(-maxPower, Math.min(maxPower, power)); // Clamp power
        }

        return power;
    }

    public void setManualPos(int manualPosition) {
        this.manualPos = manualPosition;
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
}
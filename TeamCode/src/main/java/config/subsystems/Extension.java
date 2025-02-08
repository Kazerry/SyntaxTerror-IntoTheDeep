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
    private boolean wasPressed = false;
    private int pos;
    private int curLeft;

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
        positions.put("Place", -610);
        positions.put("downPlace", -150);
        positions.put("Basket", -2300);
        positions.put("Hang", -2500);
        positions.put("Retract", -1400);
        positions.put("Specified", -50);
        positions.put("Observe", -500);
        positions.put("PosTestExtension", PosTestExtension);
    }

    public void update() {
        // Check for limit switch press transition
        boolean isPressed = limitSwitch.isPressed();
        if (isPressed && !wasPressed) {
            applyPower(0);
            leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            configureMotors();
            pos = 0;
        }
        wasPressed = isPressed;

        curLeft = leftExtension.getCurrentPosition();

        if (RobotHardware.inThresh(curLeft, pos, PIDThresh)) {
            applyPower(0);
        } else {
            double pidPower = pidController.calculate(curLeft, pos);
            if (curLeft < pos) {
                pidPower = Math.min(pidPower, 0.7);
            }
            applyPower(pidPower);
        }
    }

    public void setPos(String pos) {
        this.pos = positions.get(pos);
    }

    public void applyPower(double power) {
        if (limitSwitch.isPressed() && power < 0) {
            power = 0;
        }
        leftExtension.setPower(power);
        rightExtension.setPower(power);
    }

    public int getCurrentPosition() {
        return curLeft;
    }

    public boolean isLimitPressed() {
        return limitSwitch.isPressed();
    }

    private void configureMotors() {
        leftExtension.setPower(0);
        rightExtension.setPower(0);
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            // Handle interruption if needed
        }
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightExtension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
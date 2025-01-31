package config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import config.RobotConstants;

@Config
public class Extension {

    private DcMotorEx leftExtension, rightExtension;
    private TouchSensor limitSwitch;  // Added limit switch

    private int pos;
    private int curLeft;

    public static double kP = 0.005, kI = 0, kD = 0;
    PIDController pidController = new PIDController(kP, kI, kD);
    public static int PIDTol = 10, PIDThresh = 10;

    public static HashMap<String, Integer> positions = new HashMap<String, Integer>();

    public Extension(DcMotorEx leftExtension, DcMotorEx rightExtension, TouchSensor limitSwitch)
    {
        this.leftExtension = leftExtension;
        this.rightExtension = rightExtension;
        this.limitSwitch = limitSwitch;

        leftExtension.setDirection(DcMotorSimple.Direction.REVERSE);

        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidController.setTolerance(PIDTol);

        positions.put("Intake", -1000);
        positions.put("Idle",   -50);
        positions.put("Place",   -1200);
        positions.put("downPlace",   -950);
        positions.put("Basket", -2300);
        positions.put("Hang", -2500);
        positions.put("Retract", -1400);
    }

    public void update()
    {
        // Check limit switch
        if (limitSwitch.isPressed()) {
            // Reset encoders when limit switch is pressed
            leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        curLeft = leftExtension.getCurrentPosition();

        if (RobotConstants.inThresh(curLeft, pos, PIDThresh))
        {
            applyPower(0);
        }
        else{
            applyPower(pidController.calculate(curLeft, pos));
        }
    }

    public void applyPower(double power)
    {
        leftExtension.setPower(power);
        rightExtension.setPower(power);
    }

    public void setPos(String pos) {
        this.pos = positions.get(pos);
    }
}
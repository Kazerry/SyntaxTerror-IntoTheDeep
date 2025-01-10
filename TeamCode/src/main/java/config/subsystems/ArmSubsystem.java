package config.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import config.RobotConstants;

public class ArmSubsystem {
    private final PIDController controller;
    private final DcMotorEx armMotor;

    // PIDF constants
    private double p, i, d, f;

    // Conversion factor for ticks to degrees
    private final double ticksInDegree;


    public ArmSubsystem(DcMotorEx armMotor, double p, double i, double d, double f, double ticksInDegree) {
        this.armMotor = armMotor;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.ticksInDegree = ticksInDegree;

        controller = new PIDController(p, i, d);
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        controller.setPID(p, i, d);
    }

    public void setTargetPosition(int target) {
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticksInDegree)) * f;
        double power = pid + ff;

        armMotor.setPower(power);
    }

    public void raise() {
        setTargetPosition(RobotConstants.armUp);
    }

    public void lower() {
        setTargetPosition(RobotConstants.armDown);
    }

    public boolean isAtTarget(int target, double tolerance) {
        int armPos = armMotor.getCurrentPosition();
        return Math.abs(target - armPos) <= tolerance;
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }
}
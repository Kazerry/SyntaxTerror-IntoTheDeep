package config.subsystems.archiveSubsystems;

import static config.RobotHardware.slideDown;
import static config.RobotHardware.slideUp;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class pivotSubsystem {
    private PIDController controller;
    private DcMotorEx slide1, slide2;

    // PIDF constants
    private double p, i, d, f;

    // Conversion factor for ticks to degrees
    private double ticksInDegree;

    private int targetPosition = 0;

    public pivotSubsystem(DcMotorEx slide1, DcMotorEx slide2, double p, double i, double d, double f, double ticksInDegree) {
        this.slide1 = slide1;
        this.slide2 = slide2;
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        this.ticksInDegree = ticksInDegree;

        controller = new PIDController(p, i, d);

        slide2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.f = f;
        controller.setPID(p, i, d);
    }

    public void setTargetPosition(int target){
        targetPosition = target;
    }
    public void update() {
        int slidePos1 = slide1.getCurrentPosition();
        int slidePos2 = slide2.getCurrentPosition();

        double pid1 = controller.calculate(slidePos1, targetPosition);
        double pid2 = controller.calculate(slidePos2, targetPosition);

        double ff = Math.cos(Math.toRadians(targetPosition / ticksInDegree)) * f;

        double power1 = pid1 + ff;
        double power2 = pid2 + ff;

        slide1.setPower(power1);
        slide2.setPower(power2);
    }

    public void moveToBottom() {
        setTargetPosition(slideDown);
    }

    public void moveToTop() {
        setTargetPosition(slideUp);
    }

    public boolean isAtTarget(int target, double tolerance) {
        int slidePos1 = slide1.getCurrentPosition();
        int slidePos2 = slide2.getCurrentPosition();

        return Math.abs(targetPosition - slidePos1) <= tolerance && Math.abs(targetPosition - slidePos2) <= tolerance;
    }

    public int getCurrentPosition() {
        return (slide1.getCurrentPosition() + slide2.getCurrentPosition()) / 2;
    }
}
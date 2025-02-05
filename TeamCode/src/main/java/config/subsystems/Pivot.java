package config.subsystems;

import static config.RobotHardware.PosTestPivot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

import config.RobotHardware;

@Config
public class Pivot {
    private DcMotorEx leftPivot, rightPivot;

    public static double power = 0, lastPower = power;

    public static int MAX_PIVOT_VELOCITY = 2000;
    public static int MAX_PIVOT_ACCEL = 400;

    public static int Down = 0;
    public static int Lift = 330;
    public static int Hang = 310;
    public static int Start = 235;
    public static int Basket = 315;
    public static int Idle = 315;
    public static int PInit = 450;
    public static int PGrab = 100;



    private int pos = 0;
    private int curLeft = 0, lAngle = curLeft;
    private int lta = 0;

    public static double kP = 0.02, kI = 0, kD = 0.0005, k = 0, extendedKp = 0.03, zeroKp = 0.005, slowKp = 0.02;

    PIDController pidController = new PIDController(kP, kI, kD);

    TrapezoidProfile profile;
    TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_PIVOT_VELOCITY, MAX_PIVOT_ACCEL);

    ElapsedTime fullTimer = new ElapsedTime();
    ElapsedTime velTimer = new ElapsedTime();

    public static HashMap<String, Integer> positions = new HashMap<String, Integer>();
    public static HashMap<String, Double> kPs = new HashMap<String, Double>();

    double aVelocity, indexedPosition = 0;

    public Pivot(HardwareMap hwMap, DcMotorEx rightPivot, DcMotorEx leftPivot) {
        this.leftPivot = leftPivot;
        this.rightPivot = rightPivot;

        leftPivot.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Reset encoders once at initialization
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(0, 0));

        positions.put("Down", Down);
        positions.put("Basket", Basket);
        positions.put("Idle", Idle);
        positions.put("Start", Start);
        positions.put("Hang", Hang);
        positions.put("Lift", Lift);
        positions.put("Init", PInit);
        positions.put("Grab", PGrab);
        positions.put("gPlace", 200);
        positions.put("gPlaceUP", 270);
        positions.put("PosTestPivot", PosTestPivot);

        kPs.put("Normal", kP);
        kPs.put("Extended", extendedKp);
    }

    public void update() {

        curLeft = leftPivot.getCurrentPosition();
        //curRight = rightPivot.getCurrentPosition();
        //checkReset();
        if (pos != lta) {
            profile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(pos, 0), new TrapezoidProfile.State(curLeft, aVelocity));
            fullTimer.reset();
        }

        indexedPosition = profile.calculate(fullTimer.seconds()).position;

        pidController.setSetPoint(indexedPosition);

        power = pidController.calculate(curLeft) + (k * Math.cos(pos));

        if (!RobotHardware.inThresh(power, lastPower, 0.001)) {
            applyPower(power);
            lastPower = power;
        }


        lta = pos;

        aVelocity = (curLeft-lAngle)/velTimer.seconds();
        lAngle = curLeft;
        velTimer.reset();
    }

    public void applyPower(double power) {
        leftPivot.setPower(power);
        rightPivot.setPower(power);
    }

    public void setPos(String pos) {
        this.pos = positions.get(pos);
    }

    public void setkP(String kP) {
        pidController.setP(kPs.get(kP));
    }

    public int getTarget() {
        return pos;
    }

    public int getCurrent() {
        return leftPivot.getCurrentPosition();
    }

    public double getPower() {
        return power;
    }

    public double getError() {
        return pos - leftPivot.getCurrentPosition();
    }

}
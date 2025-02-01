package OpModes;

import static config.localization.Limelight.fiducialResults;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

import config.RobotConstants;
import config.localization.KalmanFuse;
import config.localization.Limelight;
import config.subsystems.Pivot;
import config.subsystems.Extension;
import config.subsystems.Wrist;

import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Syntax Terror TeleInit", group = "!TeleOp")
public class TeleOpInit extends OpMode {
    private Follower follower;
    private KalmanFuse kalmanFuse;
    private Pose startPose = new Pose(0,0,0);

    private Extension extension;
    private Pivot pivot;
    private Wrist wrist;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotorEx rightExtension;
    private DcMotorEx leftExtension;
    private DcMotorEx rightPivot;
    private DcMotorEx leftPivot;
    private TouchSensor extLimit;

    private Limelight3A limelight;
    private Limelight LimeInit;

    private boolean isHalfSpeed = false;
    private boolean lastRightBumperState = false;
    private boolean lastLeftBumperState = false;
    private int currentSequence = 0;
    private static final int TOTAL_SEQUENCES = 7;

    private int positions;

    private Servo bicepLeft;
    private Servo bicepRight;
    private Servo forearm;
    private Servo rotation;
    private Servo clawServo;

    private Timer initTimer;
    private boolean servoPositionsEnabled = false;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        rightExtension = hardwareMap.get(DcMotorEx.class, "rightExtension");
        leftExtension = hardwareMap.get(DcMotorEx.class, "leftExtension");
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightPivot");
        leftPivot = hardwareMap.get(DcMotorEx.class, "leftPivot");
        extLimit = hardwareMap.get(TouchSensor.class, "extLimit");

        bicepLeft = hardwareMap.get(Servo.class, "bicepLeft");
        bicepRight = hardwareMap.get(Servo.class, "bicepRight");
        rotation = hardwareMap.get(Servo.class, "rotation");
        forearm = hardwareMap.get(Servo.class, "forearm");
        clawServo = hardwareMap.get(Servo.class, "clawServo");

        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kalmanFuse = new KalmanFuse();
        kalmanFuse.KalmanInit();
        LimeInit = new Limelight();
        LimeInit.LimelightInit(limelight, follower, startPose);

        initTimer = new Timer();

        extension = new Extension(leftExtension, rightExtension, extLimit);
        pivot = new Pivot(hardwareMap, rightPivot, leftPivot);
        wrist = new Wrist(bicepLeft, bicepRight, forearm, rotation);

        follower.startTeleopDrive();
        follower.setMaxPower(1);
        initTimer.resetTimer();
    }

    @Override
    public void init_loop() {
        if(initTimer.getElapsedTimeSeconds() > 1) {
            pivot.setkP("Normal");
            pivot.setPos("Start");
        }
        if(initTimer.getElapsedTimeSeconds() > 2) {
            setPositions(0);
            setPositionsUpdate();
        }
        wrist.update();
        pivot.update();

        if (initTimer.getElapsedTimeSeconds() > 4) {
            telemetry.addData("bicepLeft", bicepLeft.getPosition());
            telemetry.addData("bicepRight", bicepRight.getPosition());
            telemetry.addData("forearm", forearm.getPosition());
            telemetry.addData("leftPivot", leftPivot.getCurrentPosition());
            telemetry.addData("rightPivot", rightPivot.getCurrentPosition());
            telemetry.addData("Init", "Finished");
            telemetry.update();
        }
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        extension.update();
        pivot.update();
        wrist.update();

        // Sequence control
        if (gamepad1.right_bumper && !lastRightBumperState) {
            currentSequence = (currentSequence + 1) % TOTAL_SEQUENCES;
            setPositions(currentSequence);
            servoPositionsEnabled = (currentSequence == 1 || currentSequence == 3);
        }
        lastRightBumperState = gamepad1.right_bumper;

        if (gamepad1.left_bumper && !lastLeftBumperState) {
            currentSequence = (currentSequence - 1 + TOTAL_SEQUENCES) % TOTAL_SEQUENCES;
            setPositions(currentSequence);
            servoPositionsEnabled = (currentSequence == 1 || currentSequence == 3);
        }
        lastLeftBumperState = gamepad1.left_bumper;

        // Servo position control when in appropriate sequence
        if (servoPositionsEnabled) {
            if (gamepad1.dpad_left) wrist.setRotationPos(0);
            if (gamepad1.dpad_up) wrist.setRotationPos(1);
            if (gamepad1.dpad_right) wrist.setRotationPos(2);
            if (gamepad1.dpad_down) wrist.setRotationPos(3);
        }

        // Speed control
        if (gamepad1.right_trigger > 0.5 && !isHalfSpeed) {
            isHalfSpeed = true;
            follower.setMaxPower(0.3);
        } else if (gamepad1.right_trigger <= 0.5 && isHalfSpeed) {
            isHalfSpeed = false;
            follower.setMaxPower(1.0);
        }

        setPositionsUpdate();

        // Telemetry
        telemetry.addData("Current Sequence", currentSequence);
        telemetry.addData("Next Sequence", (currentSequence + 1) % TOTAL_SEQUENCES);
        telemetry.addData("Previous Sequence", (currentSequence - 1 + TOTAL_SEQUENCES) % TOTAL_SEQUENCES);
        telemetry.addData("Speed Mode", isHalfSpeed ? "Half" : "Full");
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("headingDeg", follower.getPose().getHeading());
        telemetry.addData("bicepRight", bicepRight.getPosition());
        telemetry.addData("bicepLeft", bicepLeft.getPosition());
        telemetry.addData("clawServo", clawServo.getPosition());
        telemetry.addData("rightExtension", rightExtension.getCurrentPosition());
        telemetry.addData("leftExtension", leftExtension.getCurrentPosition());
        telemetry.addData("rightPivot", rightPivot.getCurrentPosition());
        telemetry.addData("leftPivot", leftPivot.getCurrentPosition());
        telemetry.addData("Apriltags", fiducialResults);
        telemetry.update();

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);
        follower.update();
    }

    public void setPositions(int pos) {
        positions = pos;
        setPositionsUpdate();
    }

    public void setPositionsUpdate() {
        switch (positions) {
            case 0: // Initial position
                pivot.setkP("Normal");
                pivot.setPos("Init");
                wrist.setForearmPos("Init");
                wrist.setBicepPos("Init");
                wrist.setRotationPos(0);
                clawServo.setPosition(RobotConstants.closeClaw);
                break;
            case 1: // Under bars for grab (claw open)
                pivot.setkP("Normal");
                pivot.setPos("Grab");
                wrist.setForearmPos("Grab");
                wrist.setBicepPos("gUP");
                extension.setPos("Idle");
                clawServo.setPosition(RobotConstants.openClaw);
                break;
            case 2: // Grab from floor
                pivot.setkP("Normal");
                pivot.setPos("Grab");
                wrist.setForearmPos("Grab");
                wrist.setBicepPos("Grab");
                extension.setPos("Idle");
                // Close claw after delay or position confirmation
                clawServo.setPosition(RobotConstants.closeClaw);
                break;
            case 3: // Under bars (claw closed)
                pivot.setkP("Normal");
                pivot.setPos("Grab");
                wrist.setForearmPos("Grab");
                wrist.setBicepPos("gUP");
                extension.setPos("Idle");
                clawServo.setPosition(RobotConstants.closeClaw);
                break;
            case 4: // Place position
                pivot.setkP("Normal");
                pivot.setPos("Start");
                wrist.setForearmPos("Place");
                wrist.setBicepPos("Place");
                extension.setPos("Place");
                clawServo.setPosition(RobotConstants.closeClaw);
                break;
            case 5: // Drop at place position
                pivot.setkP("Normal");
                pivot.setPos("Start");
                wrist.setForearmPos("Place");
                wrist.setBicepPos("Place");
                extension.setPos("Place");
                clawServo.setPosition(RobotConstants.openClaw);
                break;
            case 6: // Grab from wall
                pivot.setkP("Normal");
                pivot.setPos("gPlace");
                wrist.setForearmPos("gPlace");
                wrist.setBicepPos("gPlace");
                wrist.setRotationPos(0);
                extension.setPos("Idle");
                break;
        }
    }
}
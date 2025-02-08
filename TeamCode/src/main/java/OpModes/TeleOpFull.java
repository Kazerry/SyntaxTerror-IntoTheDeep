package OpModes;

import static java.lang.Math.abs;
import static autoModes.OneParkAutoBlue.parkPose;
import static autoModes.OneParkAutoBlue.parkTimer;
import static config.localization.Limelight.fiducialResults;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

import config.RobotHardware;
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

@TeleOp(name = "Into The Deep TeleOp", group = "!TeleOp")
public class TeleOpFull extends OpMode {
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
    private boolean lastXState = false;
    private boolean initboo = false;
    private int currentSequence = 0;
    private int currentSequenceSet = 0; // 0 = Intake, 1 = Sample, 2 = Specimen

    private boolean wasUnderBarsOpen = false;

    private Timer sequenceTimer, posTimer;
    private int grabSequenceState = 0;

    private static final int INTAKE_TOTAL_SEQUENCES = 6; // Updated to 6 cases
    private static final int SAMPLE_TOTAL_SEQUENCES = 3;
    private static final int SPECIMEN_TOTAL_SEQUENCES = 5;

    private static final String[] INTAKE_SEQUENCE_NAMES = {
            "Idle",
            "Under Bars (Open Claw)",
            "Floor Initial Position",
            "Close Claw",
            "Move To Intermediate",
            "Move To Final"
    };


    private static final String[] SAMPLE_SEQUENCE_NAMES = {
            "Idle",
            "Basket Close",
            "Basket Open"
    };

    private static final String[] SPECIMEN_SEQUENCE_NAMES = {
            "Idle",
            "Grab from Wall",
            "Grab from Wall Closed",
            "Place Position",
            "Place Position Open"
    };

    private int positions;

    private Servo bicepLeft;
    private Servo bicepRight;
    private Servo forearm;
    private Servo rotation;
    private Servo clawServo;

    private Timer initTimer, specTimer;
    private boolean servoPositionsEnabled = false;

    private int savedRotationPos = 0;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        if (parkTimer.getElapsedTimeSeconds() < 40) {
            startPose = parkPose;
        } else {
            startPose = new Pose(0,0,0);
        }
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

        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kalmanFuse = new KalmanFuse();
        kalmanFuse.KalmanInit();
        LimeInit = new Limelight();
        LimeInit.LimelightInit(limelight, follower, startPose);

        initTimer = new Timer();
        posTimer = new Timer();
        specTimer = new Timer();

        extension = new Extension(leftExtension, rightExtension, extLimit);
        pivot = new Pivot(hardwareMap, rightPivot, leftPivot);
        wrist = new Wrist(bicepLeft, bicepRight, forearm, rotation);

        wrist.setRotationPos(0);

        follower.startTeleopDrive();
        follower.setMaxPower(1);
        initTimer.resetTimer();
        sequenceTimer = new Timer();

        wrist.setForearmPos("Init");
        wrist.setBicepPos("Init");
        pivot.setkP("Normal");
        pivot.setPos("Init");
    }

    @Override
    public void init_loop() {
        if(initTimer.getElapsedTimeSeconds() >= 2) {
            setPositions(0);
            setPositionsUpdate();
        }
        if(initTimer.getElapsedTimeSeconds() < 2) {
            wrist.setForearmPos("Init");
            wrist.setBicepPos("Init");
            pivot.setkP("Normal");
            pivot.setPos("Init");
        }
        wrist.update();
        pivot.update();

        initboo = true;
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

        initboo = false;

        // Switch between sequence sets
        if (gamepad1.x && !lastXState) {
            currentSequenceSet = (currentSequenceSet + 1) % 3;
            currentSequence = 0; // Reset to first position when switching
            setPositions(currentSequence);
        }
        lastXState = gamepad1.x;

        int totalSequences = getCurrentTotalSequences();

        // Sequence control with state reset
        if (gamepad1.right_bumper && !lastRightBumperState) {
            currentSequence = (currentSequence + 1) % totalSequences;
            if (currentSequenceSet == 0 && currentSequence == 2) {
                grabSequenceState = 0; // Reset state when entering sequence 2
                sequenceTimer.resetTimer();
            }
            setPositions(currentSequence);
            servoPositionsEnabled = (currentSequence == 1 || currentSequence == 3);
        }
        lastRightBumperState = gamepad1.right_bumper;

        if (gamepad1.left_bumper && !lastLeftBumperState) {
            currentSequence = (currentSequence - 1 + totalSequences) % totalSequences;
            if (currentSequenceSet == 0 && currentSequence == 2) {
                grabSequenceState = 0; // Reset state when entering sequence 2
                sequenceTimer.resetTimer();
            }
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

        // Get current sequence names based on current sequence set
        String[] currentSequenceNames = getCurrentSequenceNames();

        // Telemetry updates
        telemetry.addData("Sequence Set",
                currentSequenceSet == 0 ? "Intake" :
                        currentSequenceSet == 1 ? "Sample" : "Specimen");
        telemetry.addData("Current Sequence",
                String.format("%d: %s", currentSequence, currentSequenceNames[currentSequence]));
        telemetry.addData("Next Sequence",
                String.format("%d: %s",
                        (currentSequence + 1) % totalSequences,
                        currentSequenceNames[(currentSequence + 1) % totalSequences]));
        telemetry.addData("Previous Sequence",
                String.format("%d: %s",
                        (currentSequence - 1 + totalSequences) % totalSequences,
                        currentSequenceNames[(currentSequence - 1 + totalSequences) % totalSequences]));

        telemetry.addData("Speed Mode", isHalfSpeed ? "Half" : "Full");
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("headingDeg", follower.getPose().getHeading());
        telemetry.addData("Grab Sequence State", grabSequenceState);
        telemetry.addData("Sequence Timer", sequenceTimer.getElapsedTimeSeconds());
        telemetry.addData("Extension Position", extension.getCurrentPosition());
        telemetry.update();

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);
        follower.update();
    }

    private int getCurrentTotalSequences() {
        switch (currentSequenceSet) {
            case 0: return INTAKE_TOTAL_SEQUENCES;
            case 1: return SAMPLE_TOTAL_SEQUENCES;
            case 2: return SPECIMEN_TOTAL_SEQUENCES;
            default: return 0;
        }
    }

    private String[] getCurrentSequenceNames() {
        switch (currentSequenceSet) {
            case 0: return INTAKE_SEQUENCE_NAMES;
            case 1: return SAMPLE_SEQUENCE_NAMES;
            case 2: return SPECIMEN_SEQUENCE_NAMES;
            default: return new String[]{};
        }
    }

    public void setPositions(int pos) {
        positions = pos;
        setPositionsUpdate();
    }

    public void setPositionsUpdate() {
        switch (currentSequenceSet) {
            case 0: setIntakePositions(); break;
            case 1: setSamplePositions(); break;
            case 2: setSpecimenPositions(); break;
        }
    }

    private void setIntakePositions() {
        if (positions == 1) {
            follower.setMaxPower(0.3);
            isHalfSpeed = true;
        } else if (isHalfSpeed && gamepad1.right_trigger <= 0.5) {
            follower.setMaxPower(1.0);
            isHalfSpeed = false;
        }

        switch (positions) {
            case 0: // Idle
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("Init");
                wrist.setRotationPos(0);
                extension.setPos("Idle");
                clawServo.setPosition(RobotHardware.closeClaw);
                if((posTimer.getElapsedTimeSeconds() > 1.5 || initboo)){
                    wrist.setForearmPos("Init");
                    wrist.setBicepPos("Init");
                }
                break;

            case 1: // Under bars (open claw)
                wasUnderBarsOpen = true;
                pivot.setkP("Normal");
                pivot.setPos("Grab");
                wrist.setForearmPos("Grab");
                wrist.setBicepPos("Grab");
                extension.setPos("Idle");
                clawServo.setPosition(RobotHardware.openClaw);
                savedRotationPos = wrist.getCurrentRotationPos();
                break;

            case 2: // Floor Initial Position
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("Grab");
                wrist.setForearmPos("Intake");
                wrist.setBicepPos("Intake");
                wrist.setRotationPos(savedRotationPos);
                break;

            case 3: // Close Claw
                clawServo.setPosition(RobotHardware.closeClaw);
                break;

            case 4: // Move To Intermediate
                wrist.setBicepPos("Grab");
                wrist.setForearmPos("Grab");
                break;

            case 5: // Move To Final
                posTimer.resetTimer();
                extension.setPos("Idle");
                wrist.setBicepPos("Middle");
                wrist.setForearmPos("Middle");
                wrist.setRotationPos(0);
                break;
        }
    }

    private void setSamplePositions() {
        switch (positions) {
            case 0:
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("Init");
                wrist.setForearmPos("Init");
                wrist.setBicepPos("Init");
                wrist.setRotationPos(0);
                extension.setPos("Idle");
                clawServo.setPosition(RobotHardware.closeClaw);
                break;
            case 1:
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("Basket");
                wrist.setForearmPos("Basket");
                wrist.setBicepPos("Basket");
                extension.setPos("Basket");
                clawServo.setPosition(RobotHardware.closeClaw);
                break;
            case 2:
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("Basket");
                wrist.setForearmPos("Basket");
                wrist.setBicepPos("Basket");
                extension.setPos("Basket");
                clawServo.setPosition(RobotHardware.openClaw);
                break;
        }
    }

    private void setSpecimenPositions() {
        if (positions == 1) {
            follower.setMaxPower(0.3);
            isHalfSpeed = true;
        } else if (isHalfSpeed && gamepad1.right_trigger <= 0.5) {
            follower.setMaxPower(1.0);
            isHalfSpeed = false;
        }

        switch (positions) {
            case 0:
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("Init");
                wrist.setForearmPos("Init");
                wrist.setBicepPos("Init");
                wrist.setRotationPos(0);
                extension.setPos("Idle");
                clawServo.setPosition(RobotHardware.closeClaw);
                break;
            case 1:
                specTimer.resetTimer();
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("SpecGrab");
                wrist.setForearmPos("SpecGrab");
                wrist.setBicepPos("SpecGrab");
                wrist.setRotationPos(0);
                extension.setPos("Idle");
                clawServo.setPosition(RobotHardware.openClaw);
                break;
            case 2:
                specTimer.resetTimer();
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("SpecGrab");
                wrist.setForearmPos("SpecGrab");
                wrist.setBicepPos("SpecGrab");
                wrist.setRotationPos(0);
                extension.setPos("Idle");
                clawServo.setPosition(RobotHardware.closeClaw);
                break;
            case 3:
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("Place");
                clawServo.setPosition(RobotHardware.closeClaw);
                if(specTimer.getElapsedTimeSeconds() > 1.5){
                    wrist.setForearmPos("SpecPlace");
                    wrist.setBicepPos("SpecPlace");
                    extension.setPos("Place");
                    clawServo.setPosition(RobotHardware.closeClaw);
                }
                if(specTimer.getElapsedTimeSeconds() > 2){
                    wrist.setRotationPos(4); //180 degrees
                    clawServo.setPosition(RobotHardware.closeClaw);
                }
                clawServo.setPosition(RobotHardware.closeClaw);
                break;
            case 4:
                specTimer.resetTimer();
                wasUnderBarsOpen = false;
                pivot.setkP("Normal");
                pivot.setPos("Place");
                wrist.setForearmPos("SpecPlace");
                wrist.setBicepPos("SpecPlace");
                extension.setPos("Place");
                clawServo.setPosition(RobotHardware.openClaw);
                wrist.setRotationPos(4); //180 degrees
                break;
        }
    }
}
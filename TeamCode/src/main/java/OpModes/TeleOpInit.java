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
    /**
     * Make sure to startPose with the actual starting pose
     **/
    private Pose startPose = new Pose(0,0,0);

    //private ClawSubsystem claw;
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
    private boolean lastToggleState = false;

    private int positions;

    private Servo bicepLeft;
    private Servo bicepRight;
    private Servo forearm;
    private Servo rotation;
    private Servo clawServo;

    private Timer initTimer;

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

        extension = new Extension(leftExtension, rightExtension, extLimit); //River Extension
        pivot = new Pivot(hardwareMap, rightPivot, leftPivot); //River Pivot
        wrist = new Wrist(bicepLeft, bicepRight, forearm, rotation); //River Wrist

        follower.startTeleopDrive();
        follower.setMaxPower(1);
        initTimer.resetTimer();
    }

    @Override
    public void init_loop(){
        //Initialization
        if(initTimer.getElapsedTimeSeconds() > 1){
            pivot.setkP("Normal");
            pivot.setPos("Start");
        }
        if(initTimer.getElapsedTimeSeconds() > 2) {
            setPositions(0);
            setPositionsUpdate();
        }
        wrist.update();
        pivot.update();
        // After 4 Seconds, Robot Initialization is complete
        if (initTimer.getElapsedTimeSeconds() > 4) {
            telemetry.addData("bicepLeft",bicepLeft.getPosition());
            telemetry.addData("bicepRight",bicepRight.getPosition());
            telemetry.addData("forearm",forearm.getPosition());
            telemetry.addData("leftPivot",leftPivot.getCurrentPosition());
            telemetry.addData("rightPivot",rightPivot.getCurrentPosition());
            telemetry.addData("Init", "Finished");
            telemetry.update();
        }
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }


    @Override
    public void loop() {
        // Update subsystems
        extension.update();
        pivot.update();
        wrist.update();

        setPositionsUpdate();

        //Kalman filtering and Pose fusing between PedroPathing and LimeLight
        //kalmanFuse.updateLocalization(follower.getPose(), LimeInit);
        //Pose tempPose = kalmanFuse.getFusedPose();
        //follower.setPose(tempPose);

        // Manual controls for claw behavior
        if (gamepad1.dpad_up) {
            clawServo.setPosition(RobotConstants.openClaw);
        }
        if (gamepad1.dpad_down) {
            clawServo.setPosition(RobotConstants.closeClaw);
        }
        if (gamepad1.dpad_left) { //testing for grabbing from side
            setPositions(-1);
        }

        if (gamepad1.x) { //Placing
            setPositions(1);
        }
        if (gamepad1.y) { //Drop to place
            setPositions(2);
        }
        if (gamepad1.left_bumper) { //Grab from wall
            setPositions(3);
        }
        if (gamepad1.right_trigger > 0.5) {
            wrist.setRotationPos(0); //0
        }
        if (gamepad1.left_trigger > 0.5) {
            wrist.setRotationPos(1); //0.65
        }
        if (gamepad1.a) { //Grab from floor
            setPositions(4);
        }
        if (gamepad1.b) { //get under bars for grab
            setPositions(5);
        }
        if(gamepad1.left_stick_button){
        setPositions(0); //Init
        }

        if(gamepad1.right_bumper && !lastToggleState) { // If button pressed and wasn't pressed before
            isHalfSpeed = !isHalfSpeed;        // Toggle the speed state
            follower.setMaxPower(isHalfSpeed ? 0.3 : 1.0); // Set power based on speed state
        }
        lastToggleState = gamepad1.right_bumper; // Update the previous button state

        if(!isHalfSpeed){
            telemetry.addLine("Robot Speed Full");
        } else {
            telemetry.addLine("Robot Speed Half");
        }

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
        telemetry.addData("Apriltags",fiducialResults);
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

    public void setPositionsUpdate()  {
        switch (positions) {
            case -1: // off wall
                wrist.setForearmPos("gPlaceUP");
                pivot.setPos("gPlaceUP");
                break;
            case 0: // Init
                pivot.setkP("Normal");
                pivot.setPos("Init");
                wrist.setForearmPos("Init");
                wrist.setBicepPos("Init");
                wrist.setRotationPos(0);
                clawServo.setPosition(RobotConstants.closeClaw);
                break;
            case 1: // Place
                pivot.setkP("Normal");
                pivot.setPos("Start");
                wrist.setForearmPos("Place");
                wrist.setBicepPos("Place");
                extension.setPos("Place");

                break;
            case 2: // Drop to Place
                pivot.setkP("Normal");
                pivot.setPos("Start");
                wrist.setForearmPos("downPlace");
                wrist.setBicepPos("downPlace");
                extension.setPos("downPlace");
                break;
            case 3: // Grab from wall
                pivot.setkP("Normal");
                pivot.setPos("gPlace");
                wrist.setForearmPos("gPlace");
                wrist.setBicepPos("gPlace");
                wrist.setRotationPos(0);
                extension.setPos("Idle");
                break;
            case 4: // Grab from floor
                pivot.setkP("Normal");
                pivot.setPos("Grab");
                wrist.setForearmPos("Grab");
                wrist.setBicepPos("Grab");
                extension.setPos("Idle");
                break;
            case 5: // Under bars for grab
                pivot.setkP("Normal");
                pivot.setPos("Grab");
                wrist.setForearmPos("Grab");
                wrist.setBicepPos("gUP");
                extension.setPos("Idle");
                break;
            case 6: // Idle
                pivot.setPos("Idle");
                pivot.setkP("Normal");
                extension.setPos("Idle");
                wrist.setForearmPos("Idle");
                wrist.setBicepPos("Idle");
                break;
        }
    }
}

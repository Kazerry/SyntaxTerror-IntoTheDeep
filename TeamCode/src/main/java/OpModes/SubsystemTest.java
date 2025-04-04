package OpModes;

import static config.localization.Limelight.LX;
import static config.localization.Limelight.LY;
import static config.localization.Limelight.fiducialResults;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.acmerobotics.dashboard.config.Config;

import config.RobotHardware;
import config.localization.KalmanFuse;
import config.localization.Limelight;
import config.subsystems.Pivot;
import config.subsystems.Extension;
import config.subsystems.Wrist;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@TeleOp(name = "SubsystemTest", group = "Test")
public class SubsystemTest extends OpMode {
    private Follower follower;
    private KalmanFuse kalmanFuse;
    /**
     * Make sure to startPose with the actual starting pose
     **/
    private final Pose startPose = new Pose(0,0,0);

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

    public static double subP, subI, subD, subF;
    public static double spP, spI, spD, spF;
    public static double testPos = 0.6;

    private Servo bicepLeft;
    private Servo bicepRight;
    private Servo forearm;
    private Servo rotation;
    private Servo clawServo;

    @Override
    public void init() {

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        /*if (parkTimer.getElapsedTimeSeconds() < 40) {
            startPose = parkPose;
        } else {
            startPose = new Pose(0,0,0);
        }*/
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
        //ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

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

        //claw = new ClawSubsystem(clawServo, colorSensor);
        extension = new Extension(leftExtension, rightExtension, extLimit); //River Extension
        pivot = new Pivot(hardwareMap, rightPivot, leftPivot); //River Pivot
        wrist = new Wrist(bicepLeft, bicepRight, forearm, rotation); //River Wrist

        follower.startTeleopDrive();
        follower.setMaxPower(1);
    }

    @Override

    public void init_loop(){
        //Init movements if you want them
        wrist.setBicepPos("Middle");
        wrist.setForearmPos("Middle");
        wrist.setRotationPos(0);
        wrist.update();
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        follower.startTeleopDrive();
    }


    @Override
    public void loop() {
        // Update subsystems
        //claw.manageClaw();
        extension.update();
        pivot.update();
        wrist.update();


        //Kalman filtering and Pose fusing between PedroPathing and LimeLight
        //kalmanFuse.updateLocalization(follower.getPose(), LimeInit);
        //Pose tempPose = kalmanFuse.getFusedPose();
        //follower.setPose(tempPose);

        // Manual controls for claw behavior
       if (gamepad1.dpad_up) {
            clawServo.setPosition(RobotHardware.openClaw);
        }
        if (gamepad1.dpad_down) {
            clawServo.setPosition(RobotHardware.closeClaw);
        }
        if (gamepad1.dpad_left) { //testing for grabbing from side
            wrist.setForearmPos("gPlaceUP");
            pivot.setPos("Start");
        }

        if (gamepad1.x) { //Placing
            pivot.setkP("Normal");
            pivot.setPos("Start");
            wrist.setForearmPos("Place");
            wrist.setBicepPos("Place");
            extension.setPos("Place");
        }
        if (gamepad1.y) { //Drop to place
            pivot.setkP("Normal");
            pivot.setPos("Start");
            wrist.setForearmPos("downPlace");
            wrist.setBicepPos("downPlace");
            extension.setPos("downPlace");
            clawServo.setPosition(RobotHardware.closeClaw);
        }
        if (gamepad1.left_bumper) { //Grab from wall
            pivot.setkP("Normal");
            pivot.setPos("gPlace");
            wrist.setForearmPos("gPlace");
            wrist.setBicepPos("gPlace");
            wrist.setRotationPos(0);
            extension.setPos("Idle");
        }
        if (gamepad1.right_trigger > 0.5) {
            wrist.setRotationPos(0); //0
        }
        if (gamepad1.left_trigger > 0.5) {
            wrist.setRotationPos(1); //0.65
        }
        if (gamepad1.a) { //Grab from floor
            pivot.setkP("Normal");
            pivot.setPos("Grab");
            wrist.setForearmPos("Grab");
            wrist.setBicepPos("Grab");
            wrist.setRotationPos(0);
            extension.setPos("Idle");
        }
        if (gamepad1.b) { //get under bars for grab
            pivot.setkP("Normal");
            pivot.setPos("Grab");
            wrist.setForearmPos("gUP");
            wrist.setBicepPos("Grab");
            wrist.setRotationPos(0);
            extension.setPos("Idle");
        }

        if(gamepad1.right_bumper && !lastToggleState) { // If button pressed and wasn't pressed before
            isHalfSpeed = !isHalfSpeed;        // Toggle the speed state
            follower.setMaxPower(isHalfSpeed ? 0.5 : 1.0); // Set power based on speed state
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
        telemetry.addData("LLx", LX);
        telemetry.addData("LLy", LY);
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
    public void setPositions(int pos)  {
            switch (pos) {
                case 0: // Idle
                    pivot.setPos("Idle");
                    pivot.setkP("Normal");
                    extension.setPos("Idle");
                    wrist.setForearmPos("Idle");
                    wrist.setBicepPos("Idle");
                    break;
                case 1: // Sample Intake: Down, Unextended
                    pivot.setPos("Down");
                    pivot.setkP("Normal");
                    extension.setPos("Idle");
                    wrist.setForearmPos("Idle");
                    wrist.setBicepPos("Idle");

                    break;
                case 2: // Sample Extend
                    pivot.setPos("Down");
                    pivot.setkP("Extended");
                    extension.setPos("Intake");
                    wrist.setForearmPos("Idle");
                    wrist.setBicepPos("Idle");
                    break;
                case 3: // Flip Down
                    pivot.setPos("Down");
                    pivot.setkP("Extended");
                    extension.setPos("Intake");
                    wrist.setForearmPos("Intake");
                    wrist.setBicepPos("Intake");
                    break;
                case 4: // Flip Up
                    pivot.setPos("Down");
                    pivot.setkP("Extended");
                    extension.setPos("Intake");
                    wrist.setForearmPos("Idle");
                    wrist.setBicepPos("Idle");
                    break;
                case 5: // Pullout
                    pivot.setPos("Down");
                    pivot.setkP("Normal");
                    extension.setPos("Idle");
                    wrist.setForearmPos("Idle");
                    wrist.setBicepPos("Idle");
                    wrist.setRotationPos(0);
                    break;
                case 6: // Idle
                    pivot.setPos("Idle");
                    pivot.setkP("Normal");
                    extension.setPos("Idle");
                    wrist.setBicepPos("Idle");
                    wrist.setForearmPos("Idle");
                    break;
                case 7: // High Basket
                    pivot.setPos("Basket");
                    pivot.setkP("Extended");
                    extension.setPos("Basket");
                    wrist.setBicepPos("Basket");
                    wrist.setForearmPos("Basket");
                    break;
                case 8: // Flip Out
                    pivot.setPos("Basket");
                    pivot.setkP("Extended");
                    extension.setPos("Basket");
                    wrist.setBicepPos("Intake");
                    wrist.setForearmPos("Intake");
                    break;
                case -1: // Hang Pivot Position
                    wrist.setBicepPos("Intake");
                    wrist.setForearmPos("Intake");
                    pivot.setPos("Hang");
                    extension.setPos("Idle");
                    break;
                case -2: // Hang Extend
                    extension.setPos("Hang");
                    break;
                case -3: // Hang Retract
                    extension.setPos("Retract");
                    break;
            }

    }
}

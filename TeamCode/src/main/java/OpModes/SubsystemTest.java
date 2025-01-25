package OpModes;

import static config.localization.Limelight.LX;
import static config.localization.Limelight.LY;
import static config.localization.Limelight.fiducialResults;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.acmerobotics.dashboard.config.Config;
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
    public static double flip1pos1 = 0.5;
    public static double flip2pos1 = 0.5;
    public static double flip1pos2 = 0.6;
    public static double flip2pos2 = 0.4;

    private Servo flip1;
    private Servo flip2;
    private Servo bicepLeft;
    private Servo bicepRight;
    private Servo forearm;
    private Servo rotation;

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

        //Servo clawServo = hardwareMap.get(Servo.class, "Cservo");
        flip1 = hardwareMap.get(Servo.class, "flip1");
        flip2 = hardwareMap.get(Servo.class, "flip2");
        bicepLeft = hardwareMap.get(Servo.class, "bicepLeft");
        bicepRight = hardwareMap.get(Servo.class, "bicepRight");
        rotation = hardwareMap.get(Servo.class, "rotation");
        forearm = hardwareMap.get(Servo.class, "forearm");
        //ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

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
        kalmanFuse.updateLocalization(follower.getPose(), LimeInit);
        Pose tempPose = kalmanFuse.getFusedPose();
        //follower.setPose(tempPose);

        // Manual controls for claw behavior
        if (gamepad1.x) {
            extension.setPos("Idle");
        }
        if (gamepad1.y) {
            extension.setPos("Intake");
        }

        if(gamepad1.dpad_up){
            flip1.setPosition(flip1pos1);
            flip2.setPosition(flip2pos1);
        };

        if(gamepad1.dpad_down){
            flip1.setPosition(flip1pos2);
            flip2.setPosition(flip2pos2);
        };

       if(gamepad1.a){
            pivot.setkP("Normal");
            pivot.setPos("Down");
        }
        if(gamepad1.b){
            pivot.setkP("Normal");
            pivot.setPos("Start");
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
        telemetry.addData("Fx", tempPose.getX());
        telemetry.addData("Fy", tempPose.getY());
        telemetry.addData("FHeadingDeg", tempPose.getHeading());
        //telemetry.addData("flip 1 pos", flip1pos1);
        //telemetry.addData("flip 2 pos", flip2pos1);
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
                    wrist.setBicepPos("Idle");
                    wrist.setForearmPos("Idle");
                    break;
                case 1: // Sample Intake: Down, Unextended
                    pivot.setPos("Down");
                    pivot.setkP("Normal");
                    extension.setPos("Idle");
                    wrist.setBicepPos("Idle");
                    wrist.setForearmPos("Idle");

                    break;
                case 2: // Sample Extend
                    pivot.setPos("Down");
                    pivot.setkP("Extended");
                    extension.setPos("Intake");
                    wrist.setBicepPos("Idle");
                    wrist.setForearmPos("Idle");
                    break;
                case 3: // Flip Down
                    pivot.setPos("Down");
                    pivot.setkP("Extended");
                    extension.setPos("Intake");
                    wrist.setBicepPos("Intake");
                    wrist.setForearmPos("Intake");
                    break;
                case 4: // Flip Up
                    pivot.setPos("Down");
                    pivot.setkP("Extended");
                    extension.setPos("Intake");
                    wrist.setBicepPos("Idle");
                    wrist.setForearmPos("Idle");
                    break;
                case 5: // Pullout
                    pivot.setPos("Down");
                    pivot.setkP("Normal");
                    extension.setPos("Idle");
                    wrist.setBicepPos("Idle");
                    wrist.setForearmPos("Idle");
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

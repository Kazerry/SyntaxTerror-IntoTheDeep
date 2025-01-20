package OpModes;

import static config.localization.Limelight.LX;
import static config.localization.Limelight.LY;
import static config.localization.KalmanFuse.rawPedroPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.acmerobotics.dashboard.config.Config;
import config.localization.KalmanFuse;
import config.localization.Limelight;
import config.subsystems.extSubsystem;
import config.subsystems.pivotSubsystem;
import config.subsystems.ClawSubsystem;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

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
    private extSubsystem slides;
    private pivotSubsystem pivot;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotorEx rightExtension;
    private DcMotorEx leftExtension;
    private DcMotorEx rightPivot;
    private DcMotorEx leftPivot;

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

        //Servo clawServo = hardwareMap.get(Servo.class, "Cservo");
        flip1 = hardwareMap.get(Servo.class, "flip1");
        flip2 = hardwareMap.get(Servo.class, "flip2");
        //ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        /*leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/

        kalmanFuse = new KalmanFuse();
        kalmanFuse.KalmanInit();
        rawPedroPose = startPose;
        LimeInit = new Limelight();
        LimeInit.LimelightInit(limelight, follower, startPose);

        //claw = new ClawSubsystem(clawServo, colorSensor);
        slides = new extSubsystem(rightExtension, leftExtension, 0, 0, 0,0,537.7/360.0);
        pivot = new pivotSubsystem(rightPivot,leftPivot,0,0,0,0,537.7/360.0);

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
        slides.update();
        pivot.update();

        //Kalman filtering and Pose fusing between PedroPathing and LimeLight
        kalmanFuse.updateLocalization(follower.getPose(), LimeInit);
        Pose tempPose = kalmanFuse.getFusedPose();
        follower.setPose(tempPose);

        // Manual controls for claw behavior
        /*if (gamepad1.x) {
            claw.closeClaw();
        }
        if (gamepad1.y) {
            claw.openClaw();
        }*/

        if(gamepad1.dpad_up){
            flip1.setPosition(flip1pos1);
            flip2.setPosition(flip2pos1);
        };

        if(gamepad1.dpad_down){
            flip1.setPosition(flip1pos2);
            flip2.setPosition(flip2pos2);
        };

        if(gamepad1.a){
            pivot.moveToBottom();
        }
        if(gamepad1.b){
            pivot.moveToTop();
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
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("LLx", LX);
        telemetry.addData("LLy", LY);
        telemetry.addData("Fx", tempPose.getX());
        telemetry.addData("Fy", tempPose.getY());
        telemetry.addData("FHeading", tempPose.getHeading());
        //telemetry.addData("flip 1 pos", flip1pos1);
        //telemetry.addData("flip 2 pos", flip2pos1);
        telemetry.addData("avgExtensionPos", slides.getCurrentPosition());
        telemetry.addData("rightExtension", rightExtension.getCurrentPosition());
        telemetry.addData("leftExtension", leftExtension.getCurrentPosition());
        telemetry.addData("avgPivotPos", pivot.getCurrentPosition());
        telemetry.addData("rightPivot", rightPivot.getCurrentPosition());
        telemetry.addData("leftPivot", leftPivot.getCurrentPosition());
        telemetry.update();

        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);
        follower.update();
    }
}

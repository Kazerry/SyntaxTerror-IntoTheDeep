package OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;

import config.localization.KalmanFuse;
import config.localization.Limelight;
import config.subsystems.SlideSubsystem;
import config.subsystems.ClawSubsystem;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "Into The Deep TeleOp", group = "!TeleOp")
public class TeleOpFull extends OpMode {
    private Follower follower;
    private KalmanFuse kalmanFuse;
    /**
     * Make sure to startPose with the actual starting pose
     **/
    private final Pose startPose = new Pose(0,0,0);

    private ClawSubsystem claw;
    private SlideSubsystem slides;
    //private SpoolSubsystem spool;

    private DcMotorEx leftFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightFront;
    private DcMotorEx rightRear;
    private DcMotorEx slide1;
    private DcMotorEx slide2;
    //private DcMotorEx spool1;
    //private DcMotorEx spool2;

    private Limelight3A limelight;
    private Limelight LimeInit;

    private boolean isHalfSpeed = false;
    private boolean lastToggleState = false;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        slide1 = hardwareMap.get(DcMotorEx.class, "slide1");
        slide2 = hardwareMap.get(DcMotorEx.class, "slide2");
        //spool1 = hardwareMap.get(DcMotorEx.class, "spool1");
        //spool2 = hardwareMap.get(DcMotorEx.class, "spool2");

        Servo clawServo = hardwareMap.get(Servo.class, "Cservo");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kalmanFuse = new KalmanFuse();
        kalmanFuse.KalmanInit();
        LimeInit = new Limelight();
        LimeInit.LimelightInit(limelight);

        claw = new ClawSubsystem(clawServo, colorSensor);
        slides = new SlideSubsystem(slide1, slide2, 0, 0, 0,0,537.7/360.0);
        //spool = new SpoolSubsystem(spool1,spool2,0,0,0,0,537.7/360.0);

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
        claw.manageClaw();
        slides.update();
        //spool.update();

        //Kalman filtering and Pose fusing between PedroPathing and LimeLight
        kalmanFuse.updateLocalization(follower.getPose(), LimeInit);
        Pose tempPose = kalmanFuse.getFusedPose();
        follower.setPose(tempPose);

        // Manual controls for claw behavior
        if (gamepad1.x) {
            claw.closeClaw();
        }
        if (gamepad1.y) {
            claw.openClaw();
        }

        if(gamepad1.a){
            slides.moveToBottom();
        }
        if(gamepad1.b){
            slides.moveToTop();
        }

        if(gamepad1.right_bumper && !lastToggleState) { // If button pressed and wasn't pressed before
            isHalfSpeed = !isHalfSpeed;        // Toggle the speed state
            follower.setMaxPower(isHalfSpeed ? 0.5 : 1.0); // Set power based on speed state
        }
        lastToggleState = gamepad1.right_bumper; // Update the previous button state

        telemetry.addData("slidePos", slides.getCurrentPosition());
        telemetry.update();



        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);
        follower.update();
    }
}
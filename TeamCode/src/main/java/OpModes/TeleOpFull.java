package OpModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import static autoModes.AutoTest.parkPose;
import static autoModes.AutoTest.parkTimer;
import static config.localization.KalmanFuse.rawPedroPose;

import config.localization.KalmanFuse;
import config.localization.Limelight;
import config.subsystems.archiveSubsystems.extSubsystem;
import config.subsystems.archiveSubsystems.pivotSubsystem;
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

    private Pose startPose;

    private ClawSubsystem claw;
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

    private Servo flip1;
    private Servo flip2;

    /**
     * This initializes the drive motors as well as the Follower and motion Vectors.
     */
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

        Servo clawServo = hardwareMap.get(Servo.class, "Cservo");
        flip1 = hardwareMap.get(Servo.class, "flip1");
        flip2 = hardwareMap.get(Servo.class, "flip2");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kalmanFuse = new KalmanFuse();
        kalmanFuse.KalmanInit();
        rawPedroPose = startPose;
        LimeInit = new Limelight();
        LimeInit.LimelightInit(limelight, follower, startPose);

        claw = new ClawSubsystem(clawServo, colorSensor);
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
        claw.manageClaw();
        slides.update();
        pivot.update();

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

        telemetry.addData("extensionPos", slides.getCurrentPosition());
        telemetry.addData("pivotPos", pivot.getCurrentPosition());
        telemetry.update();



        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false);
        follower.update();
    }
}
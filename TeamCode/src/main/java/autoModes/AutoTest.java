package autoModes;


import config.RobotConstants;
import config.subsystems.Pivot;
import config.subsystems.Extension;
import config.subsystems.Wrist;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;

import config.localization.KalmanFuse;
import config.localization.Limelight;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Example Auto Blue", group = "Test")
public class AutoTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    public static Timer parkTimer;
    private int pathState, actionState, caseState;
    private String navigation;
    private KalmanFuse kalmanFuse;
    private Extension extension;
    private Pivot pivot;
    private Wrist wrist;
    private DcMotorEx rightExtension;
    private DcMotorEx leftExtension;
    private DcMotorEx rightPivot;
    private DcMotorEx leftPivot;
    private TouchSensor extLimit;
    private Servo bicepLeft;
    private Servo bicepRight;
    private Servo forearm;
    private Servo rotation;
    private Servo clawServo;


    private Limelight3A limelight;
    private Limelight LimeInit;

    //Start pose
    private Pose startPose = new Pose(7.75, 62.8, 0); //9.3 85.3 is left blue
    public static Pose parkPose;

    /**Robot width and height were set to 19 and 18 respectively
     * when making the path in the PedroPathing Vercel generator
     */
    //Poses for generic field things
    private Pose blueBasket = new Pose(19, 95, Math.toRadians(135));
    private Pose leftSub = new Pose(51.5,96.5,Math.toRadians(270));
    private Pose rightSub = new Pose(60,45,Math.toRadians(90));
    private Pose leftBlueSub = new Pose(36,76,Math.toRadians(0));
    private Pose observationBlue = new Pose(18.4,34.7,Math.toRadians(235));
    private Pose rightBlueSub = new Pose(32,62.8,Math.toRadians(0));
    private Pose firstSpecimen = new Pose(59.7,25,Math.toRadians(0));
    private Pose secondSpecimen = new Pose(59.7,15,Math.toRadians(0));
    private Pose thirdSpecimen = new Pose(59.7,9.3,Math.toRadians(0));
    private Pose firstPlace = new Pose(36,69.5,Math.toRadians(0));
    private Pose secondPlace = new Pose(36,72,Math.toRadians(0));
    private Pose thirdPlace = new Pose(36,74.5,Math.toRadians(0));
    private Pose fourthPlace = new Pose(36,77,Math.toRadians(0));
    private Pose grabPlace = new Pose(11.9,24,Math.toRadians(0));


    private PathChain initialRightSub, rightMove, specimenCurve,
            observationBack1, basketMove, leftSubMove, observationPush1, observationDown1,
    observationPush2, observationBack2, observationDown2, observationPush3, observationDown3,
            curveBack, place1, placeBack1, place2, placeBack2, place3, placeBack3, place4,
            rightMover;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        rightMover = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(grabPlace)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0.5)
                .build();

        //Line1
        rightMove = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(rightBlueSub)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0.5)
                .build();

        //Line2
        specimenCurve = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(rightBlueSub), new Point(26.700, 32.800),
                        new Point(59.700, 32.800)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();

        //Line3
        observationDown1 = follower.pathBuilder()
                .addPath(        new BezierLine(
                        new Point(59.700, 32.800, Point.CARTESIAN),
                        new Point(firstSpecimen)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();

        //Line4
        observationPush1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstSpecimen),
                        new Point(19.700, 25.000, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();

        //Line5
        observationBack1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(19.700, 25.000),
                        new Point(firstSpecimen)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();

        //Line6
        observationDown2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(firstSpecimen),
                        new Point(secondSpecimen)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .setPathEndTimeoutConstraint(0.5)
                .build();
       //Line7
        observationPush2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSpecimen),
                        new Point(19.7, 15)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line8
        observationBack2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(19.7, 15),
                        new Point(secondSpecimen)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line9
        observationDown3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondSpecimen),
                        new Point(thirdSpecimen)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line10
        observationPush3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdSpecimen),
                        new Point(19.7, 9.3)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line11
        curveBack = follower.pathBuilder()
                .addPath(new BezierCurve(
          new Point(19.700, 9.300, Point.CARTESIAN),
          new Point(30.800, 15.100, Point.CARTESIAN),
          new Point(grabPlace)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line12
        place1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPlace),
                        new Point(firstPlace)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line13
        placeBack1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(firstPlace),
                        new Point(grabPlace)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line14
        place2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPlace),
                        new Point(secondPlace)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line15
        placeBack2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(secondPlace),
                        new Point(grabPlace)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line16
        place3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPlace),
                        new Point(thirdPlace)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line17
        placeBack3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(thirdPlace),
                        new Point(grabPlace)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .setPathEndTimeoutConstraint(0.5)
                .build();
        //Line18
        place4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(grabPlace),
                        new Point(fourthPlace)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .setPathEndTimeoutConstraint(0.5)
                .build();
    }

    //Cases for larger paths
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                //Line1
                //close claw
                //clawServo.setPosition(RobotConstants.closeClaw);
                //if(pathTimer.getElapsedTimeSeconds() > 1) {
                    //setCaseState(1); // Place
                follower.followPath(rightMove);
                    //open claw
                    //clawServo.setPosition(RobotConstants.openClaw);
                    //setPathState(11);
                //}
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 3 && follower.atParametricEnd()) {
                    //setCaseState(0); // Rest
                    //Line2
                    follower.followPath(specimenCurve);
                    setPathState(12);
                }
                break;
            case 12:
                if (follower.atParametricEnd()) {
                    //Line3
                follower.followPath(observationDown1);
                setPathState(13);
                }
                break;
            case 13:
                if (follower.atParametricEnd()) {
                    //Line4
                    follower.followPath(observationPush1);
                    setPathState(14);
                }
                break;
            case 14:
                if (follower.atParametricEnd()) {
                    //Line5
                    follower.followPath(observationBack1);
                    //setPathState(15);
                }
                break;
            case 15:
                if (follower.atParametricEnd()) {
                    //Line6
                    follower.followPath(observationDown2);
                    setPathState(16);
                }
                break;
            case 16:
                if (follower.atParametricEnd()) {
                    //Line7
                    follower.followPath(observationPush2);
                    setPathState(17);
                }
                break;
            case 17:
                if (follower.atParametricEnd()) {
                    //Line8
                    follower.followPath(observationBack2);
                    setPathState(18);
                }
                break;
            case 18:
                if (follower.atParametricEnd()) {
                    //Line9
                    follower.followPath(observationDown3);
                    setPathState(19);
                }
                break;
            case 19:
                if (follower.atParametricEnd()) {
                    //Line10
                    follower.followPath(observationPush3);
                    setPathState(20);
                }
                break;
            case 20:
                if (follower.atParametricEnd()) {
                    //Line11
                    // setCaseState(2);
                    follower.followPath(curveBack);
                    //Close claw
                    //clawServo.setPosition(RobotConstants.closeClaw);
                    //We need to grab from observation zone
                    //setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTimeSeconds() > 3 && follower.atParametricEnd()) {
                    //Line12
                    //setCaseState(1);
                    follower.followPath(place1);
                    if (follower.atParametricEnd()) {
                        //open claw
                        //clawServo.setPosition(RobotConstants.openClaw);
                        setPathState(22);
                    }
                }
                break;
            case 22:
                if (follower.atParametricEnd()) {
                    //Line13
                    //setCaseState(2);
                    follower.followPath(placeBack1);
                    if (follower.atParametricEnd()) {
                        //close claw
                        //clawServo.setPosition(RobotConstants.closeClaw);
                        setPathState(23);
                    }
                }
                break;
            case 23:
                if (follower.atParametricEnd()) {
                    //Line14
                    //setCaseState(1);
                    follower.followPath(place2);
                    if (follower.atParametricEnd()) {
                        //open claw
                        //clawServo.setPosition(RobotConstants.openClaw);
                        setPathState(24);
                    }
                }
                break;
            case 24:
                if (follower.atParametricEnd()) {
                    //Line15
                    //setCaseState(2);
                    follower.followPath(placeBack2);
                    if (follower.atParametricEnd()) {
                        //close claw
                        //clawServo.setPosition(RobotConstants.closeClaw);
                        setPathState(25);
                    }
                }
                break;
            case 25:
                if (follower.atParametricEnd()) {
                    //Line16
                    //setActionState(1);
                    follower.followPath(place3);
                    if (follower.atParametricEnd()) {
                        //open claw
                        //clawServo.setPosition(RobotConstants.openClaw);
                        setPathState(26);
                    }
                }
                break;
            case 26:
                if (follower.atParametricEnd()) {
                    //Line17
                    //setActionState(2);
                    follower.followPath(placeBack3);
                    if (follower.atParametricEnd()) {
                        //close claw
                        //clawServo.setPosition(RobotConstants.closeClaw);
                        setPathState(27);
                    }
                }
                break;
            case 27:
                if (follower.atParametricEnd()) {
                    //Line18
                    //setActionState(1);
                    follower.followPath(place4);
                    if (follower.atParametricEnd()) {
                        //open claw
                        //clawServo.setPosition(RobotConstants.openClaw);
                        setPathState(28);
                    }
                }
                break;
        }
    }

    /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    //This can be used for running the claw, lift, etc while a path is being executed
    /*public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                //This should be folded/Idle
                setCaseState(0);
                setActionState(-1);
                break;
            case 1:
                //This should be placing specimens
                //setCaseState();
                setActionState(-1);
                break;
            case 2:
                //This should be getting specimen from observation zone
                setActionState(-1);
                break;
        }
    }*/

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
        autonomousPathUpdate();
    }

    /*public void setActionState(int aState) {
        actionState = aState;
        actionTimer.resetTimer();
        autonomousActionUpdate();
    }*/

    public void setCaseState(int pos) {
        caseState = pos;
        autonomousCaseUpdate();
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop(){
        // These loop the actions and movement of the robot
        follower.update();
        autonomousPathUpdate();
        //autonomousActionUpdate();
        //autonomousCaseUpdate();

        //Kalman filtering and Pose fusing between PedroPathing and LimeLight
        kalmanFuse.updateLocalization(follower.getPose(), LimeInit);
        Pose tempPose = kalmanFuse.getFusedPose();
        //follower.setPose(tempPose);

        //extension.update();
        pivot.update();
        wrist.update();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("pathTimer",pathTimer.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        //Manual stop to save our position if we go over time in order to not incur penalties
        if (opmodeTimer.getElapsedTimeSeconds() > 30) {
            stop();
        }
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        kalmanFuse = new KalmanFuse();
        kalmanFuse.KalmanInit();
        LimeInit = new Limelight();
        LimeInit.LimelightInit(limelight, follower, startPose);

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

        extension = new Extension(leftExtension, rightExtension, extLimit); //River Extension
        pivot = new Pivot(hardwareMap, rightPivot, leftPivot); //River Pivot
        wrist = new Wrist(bicepLeft, bicepRight, forearm, rotation); //River Wrist

        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        parkTimer = new Timer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.setMaxPower(0.8); // Test this
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop(){
        //Initialization
        setCaseState(-2);
        telemetry.addData("bicepLeft",bicepLeft.getPosition());
        telemetry.addData("bicepRight",bicepRight.getPosition());
        telemetry.addData("forearm",forearm.getPosition());
        telemetry.addData("leftPivot",leftPivot.getCurrentPosition());
        telemetry.addData("rightPivot",rightPivot.getCurrentPosition());
        telemetry.addLine("init_loop finished");
        telemetry.update();
        wrist.update();
        pivot.update();
        autonomousCaseUpdate();

        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 3) {
            telemetry.addData("Init", "Finished");
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start(){
        buildPaths();
        opmodeTimer.resetTimer();
        //setCaseState(0);
        setPathState(10);
        //setActionState(0);
    }

    /** Stops robot and saves ending robot position for field centric TeleOp **/
    @Override
    public void stop(){
        parkPose = follower.getPose();
        parkTimer.resetTimer();
    }

    public void autonomousCaseUpdate()  {
        switch (caseState) {
            case -2: // Initialization movements
                pivot.setkP("Normal");
                pivot.setPos("Init");
                wrist.setForearmPos("Init");
                wrist.setBicepPos("Init");
                wrist.setRotationPos(0);
                clawServo.setPosition(RobotConstants.closeClaw);
                break;
            case 0: // Idle
                pivot.setPos("Idle");
                pivot.setkP("Normal");
                //extension.setPos("Idle");
                wrist.setBicepPos("Auton Idle");
                wrist.setForearmPos("Auton Idle");
                break;
            case 1: // Under Specimen
                pivot.setkP("Normal");
                pivot.setPos("Start");
                wrist.setForearmPos("Start");
                wrist.setBicepPos("Start");
                break;
            case 2: // Placing Specimen
                pivot.setkP("Normal");
                pivot.setPos("Start");
                wrist.setForearmPos("Specimen");
                wrist.setBicepPos("Start");
                break;
            case 3: // Picking up from observation zone
                pivot.setPos("Start");
                pivot.setkP("Normal");
                //extension.setPos("Idle");
                wrist.setBicepPos("Basket");
                wrist.setForearmPos("Idle");
                break;
        }

    }
}



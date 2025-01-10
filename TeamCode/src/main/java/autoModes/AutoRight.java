package autoModes;


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

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Autonomous(name = "Auto Right", group = "Auto")
public class AutoRight extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState, actionState, clawState;
    private String navigation;

    //Start pose
    private Pose startPose = new Pose(9.3, 85.3, 0);

    /**Robot width and height were set to 19 and 18 respectively
     * when making the path in the PedroPathing Vercel generator
     */
    //Poses for generic field things
    private Pose blueBasket = new Pose(19, 95, Math.toRadians(135));
    private Pose leftSub = new Pose(51.5,96.5,Math.toRadians(270));
    private Pose rightSub = new Pose(60,45,Math.toRadians(90));
    private Pose leftBlueSub = new Pose(24.5,74,Math.toRadians(0));
    private Pose observationBlue = new Pose(17,39,Math.toRadians(235));

    //private Pose ;
    //private Path ;
    private PathChain initialRightSub, rightMove, observationMove, observationBack, basketMove, leftSubMove;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        rightMove = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(observationBlue)))
                .setConstantHeadingInterpolation(0)
                .setPathEndTimeoutConstraint(0.5)
                .build()
        ;

        observationMove = follower.pathBuilder()
                .addPath(new BezierLine(new Point(leftBlueSub), new Point(observationBlue)))
                .setLinearHeadingInterpolation(0,Math.toRadians(235))
                .setPathEndTimeoutConstraint(0.5)
                .build()
        ;

        observationBack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(observationBlue), new Point(leftBlueSub)))
                .setLinearHeadingInterpolation(Math.toRadians(235),0)
                .setPathEndTimeoutConstraint(0.5)
                .build()
        ;

        basketMove = follower.pathBuilder()
                .addPath(new BezierLine(new Point(leftBlueSub), new Point(blueBasket)))
                .setLinearHeadingInterpolation(0,Math.toRadians(135))
                .setPathEndTimeoutConstraint(0.5)
                .build()
        ;

        leftSubMove = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(blueBasket), new Point(26.6,104), new Point(40.3,113),new Point(leftSub)))
                .setTangentHeadingInterpolation()
                .setPathEndTimeoutConstraint(0.5)
                .build()
        ;

        initialRightSub = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(rightSub), new Point(9.8, 35, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .setPathEndTimeoutConstraint(0.5)
                .build();

    }

    //Cases for larger paths
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 10:
                follower.setMaxPower(0.75);
                follower.followPath(rightMove);
                //setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.atParametricEnd()) {
                    follower.holdPoint(new Pose(leftBlueSub.getX(), leftBlueSub.getY(), leftBlueSub.getHeading()));
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(observationMove);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.atParametricEnd()) {
                    follower.holdPoint(new Pose(observationBlue.getX(), observationBlue.getY(), observationBlue.getHeading()));
                    setPathState(14);
                }
                break;
            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(observationBack);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.atParametricEnd()) {
                    follower.holdPoint(new Pose(leftBlueSub.getX(), leftBlueSub.getY(), leftBlueSub.getHeading()));
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(basketMove);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.atParametricEnd()) {
                    follower.holdPoint(new Pose(blueBasket.getX(), blueBasket.getY(), blueBasket.getHeading()));
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(leftSubMove);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && follower.atParametricEnd()) {
                    follower.holdPoint(new Pose(leftSub.getX(), leftSub.getY(), leftSub.getHeading()));
                    //setPathState(20);
                }
                break;
        }
    }

    /** This switch is called continuously and runs the necessary actions, when finished, it will set the state to -1.
     * (Therefore, it will not run the action continuously) **/
    /* basically more methods we can use if we need them for some reason
    public void autonomousActionUpdate() {
        switch (actionState) {
            case 0:
                setClawState(0);
                setActionState(-1);
                break;
            case 1:
                setClawState(1);
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
        pathTimer.resetTimer();
        autonomousActionUpdate();
    }*/


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop(){
        // These loop the actions and movement of the robot
        follower.update();
        autonomousPathUpdate();
        //autonomousActionUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop(){
        /*
        Put camera/sensor initialization stuff in here
         */

        // After 4 Seconds, Robot Initialization is complete
        if (opmodeTimer.getElapsedTimeSeconds() > 4) {
            telemetry.addData("Init", "Finished");
        }
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start(){
        buildPaths();
        opmodeTimer.resetTimer();
        setPathState(10);
        //setActionState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop(){
    }
}


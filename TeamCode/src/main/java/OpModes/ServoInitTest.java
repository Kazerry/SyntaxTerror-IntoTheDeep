package OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import config.RobotHardware;

@Config
@TeleOp(name = "ServoInitTest", group = "Test")
public class ServoInitTest extends OpMode {
    private Servo bicepLeft;
    private Servo bicepRight;
    private Servo forearm;
    private Servo rotation;

    // Motors for telemetry
    private DcMotorEx rightExtension;
    private DcMotorEx leftExtension;
    private DcMotorEx rightPivot;
    private DcMotorEx leftPivot;

    @Override
    public void init() {
        // Initialize servos
        bicepLeft = hardwareMap.get(Servo.class, "bicepLeft");
        bicepRight = hardwareMap.get(Servo.class, "bicepRight");
        forearm = hardwareMap.get(Servo.class, "forearm");
        rotation = hardwareMap.get(Servo.class, "rotation");

        // Initialize motors for telemetry
        rightExtension = hardwareMap.get(DcMotorEx.class, "rightExtension");
        leftExtension = hardwareMap.get(DcMotorEx.class, "leftExtension");
        rightPivot = hardwareMap.get(DcMotorEx.class, "rightPivot");
        leftPivot = hardwareMap.get(DcMotorEx.class, "leftPivot");

        // Reset all motor encoders
        leftPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors back to run mode
        leftPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set initial positions
        updateServoPositions();

        // Initialize dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        // Update servo positions continuously during init
        updateServoPositions();

        // Telemetry
        telemetry.addData("=== Servo Positions ===", "");
        telemetry.addData("Bicep Test Position", RobotHardware.PosTestBicep);
        telemetry.addData("Forearm Test Position", RobotHardware.PosTestForearm);
        telemetry.addData("Bicep Left Current", bicepLeft.getPosition());
        telemetry.addData("Bicep Right Current", bicepRight.getPosition());
        telemetry.addData("Forearm Current", forearm.getPosition());
        telemetry.addData("Rotation Current", rotation.getPosition());

        telemetry.addData("=== Motor Positions ===", "");
        telemetry.addData("Right Extension Pos", rightExtension.getCurrentPosition());
        telemetry.addData("Left Extension Pos", leftExtension.getCurrentPosition());
        telemetry.addData("Right Pivot Pos", rightPivot.getCurrentPosition());
        telemetry.addData("Left Pivot Pos", leftPivot.getCurrentPosition());

        telemetry.addLine("\nNote: Adjust PosTestBicep and PosTestForearm in Dashboard");
        telemetry.update();
    }

    @Override
    public void loop() {
        // This OpMode is designed for initialization testing only
        telemetry.addLine("This OpMode is for initialization testing only.");
        telemetry.addLine("No need to press Start - adjust values in Dashboard during Init.");
        telemetry.update();
    }

    private void updateServoPositions() {
        bicepLeft.setPosition(RobotHardware.PosTestBicep);
        bicepRight.setPosition(1 - RobotHardware.PosTestBicep);  // Inverse movement
        forearm.setPosition(RobotHardware.PosTestForearm);
        rotation.setPosition(RobotHardware.PosTestRotation);
    }
}
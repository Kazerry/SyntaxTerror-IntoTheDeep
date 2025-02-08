package config.subsystems;

import static config.RobotHardware.PosTestBicep;
import static config.RobotHardware.PosTestForearm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Config
public class Wrist {

    private Servo rotation, forearm, bicepLeft, bicepRight;
    public static double
            bInit = 1, // Intake 0
            bGrab = 0.35, // Intake 1
            bIntake = 0.39, // Intake 2

            bSpecGrab = 0.215, // Spec 1
            bSpecPlace = 0.61,
            bBasket = 0.5,
            bIdle = 0.5,
            bStart = 0.5,
            bMiddle = 0.4,

            autonIdle = 0.2,

            fInit = 1, //Intake 0
            fGrab = 0.7, // Intake 1
            fIntake = 0.89, // Intake 2

            fSpecGrab = 0.40, //Spec 1
            fSpecPlace = 0.5, //Spec 2
            fBasket = 0.1,
            fIdle = 0.5,
            fStart = 0.5,
            fAutonIdle = 0.5,
            fMiddle = bMiddle;

    private double bicepPos, forearmPos, rotationPos;

    public static HashMap<String, Double> bicepPositions = new HashMap<String, Double>();
    public static HashMap<String, Double> forearmPositions = new HashMap<String, Double>();

    public static double[] rotationPositions = new double[5];

    public Wrist(Servo bicepLeft, Servo bicepRight, Servo forearm, Servo rotation) {
        this.bicepLeft = bicepLeft;
        this.bicepRight = bicepRight;
        this.forearm = forearm;
        this.rotation = rotation;

        //Bicep 0.4 goes towards the back 0.6 goes towards the front
        bicepPositions.put("Init",      bInit); // Intake 0
        bicepPositions.put("Grab",      bGrab); // Intake 1
        bicepPositions.put("Intake",      bIntake); // Intake 2

        bicepPositions.put("SpecGrab",      bSpecGrab); // Spec 1
        bicepPositions.put("Basket",      bBasket);
        bicepPositions.put("Idle",        bIdle);
        bicepPositions.put("Start",       bStart);
        bicepPositions.put("Start2",       0.6);
        bicepPositions.put("Auton Idle",  autonIdle);
        bicepPositions.put("Middle",  bMiddle);
        bicepPositions.put("Specimen",      0.5);
        bicepPositions.put("SpecPlace",      bSpecPlace); //Lowkey fine
        bicepPositions.put("downPlace",      0.392); //same wit this
        bicepPositions.put("gUP",      0.20);
        bicepPositions.put("PosTestBicep", PosTestBicep);

        forearmPositions.put("Init",      fInit); // Intake 0
        forearmPositions.put("Grab", fGrab); // Intake 1
        forearmPositions.put("Intake",      fIntake); // Intake 2

        forearmPositions.put("SpecGrab",   fSpecGrab); // Spec 1
        forearmPositions.put("SpecPlace",      fSpecPlace); // Spec 2
        forearmPositions.put("Basket",      fBasket);
        forearmPositions.put("Idle",        fIdle);
        forearmPositions.put("Start",       fStart);
        forearmPositions.put("Start2",       0.62);
        forearmPositions.put("Auton Idle",  fAutonIdle);
        forearmPositions.put("Specimen",      0.46);
        forearmPositions.put("Middle",      fMiddle);
        forearmPositions.put("gUP",      0.47);
        forearmPositions.put("downPlace",      0.73);
        forearmPositions.put("gPlace",      0.6);
        forearmPositions.put("gPlaceUP",      0.575);
        forearmPositions.put("PosTestForearm", PosTestForearm);

        rotationPositions[0] = 0.3;
        rotationPositions[1] = 0;
        rotationPositions[2] = 0.2;
        rotationPositions[3] = 0.4;
        rotationPositions[4] = 0.95;//180 degrees
    }

    public void update()
    {
        // For opposed servos, one gets the position, the other gets (1 - position)
        bicepLeft.setPosition(bicepPos + 0);
        bicepRight.setPosition(1 - bicepPos);  // Inverse movement
        forearm.setPosition(forearmPos);
        rotation.setPosition(rotationPos);
    }

    public void setBicepPos(String pos)
    {
        bicepPos = bicepPositions.get(pos);
    }
    public void setForearmPos(String pos)
    {
        forearmPos = forearmPositions.get(pos);
    }
    public void setRotationPos(int pos)
    {
        rotationPos = rotationPositions[pos];
    }

    public void setPos(String pos) {
        bicepPos = bicepPositions.get(pos);
        forearmPos = forearmPositions.get(pos);
    }

    public int getCurrentRotationPos() {
        // Return the current rotation position index (0-3)
        for (int i = 0; i < rotationPositions.length; i++) {
            if (Math.abs(rotationPositions[i] - rotationPos) < 0.01) {
                return i;
            }
        }
        return 0; // Default to 0 if no match found
    }
}
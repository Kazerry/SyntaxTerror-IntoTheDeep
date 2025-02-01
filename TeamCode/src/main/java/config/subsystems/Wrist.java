package config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Config
public class Wrist {

    private Servo rotation, forearm, bicepLeft, bicepRight;
    public static double bIntake= 0.0, bBasket = 0.15, bIdle = 0.25, bStart = 0.45,
            bMiddle = 0.5, autonIdle = 0.2, fIntake = 0.4, fBasket = 0.1, fIdle = 0.5, fStart = 0.65,
    fAutonIdle = 0.5, fInit = 1.0; //fIntake = 1.0, fBasket = 0.3, bMiddle = 0.5 bStart = 0.8
    private double bicepPos, forearmPos, rotationPos;

    public static HashMap<String, Double> bicepPositions = new HashMap<String, Double>();
    public static HashMap<String, Double> forearmPositions = new HashMap<String, Double>();

    public static double[] rotationPositions = new double[4];

    public Wrist(Servo bicepLeft, Servo bicepRight, Servo forearm, Servo rotation) {
        this.bicepLeft = bicepLeft;
        this.bicepRight = bicepRight;
        this.forearm = forearm;
        this.rotation = rotation;

        //Bicep 0.4 goes towards the back 0.6 goes towards the front
        bicepPositions.put("Intake",      bIntake);
        bicepPositions.put("Basket",      bBasket);
        bicepPositions.put("Idle",        bIdle);
        bicepPositions.put("Start",       bStart);
        bicepPositions.put("Start2",       0.545);
        bicepPositions.put("Auton Idle",  autonIdle);
        bicepPositions.put("Middle",  bMiddle);
        bicepPositions.put("Init",      1.0);
        bicepPositions.put("Specimen",      0.47);
        bicepPositions.put("Grab",      0.62); //0.525 0.160 change from previous
        bicepPositions.put("Place",      0.325); //Lowkey fine
        bicepPositions.put("downPlace",      0.392); //same wit this
        bicepPositions.put("gPlace",      0.835);
        bicepPositions.put("gUP",      0.50);

        forearmPositions.put("Intake",      fIntake);
        forearmPositions.put("Basket",      fBasket);
        forearmPositions.put("Idle",        fIdle);
        forearmPositions.put("Start",       fStart);
        forearmPositions.put("Start2",       0.62);
        forearmPositions.put("Auton Idle",  fAutonIdle);
        forearmPositions.put("Init",      fInit);
        forearmPositions.put("Specimen",      0.50);
        forearmPositions.put("Middle",      0.50);
        forearmPositions.put("Grab",      0.93); //0.96 -0.03 change from previous
        forearmPositions.put("gUP",      0.47);
        forearmPositions.put("Place",      0.72);
        forearmPositions.put("downPlace",      0.73);
        forearmPositions.put("gPlace",      0.6);
        forearmPositions.put("gPlaceUP",      0.575);

        rotationPositions[0] = 0; //Horizontal
        rotationPositions[1] = 0.65;
        rotationPositions[2] = 0.8; //Vertical
        rotationPositions[3] = 0.88;
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
}
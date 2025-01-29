package config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Config
public class Wrist {

    private Servo rotation, forearm, bicepLeft, bicepRight;
    public static double bIntake= 0.0, bBasket = 0.15, bIdle = 0.25, bStart = 0.45,
            bMiddle = 0.5, autonIdle = 0.2, fIntake = 0.4, fBasket = 0.1, fIdle = 0.5, fStart = 0.35,
    fAutonIdle = 0.5, fInit = 0.8; //fIntake = 1.0, fBasket = 0.3, bMiddle = 0.5 bStart = 0.8
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
        bicepPositions.put("Auton Idle",  autonIdle);
        bicepPositions.put("Middle",  bMiddle);
        bicepPositions.put("Init",      1.0);

        forearmPositions.put("Intake",      fIntake);
        forearmPositions.put("Basket",      fBasket);
        forearmPositions.put("Idle",        fIdle); //Middle
        forearmPositions.put("Start",       fStart);
        forearmPositions.put("Auton Idle",  fAutonIdle);
        forearmPositions.put("Init",      fInit);
        forearmPositions.put("Specimen",      0.3);

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
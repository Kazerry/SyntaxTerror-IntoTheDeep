package config.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@Config
public class Wrist {

    private Servo rotation, forearm, bicepLeft, bicepRight;

    private double bicepPos, forearmPos, rotationPos;

    public static HashMap<String, Double> bicepPositions = new HashMap<String, Double>();
    public static HashMap<String, Double> forearmPositions = new HashMap<String, Double>();

    public static double[] rotationPositions = new double[4];

    public Wrist(Servo bicepLeft, Servo bicepRight, Servo forearm, Servo rotation) {
        this.bicepLeft = bicepLeft;
        this.bicepRight = bicepRight;
        this.forearm = forearm;
        this.rotation = rotation;

        bicepPositions.put("Intake",      0.15);
        bicepPositions.put("Basket",      0.25);
        bicepPositions.put("Idle",        0.25);
        bicepPositions.put("Start",       0.8);
        bicepPositions.put("Auton Idle",  0.2);

        forearmPositions.put("Intake",      1.0);
        forearmPositions.put("Basket",      0.3);
        forearmPositions.put("Idle",        0.5);
        forearmPositions.put("Start",       0.05);
        forearmPositions.put("Auton Idle",  0.5);

        rotationPositions[0] = 0.07;
        rotationPositions[1] = 0.34;
        rotationPositions[2] = 0.61;
        rotationPositions[3] = 0.88;
    }

    public void update()
    {
        // For opposed servos, one gets the position, the other gets (1 - position)
        bicepLeft.setPosition(bicepPos);
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
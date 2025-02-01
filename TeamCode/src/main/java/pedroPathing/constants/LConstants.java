package pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LConstants {
    static {
        OTOSConstants.useCorrectedOTOSClass = false;
        OTOSConstants.hardwareMapName = "OTOS";
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(1.75, 0, 3 * Math.PI / 2);
        OTOSConstants.linearScalar = 1.27; //1.17 on light tiles 0.9763 on dark
        OTOSConstants.angularScalar = 0.995;
    }
}





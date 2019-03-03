package org.firstinspires.ftc.teamcode.Modules;

/**
 * Created by tpowe on 2/21/2019.
 */

public class GyroOrientation {
    public GyroAxis axis = null;
    public double heading = 0.0d;

    public GyroOrientation(double x, double y, double z) {
        this.axis = new GyroAxis(x, y, z);
    }

    public GyroOrientation(double x, double y, double z, double heading) {
        this.axis = new GyroAxis(x, y, z);
        this.heading = heading;
    }
}

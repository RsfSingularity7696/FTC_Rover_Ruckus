package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class GyroModule {
    // The IMU sensor object
    private BNO055IMU imu;

    // State used for updating telemetry
    private Orientation angles;

    public void init(OpMode opMode) {
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void start() {
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public GyroOrientation loop() {

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double heading = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
        double roll = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle);
        double pitch = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle);
        GyroOrientation gyroOrientation = new GyroOrientation(AngleUnit.DEGREES.normalize(heading), AngleUnit.DEGREES.normalize(roll), AngleUnit.DEGREES.normalize(pitch));

        //When robot moves up, x value goes  up
        //When robot move right, z value goes down
        //when robot barell rolls clockwise, y goes down
        return gyroOrientation;
    }

    public void stop() {
        imu.stopAccelerationIntegration();
    }
}

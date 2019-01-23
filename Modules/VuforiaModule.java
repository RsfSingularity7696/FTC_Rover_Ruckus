package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

public class VuforiaModule {
    private static final String VUFORIA_KEY = "ARHbQtb/////AAABmWcIXrrnZERtjdHTy4MLMsBfrh68ImsBSX4N2+nD+E4HiRVLKmQkN+xrViWcSCKbMYpSRjwM0Ug0IuKFeZ61GHCWEZ/70fh3R6Q1ZLauKTBXPZfoRu64tDEYOUDdIpsU9fd/wpkNq4mRWDyVxfQGjiP2CHefp9pLAFQ+Eesb0HC0Z1l7dsPIJD6je0wEOCKnahQzQE7ZJ1ojBFQYj3eCrnfBPVXgPB9tdPHiMeNnvAvGRg8qFVP+OJEjD08/J3maDAbLdUOZVRKkNTe56EQO/eaFSwLBvwyoRCurHs64EEjUyIRjUBoz1OyJ75vVR7QYrozq9LjwrF+8OlqrfxFpfab5mSR/0PzyZ3vx5/jbjJYl";

    protected VuforiaLocalizer vuforia;

    public void init(OpMode opMode) {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        //parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
}

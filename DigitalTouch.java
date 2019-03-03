package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by tpowe on 2/25/2019.
 */

public class DigitalTouch {
    private DigitalChannel digitalTouch = null;

    public void init(OpMode opMode){
        // get a reference to our digitalTouch object.
        digitalTouch = opMode.hardwareMap.get(DigitalChannel.class, "sd");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);
    }
    public  boolean loop(){
      //  Test test = new Test();
      //  test.mode = digitalTouch.getMode();
      //  test.state = digitalTouch.getState();

        return !digitalTouch.getState();
    }


    public class Test{
       // public  boolean state = false;
       // public  DigitalChannel.Mode mode;
    }
}

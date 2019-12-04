/*
Copyright (c) 2016 Robert Atkinson
All rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:
Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.
Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Distance_Sensor;
import org.firstinspires.ftc.teamcode.Modules.GryoModule;
import org.firstinspires.ftc.teamcode.Modules.SampleOp_EngineModule;
import org.firstinspires.ftc.teamcode.Modules.SampleOp_States;

//import com.qualcomm.robotcore.hardware.DistanceSensor;

//In this test I programmed everything according to the chart on my phone. Programmed the motors to move with encoders
@TeleOp(name = "AutonomousTest_Distance", group = "Pushbot")
public class AutonomousTest_Distance extends OpMode {
    private SampleOp_EngineModule engine = new SampleOp_EngineModule();
    TestSoundsSKYSTONE sounds = new TestSoundsSKYSTONE();
    GryoModule imu = new GryoModule();
    Distance_Sensor distance = new Distance_Sensor();

    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;

    private int stage = 1;

    private double timeOffSet = 0;

    private int mechanum = 0;

    //private  String soundstring = sounds.sounds[0];
    SoundPlayer soundplaya = new SoundPlayer(3, 6);

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);
        //sounds.myApp = hardwareMap.appContext;
        imu.initialize(hardwareMap);
        //accuator = hardwareMap.servo.get("accuator");


        engine.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        //distance.start();
        engine.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        imu.start();
        //  sounds.runOpMode();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
       // distance.runOpMode();
        switch (stage) {
          //  case 1:
          //      if(distance.sensorRange.getDistance(DistanceUnit.INCH) >= 6.0d){
          //          engine.Move(SampleOp_States.Dpad.Up, 0.2d);
          //      }
          //      break;

           // case 1:
           //     if(imu.currentRotation()){
           //
           //     }
           //     break;
           /* case 2:
                if(distance.sensorRange.getDistance(DistanceUnit.INCH) >= 6.0d){

                    engine.Move(SampleOp_States.Dpad.Up, -0.20d);
                    // sounds.playSound(12);
                    //sounds.soundPlaying = false;
                   // imu.currentRotation(GryoModule.Axis.X);

                }
                else{
                    NextStage();
                }


                break;
                */
            case 1:
                if(distance.sensorRange.getDistance(DistanceUnit.INCH) >= 10.0d){

                }
                break;
               default:
                engine.Stop();
                break;
        }

        telemetry.addData("Stage: ", stage);
        telemetry.addData("FrontLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontLeft));
        telemetry.addData("FrontRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontRight));
        telemetry.addData("BackLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackLeft));
        telemetry.addData("BackRight: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.BackRight));
        telemetry.addData("x: ", imu.currentRotation(GryoModule.Axis.X));
        telemetry.addData("y: ", imu.currentRotation(GryoModule.Axis.Y));
        telemetry.addData("z: ", imu.currentRotation(GryoModule.Axis.Z));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        engine.Stop();


    }

    public SampleOp_States.Dpad GetInputs(Gamepad gamepad) {
        if (gamepad.dpad_down && gamepad.dpad_left) {
            return SampleOp_States.Dpad.DownLeft;
        } else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        } else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        } else if (gamepad.dpad_up && gamepad.dpad_left) {
            return SampleOp_States.Dpad.UpLeft;
        } else if (gamepad.dpad_up && gamepad.dpad_right) {
            return SampleOp_States.Dpad.UpRight;
        } else if (gamepad.dpad_down) {
            return SampleOp_States.Dpad.Down;
        } else if (gamepad.dpad_left) {
            return SampleOp_States.Dpad.Left;
        } else if (gamepad.dpad_right) {
            return SampleOp_States.Dpad.Right;
        } else if (gamepad.dpad_up) {
            return SampleOp_States.Dpad.Up;
        } else {
            return SampleOp_States.Dpad.None;
        }
    }

    private double clamp(double value) {
        if (value > maxSpeed) {
            return maxSpeed;
        } else if (value < -maxSpeed) {
            return -maxSpeed;
        } else {
            return value;
        }
    }

    private void GoToStage(int target) {
        timeOffSet = time;
        stage = target;
    }

    private void NextStage() {
        timeOffSet = time;
        stage++;
    }

    private void PreviousStage() {
        timeOffSet = time;
        stage--;
    }


    private void turn(int degrees) {

    }

    private void move(int distance, double power) {
        if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > distance) {
            if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > distance) {
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
            }
            engine.SetSpeed(-power, -power);
        }
    }

    private void reset() {
        if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
            engine.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }


}
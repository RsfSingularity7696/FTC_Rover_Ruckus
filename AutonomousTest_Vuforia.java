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


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.ImageTarget;
import org.firstinspires.ftc.teamcode.Modules.SampleOp_EngineModule;
import org.firstinspires.ftc.teamcode.Modules.SampleOp_States;
//import org.firstinspires.ftc.teamcode.Modules.TensorFlowModule;
import org.firstinspires.ftc.teamcode.Modules.VuforiaModule;

//import com.qualcomm.robotcore.hardware.DistanceSensor;

//In this test I programmed everything according to the chart on my phone. Programmed the motors to move with encoders
@TeleOp(name = "AutonomousTest_VuforiaRed", group = "Pushbot")
public class AutonomousTest_Vuforia extends OpMode {
    private SampleOp_EngineModule engine = new SampleOp_EngineModule();
    TestSoundsSKYSTONE sounds = new TestSoundsSKYSTONE();
    VuforiaModule vuforia = new VuforiaModule();
    ImageTarget target = new ImageTarget();
     XYZ xyzPosition = new XYZ();
    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;


    private int stage = -3;

    private double timeOffSet = 0;

    private Servo gripLeft = null;
    private Servo gripRight = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        vuforia.initialize(this.hardwareMap);
        engine.Initialize(hardwareMap);
        sounds.myApp = hardwareMap.appContext;
        engine.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gripLeft = hardwareMap.servo.get("gripLeft");
        gripLeft.setDirection(Servo.Direction.REVERSE);

        gripRight = hardwareMap.servo.get("gripRight");
        gripRight.setDirection(Servo.Direction.REVERSE);

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

        vuforia.start();
        engine.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripRight.setPosition(0.0d);
        gripLeft.setPosition(0.0d);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        runVuforia();


        switch (stage) {
            case -5:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > -1000) {
                    if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    engine.SetSpeed(-0.3d, 0.3d, -0.3d, 0.3d);
                } else {
                   GoToStage(100);
                }
                break;
            case -3:
                if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getCurrentPosition() > -550) {
                    if (engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        engine.GetMotor(SampleOp_EngineModule.EngineMotor.FrontLeft).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    engine.SetSpeed(-0.3d, -0.3d);

                } else {
                   GoToStage(-1);

                }
                break;
            case -2:
                if(wait(4.0) > time) {


                    reset();
                    NextStage();
                }
                break;
            case -1:
                if(target != null){
                    //engine.SetSpeed(-0.3d, 0.3d, -0.3d, 0.3d);
                    NextStage();

                }
                else{
                    engine.SetSpeed(-0.3d, 0.3d, -0.3d, 0.3d);
                }
                break;
            case 0:
                runVuforia();
                XYZ position2 = new XYZ();
                position2.setY(8);
                position2.setX(-16);
                //locationPosition(null, position2, "x") &&
                if ( locationPosition(null, position2, "y")) {
                    NextStage();
                    engine.Stop();
                }
                else{
                    engine.SetSpeed(-0.4d, 0.4d, -0.4d, 0.4d);
                }
                break;
            case 1:
                XYZ position1 = new XYZ();
                position1.setX(-16);

                if (locationPosition(position1, null, "x")) {
                    NextStage();
                    engine.Stop();
                }
                else{
                    engine.SetSpeed(-0.18d, -0.18d);
                }
                break;
            default:
                engine.Stop();
                break;
        }
        telemetry.addData("Stage: ", stage);
        telemetry.addData("FrontLeft: ", engine.GetPosition(SampleOp_EngineModule.EngineMotor.FrontLeft));

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

    public double wait(double increment) {
        return time + increment;
    }

    public boolean runVuforia() {
        // Provide feedback as to where the robot is located (if we know).
        target = vuforia.run();

        if (target != null) {
            // express position (translation) of robot in inches.
            telemetry.addData("Visible Target", target.name);

            if (target.position != null) {
                float mmPerInch = 25.4f;

                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        target.position.get(0) / mmPerInch, target.position.get(1) / mmPerInch, target.position.get(2) / mmPerInch);

            }

            if (target.rotation != null) {
                // express the rotation of the robot in degrees.
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", target.rotation.firstAngle, target.rotation.secondAngle, target.rotation.thirdAngle);
            }
        } else {
            telemetry.addData("Visible Target", "none");
        }

        telemetry.update();
        return false;
    }

    public boolean locationPosition(XYZ position, XYZ rotation , String axis) {
        if (target != null) {
            if (position != null) {
                switch (axis) {
                    case "x":
                    case "X":
                        return checkAxisX(position, "position");
                    case "y":
                    case "Y":
                        return checkAxisY(position, "position");
                    case "z":
                    case "Z":
                        return checkAxisZ(position, "position");
                    default:
                        return false;
                }
            }
            else if (rotation != null) {
                switch (axis) {
                    case "x":
                    case "X":
                        return checkAxisX(rotation, "rotation");
                    case "y":
                    case "Y":
                        return checkAxisY(rotation, "rotation");
                    case "z":
                    case "Z":
                        return checkAxisZ(rotation, "rotation");
                    default:
                        return false;
                }
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }

    private boolean checkAxis(ImageTarget imageTarget, XYZ coordinate, String axis, String type) {
        if (type == "position") {
            switch (axis) {
                case "x":
                case "X":
                    return (coordinate.getX() <= (imageTarget.position.get(0) + coordinate.getTolerance())) || (coordinate.getX() >= (imageTarget.position.get(0) - coordinate.getTolerance()));
                case "y":
                case "Y":
                    return (coordinate.getY() <= (imageTarget.position.get(1) + coordinate.getTolerance())) || (coordinate.getY() >= (imageTarget.position.get(1) - coordinate.getTolerance()));
                case "z":
                case "Z":
                    return (coordinate.getZ() <= (imageTarget.position.get(2) + coordinate.getTolerance())) || (coordinate.getZ() >= (imageTarget.position.get(2) - coordinate.getTolerance()));
                default:
                    return false;
            }
        }
        else if (type == "rotation") {
            switch (axis) {
                case "x":
                case "X":
                    return (coordinate.getX() <= (imageTarget.rotation.firstAngle + coordinate.getTolerance())) || (coordinate.getX() >= (imageTarget.rotation.firstAngle - coordinate.getTolerance()));
                case "y":
                case "Y":
                    return (coordinate.getY() <= (imageTarget.rotation.secondAngle + coordinate.getTolerance())) || (coordinate.getY() >= (imageTarget.rotation.secondAngle - coordinate.getTolerance()));
                case "z":
                case "Z":
                    return (coordinate.getZ() <= (imageTarget.rotation.thirdAngle + coordinate.getTolerance())) || (coordinate.getZ() >= (imageTarget.rotation.thirdAngle - coordinate.getTolerance()));
                default:
                    return false;
            }
        }
        else {
            return false;
        }
    }

    public boolean checkAxisX(XYZ xyz, String type) {
        return checkAxis(target, xyz, "x", type);
    }

    public boolean checkAxisY(XYZ xyz, String type) {
        return checkAxis(target, xyz, "y", type);
    }

    public boolean checkAxisZ(XYZ xyz, String type) {
        return checkAxis(target, xyz, "z", type);
    }
}
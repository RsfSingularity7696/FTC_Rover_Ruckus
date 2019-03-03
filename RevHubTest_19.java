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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.ArmModule;
import org.firstinspires.ftc.teamcode.Modules.EngineModule;
import org.firstinspires.ftc.teamcode.Modules.TensorFlowModule;
@Disabled
@TeleOp(name="RevHub_19", group="Pushbot")
public class RevHubTest_19 extends OpMode {
    private EngineModule engine = new EngineModule();
    private ArmModule army = new ArmModule();

    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;


    DcMotor carriage = null;
    DcMotor collector = null;

    Servo leftDump = null;
    Servo rightDump = null;

    boolean dumpRight = false;
    boolean dumpLeft = false;

    double timestamp = 0.0d;
    boolean power = false;
    boolean collect = false;
    boolean out = false;
    private double maxMotorPower = 1.0d;

    boolean pressed = false;
    int stage = 0;

    double dumpWait = 0.5d;
    double leftDumpWait = 0.0d;
    double rightDumpWait = 0.0d;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);
        army.init(hardwareMap);

        collector = hardwareMap.dcMotor.get("collector");
        collector.setDirection(DcMotor.Direction.FORWARD);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setPower(0.0d);

        carriage = hardwareMap.dcMotor.get("carriage");
        carriage.setDirection(DcMotor.Direction.FORWARD);
        carriage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carriage.setPower(0.0d);

        rightDump = hardwareMap.servo.get("rightDump");
        leftDump = hardwareMap.servo.get("leftDump");
        rightDump.setDirection(Servo.Direction.REVERSE);
        leftDump.setDirection(Servo.Direction.FORWARD);
        rightDump.setPosition(0.0d);
        leftDump.setPosition(0.0d);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

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
        engine.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */

    @Override
    public void loop() {
        // army.loop(this);

        //Drive engines
        double left = clamp(gamepad1.left_stick_y);
        double right = clamp(gamepad1.right_stick_y);

        if (gamepad1.left_bumper) {
            if (army.arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                army.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            //army.lift.setTargetPosition(4893);
            army.arm.setTargetPosition(1780);
        }
        if (gamepad1.a) {
            if (army.arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                army.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            /*if (army.lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                army.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }*/
            if (army.arm.getCurrentPosition() >= 875) {
                army.arm.setPower(-0.20);
            } else {
                army.arm.setPower(0.35d);
            }

            //army.lift.setPower(1.0d);
            //army.lift.setTargetPosition(4893);
            army.arm.setTargetPosition(2120);
        } else if (gamepad1.b) {
            if (army.arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                army.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            /*if (army.lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                army.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }*/

            //army.lift.setTargetPosition(50);
            //army.lift.setPower(-1.0d);
            army.arm.setTargetPosition(0);
            army.arm.setPower(-0.3d);
        } else if (gamepad1.y) {
            if (army.arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                army.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            army.arm.setTargetPosition(1279);
            army.arm.setPower(0.35d);
            /*if(army.arm.getCurrentPosition() > 1279){
                army.arm.setPower(-0.3d);
            }
            else if(army.arm.getCurrentPosition() < 1279){
                army.arm.setPower(0.35d);
            }*/

        }

        if (gamepad1.right_trigger >= 0.3d) {
            army.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            army.arm.setPower(-gamepad1.right_trigger);
        } else if (gamepad1.left_trigger >= 0.3d) {
            army.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            army.arm.setPower(gamepad1.left_trigger);
        } else {
            army.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            army.arm.setPower(0.5d);
            //army.arm.setTargetPosition(army.arm.getCurrentPosition());
        }

        if (gamepad2.y) {
            if (army.lift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                army.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            army.lift.setPower(1.0d);
        } else if (gamepad2.x) {
            if (army.lift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                army.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            army.lift.setPower(-1.0d);
        } else {
            if (army.lift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                army.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            army.lift.setPower(0.0d);
        }

        if (gamepad1.left_bumper) {
            // engine.SetMaxMotorPower(1.0d);
            army.lift.setPower(0.4d);
        } else if (gamepad1.right_bumper) {
            // engine.SetMaxMotorPower(0.50d);
            army.lift.setPower(-0.4d);
        }

        if (!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up) {
            engine.SetSpeed(left, left, right, right);
        } else {
            engine.Move(GetInputs(gamepad1), 1.0d);
        }

        if (gamepad1.x) {
            army.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        //other motors

        if (gamepad2.b) {
            out = true;
            collect = false;
            // power = true;
            //  collector.setPower(0.75d);
        } else if (gamepad2.a) {
            collect = true;
            out = false;

            // collector.setPower(-1.0d);
        } else {
            //  collector.setPower(0.0d);
            //    power = false;
        }
        if (gamepad2.left_bumper) {
            if (!dumpRight && wait(rightDumpWait, dumpWait)) {
                rightDump.setPosition(1.0d);
                dumpRight = true;
                rightDumpWait = time;
            } else if (dumpRight && wait(rightDumpWait, dumpWait)) {
                rightDump.setPosition(0.0d);
                dumpRight = false;
                rightDumpWait = time;
            }

        } else if (gamepad2.right_bumper) {
            if (!dumpLeft && wait(leftDumpWait, dumpWait)) {
                leftDump.setPosition(1.0d);
                dumpLeft = true;
                leftDumpWait = time;
            } else if (dumpLeft && wait(leftDumpWait, dumpWait)) {
                leftDump.setPosition(0.0d);
                dumpLeft = false;
                leftDumpWait = time;
            }
        }

        if (gamepad2.right_trigger >= 0.1d) {
            collect = false;
            out = false;
            collector.setPower(0.0d);
        }

        if (collect) {
            collector.setPower(-1.0d);
        } else if (out) {
            /*if (time % 2 == 0) {
                collector.setPower(0.0d);
            } else {
                collector.setPower(0.42d);
            }*/

          /*  if (time > timestamp + 0.20d) {
                power = !power;
                timestamp = time;
            }
            if (power) {
                collector.setPower(0.7d);
            } else {
                collector.setPower(0.0d);
            }
            */
            collector.setPower(-0.6d);
            // collector.setPower(0.42d);
        }

        if (gamepad2.dpad_down) {
            carriage.setPower(1.0d);
        } else if (gamepad2.dpad_up) {
            carriage.setPower(-1.0d);
        } else {
            carriage.setPower(0.0d);
        }

        if (gamepad1.a && !pressed) {
            pressed = true;
        }

        //else if(gamepad1.b){
        //   arm.setPower(0.0d);
        //   lift.setPower(0.0d);
        // }
        // else{
        //      arm.setTargetPosition(arm.getCurrentPosition());
        //      lift.setTargetPosition(lift.getCurrentPosition());
        // }

        //telemetry

        //      telemetry.addData("Time: ", time);
        //      telemetry.addData("Left Stick: ", gamepad1.left_stick_y);
        //      telemetry.addData("Right Stick: ", gamepad1.right_stick_y);
        //   telemetry.addData("Left Power: ", left);
        //  telemetry.addData("Right Power: ", right);
        //  telemetry.addData("Max Speed: ", maxSpeed);
        //     telemetry.addData("Arm: ", army.arm.getCurrentPosition());
        //     telemetry.addData("Lift: ", army.lift.getCurrentPosition());

        telemetry.addData("Time: ", time);

        telemetry.addData("Arm: ", army.arm.getCurrentPosition());
        telemetry.addData("LeftB: ", engine.GetMotor(EngineModule.EngineMotor.BackLeft).getCurrentPosition());
        telemetry.addData("LeftF: ", engine.GetMotor(EngineModule.EngineMotor.FrontLeft).getCurrentPosition());
        telemetry.addData("RightB: ", engine.GetMotor(EngineModule.EngineMotor.BackRight).getCurrentPosition());
        telemetry.addData("RightF: ", engine.GetMotor(EngineModule.EngineMotor.FrontRight).getCurrentPosition());

        telemetry.addData("Left Wait: ", leftDumpWait);
        telemetry.addData("Right Wait: ", rightDumpWait);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        engine.Stop();
        army.stop();
        //   arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carriage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public SampleOp_States.Dpad GetInputs(Gamepad gamepad) {
        if (gamepad.dpad_down && gamepad.dpad_left) {
            return SampleOp_States.Dpad.DownLeft;
        }
        else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        }
        else if (gamepad.dpad_down && gamepad.dpad_right) {
            return SampleOp_States.Dpad.DownRight;
        }
        else if (gamepad.dpad_up && gamepad.dpad_left) {
            return SampleOp_States.Dpad.UpLeft;
        }
        else if (gamepad.dpad_up && gamepad.dpad_right) {
            return SampleOp_States.Dpad.UpRight;
        }
        else if (gamepad.dpad_down) {
            return SampleOp_States.Dpad.Down;
        }
        else if (gamepad.dpad_left) {
            return SampleOp_States.Dpad.Left;
        }
        else if (gamepad.dpad_right) {
            return SampleOp_States.Dpad.Right;
        }
        else if (gamepad.dpad_up) {
            return SampleOp_States.Dpad.Up;
        }
        else {
            return SampleOp_States.Dpad.None;
        }
    }

    private double clamp(double value) {
        if (value > maxSpeed) {
            return maxSpeed;
        }
        else if (value < -maxSpeed) {
            return -maxSpeed;
        }
        else {
            return value;
        }
    }

    private boolean wait(double timestamp, double wait) {
        return time > timestamp + wait;
    }
}
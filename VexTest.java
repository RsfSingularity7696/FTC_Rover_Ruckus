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

import org.firstinspires.ftc.teamcode.Modules.TensorFlowModule;

@TeleOp(name="RevHub 01", group="Pushbot")
public class VexTest extends OpMode {
    //private EngineMod_7696 engine = new EngineMod_7696();

    private double maxSpeed = 1.0d;
    private double moveSpeed = 1.0d;

    private double maxMotorPower = 1.0d;

    private TensorFlowModule tensorFlowModule = new TensorFlowModule();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /*engine.Initialize(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");*/

        tensorFlowModule.init(this);
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
        tensorFlowModule.start();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Drive engines
        /*double left = clamp(gamepad1.left_stick_y);
        double right = clamp(gamepad1.right_stick_y);

        if (gamepad1.left_bumper) {
            engine.SetMaxMotorPower(1.0d);
        }
        else if (gamepad1.right_bumper) {
            engine.SetMaxMotorPower(0.50d);
        }

        if (!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up) {
            engine.SetSpeed(left, left, right, right);
        }
        else {
            engine.Move(GetInputs(gamepad1), 1.0d);
        }



        //telemetry

        telemetry.addData("Left Stick: ", gamepad1.left_stick_y);
        telemetry.addData("Right Stick: ", gamepad1.right_stick_y);
        //   telemetry.addData("Left Power: ", left);
        //  telemetry.addData("Right Power: ", right);
        //  telemetry.addData("Max Speed: ", maxSpeed);

        //    telemetry.addData("LeftB: ", engine.GetMotor(EngineMod_7696.EngineMotor.BackLeft).getCurrentPosition());
        //   telemetry.addData("LeftF: ", engine.GetMotor(EngineMod_7696.EngineMotor.FrontLeft).getCurrentPosition());
        //  telemetry.addData("RightB: ", engine.GetMotor(EngineMod_7696.EngineMotor.BackRight).getCurrentPosition());
        //  telemetry.addData("RightF: ", engine.GetMotor(EngineMod_7696.EngineMotor.FrontRight).getCurrentPosition());

        telemetry.update();*/

        tensorFlowModule.loop(this);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        //engine.Stop();

        tensorFlowModule.stop();
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
}
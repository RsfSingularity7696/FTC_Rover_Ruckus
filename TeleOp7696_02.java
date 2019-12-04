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

import org.firstinspires.ftc.teamcode.Modules.SampleOp_EngineModule;
import org.firstinspires.ftc.teamcode.Modules.SampleOp_States;

//this test will have provided for thechanges on the lists. Anything that was deleted or commented out in this program can be found in Test 25.
@TeleOp(name="7696 Teleop_02", group="Pushbot")
public class TeleOp7696_02 extends OpMode {

    private double maxSpeed = 1.0d;

    private SampleOp_EngineModule engine = new SampleOp_EngineModule();

    private DcMotor stone_Lift = null;
    private DcMotor extension = null;
    private DcMotor collectorRight = null;
    private DcMotor collectorLeft = null;

    private Servo grip = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        engine.Initialize(hardwareMap);

        stone_Lift = hardwareMap.dcMotor.get("lift");
        extension = hardwareMap.dcMotor.get("extension");
        collectorLeft = hardwareMap.dcMotor.get("collectorLeft");
        collectorRight = hardwareMap.dcMotor.get("collectorRight");

        grip = hardwareMap.servo.get("grip");
        grip.setDirection(Servo.Direction.REVERSE);



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

        stone_Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        collectorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        grip.setPosition(0.0);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left = clamp(-gamepad1.right_stick_y);
        double right = clamp(-gamepad1.left_stick_y);


        if (gamepad1.right_bumper) {
            engine.SetMaxMotorPower(0.5d);
        }
        else if (gamepad1.left_bumper){
            engine.SetMaxMotorPower(1.0d);
        }

        //This controls the mechanum movement staments as well as the joysticks. Please refer to sampleOp_Engine Module
        //For more information and how the joysticks and robot moves.
        if (!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up) {
            engine.SetSpeed(left, right);
        } else {
            engine.Move(GetInputs(gamepad1), 1.0d);
        }


        //this extends the vertical lift
        if(gamepad2.y){
            stone_Lift.setPower(0.5d);
        } else{
            stone_Lift.setPower(0.0d);
        }

        if(gamepad2.a){
            stone_Lift.setPower(-0.5d);
        } else{
            stone_Lift.setPower(0.0d);
        }


        //this extends the lift arm
        if(gamepad2.right_trigger >= 0.1d){
            extension.setPower(1.0d);
        } else if(gamepad2.right_trigger < 0.1d && gamepad2.right_trigger >= 0.0d){
          extension.setPower(0.0d);
        }

        if(gamepad2.left_trigger >= 0.1d){
            extension.setPower(-1.0d);
        } else if(gamepad2.left_trigger < 0.1d && gamepad2.left_trigger >= 0.0d){
            extension.setPower(0.0d);
        }


        //these lines  of  code control the collector
        if(gamepad2.left_stick_y >= 0.01d) {
            collectorLeft.setPower(1.0d);
        }
       else if(gamepad2.left_stick_y < -0.01d){
            collectorLeft.setPower(-1.0d);
        }
        else{
            collectorLeft.setPower(0.0d);
        }

        if(gamepad2.right_stick_y >= 0.01d) {
            collectorRight.setPower(1.0d);
        }
        else if(gamepad2.right_stick_y < -0.01d){
            collectorRight.setPower(-1.0d);
        }
        else{
            collectorRight.setPower(0.0d);
        }


        //this controls the stone gripper
        if(gamepad1.right_trigger >=0.1d){
            grip.setPosition(1.0d);
        }

        if(gamepad1.left_trigger >=0.1d){
            grip.setPosition(-1.0d);
        }


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

    private int clamp(int min, int max, int value) {
        return Math.max(min, Math.min(max, value));
    }



}
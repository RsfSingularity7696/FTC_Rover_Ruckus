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

import org.firstinspires.ftc.teamcode.Modules.ArmModule;
import org.firstinspires.ftc.teamcode.Modules.EngineModule;

@TeleOp(name="RevHubTest_Mag", group="Pushbot")
public class RevHubTest_Mag extends OpMode {
    private EngineModule engine = new EngineModule();
    private ArmModule army = new ArmModule();
    DigitalTouch touch = new DigitalTouch();

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
        touch.init(this);

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
        boolean isPressed = touch.loop();

        army.loop(this, isPressed);

        //Drive engines
        double left = clamp(gamepad1.left_stick_y);
        double right = clamp(gamepad1.right_stick_y);

        /*if (gamepad2.y) {
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
            army.lift.setPower(0.4d);
        } else if (gamepad1.right_bumper) {
            army.lift.setPower(-0.4d);
        }*/

        if (!gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right && !gamepad1.dpad_up) {
            engine.SetSpeed(left, left, right, right);
        } else {
            engine.Move(GetInputs(gamepad1), 1.0d);
        }

        //other motors
        if (gamepad2.b) {
            out = true;
            collect = false;
        } else if (gamepad2.a) {
            collect = true;
            out = false;
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

        if (collect) {
            collector.setPower(-1.0d);
        } else if (out) {
            collector.setPower(-0.6d);
        }

        if (gamepad2.dpad_down) {
            carriage.setPower(1.0d);
        } else if (gamepad2.dpad_up) {
            carriage.setPower(-1.0d);
        } else {
            carriage.setPower(0.0d);
        }

        if (gamepad2.left_stick_y > 0.2d || gamepad2.left_stick_x > 0.2d) {
            collect = false;
            out = false;
            collector.setPower(0.0d);
        }
        if (gamepad2.right_stick_y > 0.2d || gamepad2.right_stick_x > 0.2d) {
            collector.setPower(1.0d);
            collect = false;
        }

        telemetry.addData("Time: ", time);
        telemetry.addData("IsPressed: ", isPressed);
        telemetry.addData("Arm: ", army.arm.getCurrentPosition());
        telemetry.addData("Arm Mode: ", army.arm.getMode());
        telemetry.addData("Arm Power: ", army.arm.getPower());
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
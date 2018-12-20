/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Test Op", group="Iterative Opmode")
public class BasicTestOp extends OpMode
{
    // Declare OpMode members.
    DcMotor arm = null;
    DcMotor carriage = null;
    DcMotor collector = null;
    DcMotor lift = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        arm = hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotor.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setPower(0.0d);

        collector = hardwareMap.dcMotor.get("collector");
        collector.setDirection(DcMotor.Direction.FORWARD);
        collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        collector.setPower(0.0d);

        carriage = hardwareMap.dcMotor.get("carriage");
        carriage.setDirection(DcMotor.Direction.FORWARD);
        carriage.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carriage.setPower(0.0d);

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(0.0d);

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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad2.a) {
            collector.setPower(1.0d);
        }
        else if (gamepad2.b) {
            collector.setPower(-1.0d);
        }
        else {
            collector.setPower(0.0d);
        }

        if (gamepad2.y) {
            lift.setPower(1.0d);
        }
        else if (gamepad2.x) {
            lift.setPower(-1.0d);
        }
        else {
            lift.setPower(0.0d);
        }

        if (gamepad2.left_bumper) {
            arm.setPower(1.0d);
        }
        else if (gamepad2.right_bumper) {
            arm.setPower(-1.0d);
        }
        else {
            arm.setPower(0.0d);
        }

        if (gamepad2.dpad_up) {
            carriage.setPower(1.0d);
        }
        else if (gamepad2.dpad_down) {
            carriage.setPower(-1.0d);
        }
        else {
            carriage.setPower(0.0d);
        }

        telemetry.addData("arm", arm.getCurrentPosition());
        telemetry.addData("carriage", carriage.getCurrentPosition());
        telemetry.addData("lift", lift.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carriage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

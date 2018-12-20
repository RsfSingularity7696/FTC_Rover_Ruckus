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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Basic: Autonomous_Depot", group="Iterative Opmode")

public class BasicOpMode_Autonomous_R2 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    int stage = 0;

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft = hardwareMap.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

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
        runtime.reset();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (stage) {
            case 0:
                if (frontRight.getCurrentPosition() > -5500) {
                    frontRight.setPower(-.6d);
                    frontLeft.setPower(-.6d);
                    backRight.setPower(-.6d);
                    backLeft.setPower(-.6d);

                } else {
                    stage++;
                }
                break;
            default:
                stopMotors();
                break;
        }

        /*leftDrive.setPower(1.0d);
        rightDrive.setPower(1.0d);
        if(leftDrive.getCurrentPosition() > 5000 || rightDrive.getCurrentPosition() > 5000){
            leftDrive.setPower(0.0d);
            rightDrive.setPower(0.0d);
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        */


        /*switch (stage){
            case 0:
                leftDrive.setPower(-0.5d);
                rightDrive.setPower(0.5d);
                if(leftDrive.getCurrentPosition() < -3000 || rightDrive.getCurrentPosition() > 3000){
                    leftDrive.setPower(0.0d);
                    rightDrive.setPower(0.0d);
                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    stage = 1;
                }
               else{

                }
                break;
            case 1:
                leftDrive.setPower(1.0d);
                rightDrive.setPower(1.0d);
                if(leftDrive.getCurrentPosition() > 4000 || rightDrive.getCurrentPosition() > 4000){
                    leftDrive.setPower(0.0d);
                    rightDrive.setPower(0.0d);
                    leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    stage = 2;
                }
                break;
            default:
                leftDrive.setPower(0.0d);
                rightDrive.setPower(0.0d);
                break;
        }*/


        telemetry.addData("Back Left: ", backLeft.getCurrentPosition());
        telemetry.addData("Front Left: ", frontLeft.getCurrentPosition());
        telemetry.addData("Back Right: ", backRight.getCurrentPosition());
        telemetry.addData("Front Right: ", frontRight.getCurrentPosition());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        stopMotors();

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stopMotors() {
        backLeft.setPower(0.0d);
        backRight.setPower(0.0d);
        frontLeft.setPower(0.0d);
        frontRight.setPower(0.0d);
    }
}

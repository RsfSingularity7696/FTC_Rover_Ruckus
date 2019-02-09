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

package org.firstinspires.ftc.teamcode.Old_Autonomous_Programs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Modules.TensorFlowModule;

/*
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
@Disabled
@TeleOp(name="Auto: Team 8606 Detector", group="Iterative Opmode")
public class Autonomous7696_R3 extends OpMode
{
    // Declare OpMode members.
    private TensorFlowModule detector = new TensorFlowModule();


    private Servo marker = null;
    private String sample = "";
    private boolean endgame = false;
    private int ascend = -2715;
    private int decend = -10;
    private int stage = 0;
    private double timestamp = 0.0f;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        detector.init(this);


        marker = hardwareMap.get(Servo.class, "Marker");

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
        detector.start();


        stage = 0;
        timestamp = time;
        sample = "";
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        switch (stage) {
            case 0:
                // Sample Minerals
                if (sample == "" && wait(4.0d) > time) {
                    sample = detector.loop(this);
                }
                else {
                    next();
                }
                break;
            case 1:
                // Lower Robot
                if (wait(4.0d) > time) {
                }
                else {

                    next();
                }
                break;
            case 2:
                // Back Up
                if (wait(0.25d) > time) {
                    move(-0.25d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
               //     move(0.0d);
                    next();
                }
                break;
            case 3:
                // Turn
                if (wait(0.60d) > time) {
                //    move(0.0d, 1.0d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                 //   move(0.0d);
                    next();
                }
                break;
            case 4:
                // Move Forward
                if (wait(0.50d) > time) {
                    move(0.50d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                 //   move(0.0d);
                    next();
                }
                break;
            case 5:
                // Retract Lift
                if (wait(2.0d) > time) {
                }
                else {
                    next();
                }
                break;
            case 6:
                // Turn Back
                if (wait(0.70d) > time) {
                  //  move(0.0d, -1.0d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                 //   move(0.0d);
                    next();
                }
                break;

            default:
                break;
        }


        /*switch (stage) {
            case 0:
                // Lower Robot
                if (wait(4.0d) > time) {
                    liftLeft.setPower(-0.35d);
                    liftRight.setPower(-0.35d);

                    liftLeft.setTargetPosition(ascend);
                    liftRight.setTargetPosition(ascend);
                }
                else {
                    liftLeft.setPower(0.0d);
                    liftRight.setPower(0.0d);

                    next();
                }
                break;
            case 1:
                // Back Up
                if (wait(0.25d) > time) {
                    move(-0.25d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                    move(0.0d);
                    next();
                }
                break;
            case 2:
                // Turn
                if (wait(0.60d) > time) {
                    move(-0.30d, 0.30d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                    move(0.0d);
                    next();
                }
                break;
            case 3:
                // Move Forward
                if (wait(0.50d) > time) {
                    move(0.50d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                    move(0.0d);
                    next();
                }
                break;
            case 4:
                // Turn Back
                if (wait(0.70d) > time) {
                    move(0.30d, -0.30d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                    move(0.0d);
                    next();
                }
                break;
            case 5:
                // Retract Lift
                if (wait(2.0d) > time) {
                    if (liftRight.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }

                    liftLeft.setPower(-0.35d);
                    //liftRight.setPower(-0.35d);

                    liftLeft.setTargetPosition(decend);
                    //liftRight.setTargetPosition(decend);
                }
                else {
                    liftLeft.setPower(0.0d);
                    //liftRight.setPower(0.0d);

                    next();
                }
                break;
            case 6:
                // Square Up
                if (wait(0.60d) > time) {
                    move(-0.50d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                    move(0.0d);
                    next();
                }
                break;
            case 7:
                // Park
                if (wait(0.55d) > time) {
                    move(1.0d, DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else {
                    move(0.0d);
                    next();
                }
                break;
            case 8:
                // Forward Deposit
                marker.setPosition(0.60d);
                next();
                break;
            default:
                break;
        }*/

        telemetry.addData("Time: ", time);
        telemetry.addData("Stage: ", stage);

        telemetry.addData("Sample: ", sample);

      /*  telemetry.addData("Lift Left: ", liftLeft.getCurrentPosition());
        telemetry.addData("Lift Right: ", liftRight.getCurrentPosition());
        telemetry.addData("Drive Left: ", driveLeftFront.getCurrentPosition());
        telemetry.addData("Drive Right: ", driveRightFront.getCurrentPosition());
*/
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
       /* driveLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        */
        marker.setPosition(0.0d);
    }

    public void move(double power, DcMotor.RunMode mode) {
      //  move(power, power, mode);
    }

   /* public void move(double left, double right, DcMotor.RunMode mode) {
        if (driveLeftFront.getMode() != mode) {
            driveLeftFront.setMode(mode);
        }

        if (driveLeftBack.getMode() != mode) {
            driveLeftBack.setMode(mode);
        }

        if (driveRightFront.getMode() != mode) {
            driveRightFront.setMode(mode);
        }

        if (driveRightBack.getMode() != mode) {
            driveRightBack.setMode(mode);
        }

        driveLeftFront.setPower(left);
        driveLeftBack.setPower(left);
        driveRightFront.setPower(right);
        driveRightBack.setPower(right);
    }

    public void move(double power) {
        driveLeftFront.setPower(power);
        driveLeftBack.setPower(power);
        driveRightFront.setPower(power);
        driveRightBack.setPower(power);
    }
    */

    public void next() {
        next(stage + 1);
    }

    public void next(int target) {
        stage = target;
        timestamp = time;
    }

    public void previous() {
        next(stage - 1);
    }

    public double wait(double increment) {
        return timestamp + increment;
    }
}

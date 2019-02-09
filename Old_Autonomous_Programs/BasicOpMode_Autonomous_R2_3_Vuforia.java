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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Modules.ArmModule;
import org.firstinspires.ftc.teamcode.Modules.EngineModule;
import org.firstinspires.ftc.teamcode.Modules.TensorFlowModule;

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
@Disabled
@TeleOp(name="Auto Depot2 Vuforia", group="Iterative Opmode")

public class BasicOpMode_Autonomous_R2_3_Vuforia extends OpMode
{
    ArmModule army = new ArmModule();

    int stage = 0;

    private String sample = "";
    private double timestamp = 0.0d;
    private TensorFlowModule detector = new TensorFlowModule();

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

        army.init(hardwareMap);
        detector.init(this);

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontRight = hardwareMap.dcMotor.get("frontRight");
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backLeft = hardwareMap.dcMotor.get("backLeft");
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        backRight = hardwareMap.dcMotor.get("backRight");
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
       // engine.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        army.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        army.arm.setPower(0.0d);

        detector.start();

        stage = 0;
        timestamp = time;
        sample = "";
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    //@Override
    @Override
    public void loop() {
        double left = -0.3d;
        double right = 0.3d;

        switch (stage) {
            case 0:
                if (sample == "" && wait(4.0d) > time) {
                    sample = detector.loop(this);
                }
                else {
                    next();
                }
                break;
            case 1:
                if(sample.equals("Left")){
                    next(2);
                }
                else if (sample.equals("Right")){
                    next(3);
                }
                else {
                    next(4);
                }
                break;
            case 2:
                if (wait(1.02d) > time) {
                    frontLeft.setPower(right);
                    backLeft.setPower(right);
                    frontRight.setPower(left);
                    backRight.setPower(left);
                }
                else {
                    stopMotors();
                    next(5);
                }
                break;
            case 3:
                if (wait(1.02d) > time) {
                    frontLeft.setPower(left);
                    backLeft.setPower(left);
                    frontRight.setPower(right);
                    backRight.setPower(right);
                    army.arm.setPower(0.0d);
                }
                else {
                    stopMotors();
                    next(6);
                }
                break;
            //Center
            case 4:
                if (wait(4.0d) > time){
                    frontLeft.setPower(left);
                    backLeft.setPower(left);
                    frontRight.setPower(-right);
                    backRight.setPower(-right);
                    army.arm.setPower(0.0d);
                }
                else{
                    stopMotors();
                    next(10);
                }
                break;
            //Left
            case 5:
                if (wait(4.5d) > time){
                    frontLeft.setPower(left);
                    backLeft.setPower(left);
                    frontRight.setPower(-right);
                    backRight.setPower(-right);
                    army.arm.setPower(0.0d);
                }
                else{
                    stopMotors();
                    next(7);
                }
                break;
            //Right
            case 6:
                if (wait(4.5d) > time){
                    frontLeft.setPower(left);
                    backLeft.setPower(left);
                    frontRight.setPower(-right);
                    backRight.setPower(-right);
                    army.arm.setPower(0.0d);
                }
                else{
                    stopMotors();
                    next(8);
                }
                break;
            //Left turn
            case 7:
                if(wait(2.34d) > time) {
                    frontLeft.setPower(left);
                    backLeft.setPower(left);
                    frontRight.setPower(right);
                    backRight.setPower(right);
                }
                else {
                    stopMotors();
                    next(9);
                }
                //Right turn
            case 8:
                if(wait(2.34d) > time){
                    frontLeft.setPower(right);
                    backLeft.setPower(right);
                    frontRight.setPower(left);
                    backRight.setPower(left);
                }
                else{
                    stopMotors();
                    next(9);
                }
                break;
            case 9:
                if(wait(3.0d) > time){
                    frontLeft.setPower(left);
                    backLeft.setPower(left);
                    frontRight.setPower(-right);
                    backRight.setPower(-right);
                }
                else{
                    stopMotors();
                    next(10);
                }
                break;
            case 10:
                if(army.arm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    army.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if(wait(2.0d) > time){
                    army.arm.setPower(0.35d);
                }
                else{
                    stopMotors();
                    next();
                }
                break;
            default:
                stopMotors();
                break;
        }

        frontLeft.setPower(left);
        backLeft.setPower(left);
        frontRight.setPower(right);
        backRight.setPower(right);
    }
        public void loop2() {
        switch (stage) {
            case 0:
                if (sample == "" && wait(4.0d) > time) {
                    sample = detector.loop(this);
                }
                else {
                    next();
                }
                break;
            case 1:
                next(2);

                /*if(sample.equals("Left")){
                    next(2);
                }
                else if (sample.equals("Right")){
                    next(3);
                }
                else {
                    next(4);
                }*/
                break;
            case 2:
                if (wait(1.02d) > time) {
                  //  engine.SetSpeed(0.3d, -0.3d);
                    army.arm.setPower(0.0d);
                }
                else {
                    stopMotors();
                    next(5);
                }
                break;
            case 3:
                if (wait(1.02d) > time) {
                  //  engine.SetSpeed(-0.3d, 0.3d);
                    army.arm.setPower(0.0d);
                }
                else {
                    stopMotors();
                    next(6);
                }
                break;
            //Center
            case 4:
                if (wait(4.0d) > time){
                  //  engine.SetSpeed(-0.3d, -0.3d);
                    army.arm.setPower(0.0d);
                }
                else{
                    stopMotors();
                    next(100);
                }
                break;
            //Left
            case 5:
                if (wait(4.5d) > time){
                 //   engine.SetSpeed(-0.3d, -0.3d);
                    army.arm.setPower(0.0d);
                }
                else{
                    stopMotors();
                    next(7);
                }
                break;
            //Right
            case 6:
                if (wait(4.5d) > time){
                  //  engine.SetSpeed(-0.3d, -0.3d);
                    army.arm.setPower(0.0d);
                }
                else{
                    stopMotors();
                    next(8);
                }
                break;
            //Left turn
            case 7:
                if(wait(2.34d) > time) {
                    //engine.SetSpeed(-0.3d, 0.3d);

                    double left = -0.3d;
                    double right = 0.3d;

                 //   engine.GetMotor(EngineModule.EngineMotor.FrontLeft).setPower(left);
                 //   engine.GetMotor(EngineModule.EngineMotor.BackLeft).setPower(left);
                 //   engine.GetMotor(EngineModule.EngineMotor.FrontRight).setPower(right);
                 //   engine.GetMotor(EngineModule.EngineMotor.BackRight).setPower(right);
                }
                else {
                    stopMotors();
                    next(9);
                }
            //Right turn
            case 8:
                if(wait(2.34d) > time){
                 //   engine.SetSpeed(0.3d, -0.3d);
                }
                else{
                    stopMotors();
                    next(9);
                }
                break;
            case 9:
                if(wait(3.0d) > time){
                 //   engine.SetSpeed(-0.3d, -0.3d);
                }
                else{
                    stopMotors();
                    next(100);
                }
                break;
            case 10:
                if(army.arm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    army.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if(wait(2.0d) > time){
                    army.arm.setPower(0.35d);
                }
                else{
                    stopMotors();
                    //army.arm.setPower(0.0d);
                    next();
                }
                break;
            default:
                stopMotors();
                break;
        }


        //IMU calibration with lander at 0 0
        //if vuforia == left, use imu to turn 20 degrees left. drive forward. Turn 20 degrees right. Drive forward
        //if vuforia == center, use motors to drive forward.
        //if vuforia == right, use  imu to turn 20 degrees right. drive forward. Imu turn 20 degrees left. Drive forward.
        //telemetry.addData("Back Left: ", engine.GetPosition(EngineModule.EngineMotor.BackLeft));
        //telemetry.addData("Front Left: ", engine.GetPosition(EngineModule.EngineMotor.FrontLeft));
        //telemetry.addData("Back Right: ", engine.GetPosition(EngineModule.EngineMotor.BackRight));
        //telemetry.addData("Front Right: ", engine.GetPosition(EngineModule.EngineMotor.FrontRight));
        telemetry.addData("Sample: ", sample);
        telemetry.addData("Arm: " , army.arm.getCurrentPosition());
        telemetry.addData("Stage: ", stage);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
       // engine.Stop();
       // engine.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void stopMotors() {
        army.arm.setPower(0.0d);
      //  engine.SetSpeed(0.0d);
    }
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

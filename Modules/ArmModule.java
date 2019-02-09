package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by tpowe on 12/27/2018.
 */

public class ArmModule {
    public DcMotor arm = null;
    public DcMotor lift = null;

    boolean pressed = false;
    int stage = 0;

    int arm1 = 500;
    int arm2 = 1000;
    int lift1 = 0;
    int lift2 = 150;

    public void init(HardwareMap hardwareMap) {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(1.0d);
        arm.setTargetPosition(arm.getCurrentPosition());

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //  lift.setPower(1.0d);
        lift.setTargetPosition(lift.getCurrentPosition());
    }

    public void loop(OpMode opMode) {

        if (opMode.gamepad2.y) {
            lift.setPower(1.0d);
        } else if (opMode.gamepad2.x) {
            lift.setPower(-1.0d);
        } else {
            lift.setPower(0.0d);
        }

      /*  if (opMode.gamepad2.right_trigger >= 0.3d) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(-opMode.gamepad2.right_trigger);
        } else if (opMode.gamepad2.left_trigger >= 0.3d) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            arm.setPower(opMode.gamepad2.left_trigger);
        } else {
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.5d);
            arm.setTargetPosition(arm.getCurrentPosition());
        }
        */
       /* if (opMode.gamepad1.a && !pressed) {
            pressed = true;
        }
        if (pressed) {

            switch (stage) {
                case 0:
                    if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if (lift.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    if (arm.getCurrentPosition() >= -50 || lift.getCurrentPosition() <= 30) {
                        arm.setPower(-1.0d);
                        lift.setPower(1.0d);
                        arm.setTargetPosition(-500);
                        lift.setTargetPosition(0);
                    } else {
                        stage++;
                    }
                    break;
                case 1:
                    if (arm.getCurrentPosition() <= -490 || lift.getCurrentPosition() <= 30) {
                        arm.setPower(-1.0d);
                        lift.setPower(1.0d);
                        arm.setTargetPosition(-1000);
                        lift.setTargetPosition(150);
                    }
                    else{
                        stage++;
                    }
                    break;
                default:
                    arm.setPower(0.0d);
                    lift.setPower(0.0d);
                    break;
            }


        }
        */
        opMode.telemetry.addData("arm", arm.getCurrentPosition());
        opMode.telemetry.addData("lift", lift.getCurrentPosition());
    }

    public void stop() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}

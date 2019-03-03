package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * Created by tpowe on 12/27/2018.
 */

public class ArmModule {
    public enum ArmMode {
        Crater, Done, Down, Hang, Up
    }

    public DcMotor arm = null;
    public DcMotor lift = null;

    public ArmMode armMode = ArmMode.Done;
    public double armSpeed = 0.0d;
    public int armPosition = 0;

    public void init(HardwareMap hardwareMap) {
        arm = hardwareMap.dcMotor.get("arm");
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.0d);
        arm.setTargetPosition(arm.getCurrentPosition());

        lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(lift.getCurrentPosition());
    }

    public void loop (OpMode opMode, boolean isPressed) {
        if (opMode.gamepad2.y) {
            lift.setPower(1.0d);
        } else if (opMode.gamepad2.x) {
            lift.setPower(-1.0d);
        } else {
            lift.setPower(0.0d);
        }

        if (opMode.gamepad2.y) {
            if (lift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            lift.setPower(1.0d);
        } else if (opMode.gamepad2.x) {
            if (lift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            lift.setPower(-1.0d);
        } else {
            if (lift.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            lift.setPower(0.0d);
        }

        if (opMode.gamepad1.left_bumper) {
            lift.setPower(0.4d);
        } else if (opMode.gamepad1.right_bumper) {
            lift.setPower(-0.4d);
        }

        if (opMode.gamepad1.x) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if (opMode.gamepad1.left_bumper) {
            moveArmToPosition(ArmMode.Hang, 1780, 0.50d);
        } else if (opMode.gamepad1.a || opMode.gamepad2.left_trigger > 0.1d) {
            moveArmToPosition(ArmMode.Up, 1800, 0.50d);
        } else if (opMode.gamepad1.b) {
            moveArmToPosition(ArmMode.Down, 0, -0.40d);
        } else if (opMode.gamepad2.right_trigger >= 0.1d) {
            moveArmToPosition(ArmMode.Crater, 610, 0.50d);
        } else if (opMode.gamepad1.right_trigger >= 0.3d) {
            moveArm(-opMode.gamepad1.right_trigger);
        } else if (opMode.gamepad1.left_trigger >= 0.3d) {
            moveArm(opMode.gamepad1.left_trigger);
        } else {
            if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        if (arm.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
            if (isPressed && armMode == ArmMode.Up) {
                armPosition = arm.getCurrentPosition() + 890;
                armMode = ArmMode.Done;
            }

            arm.setTargetPosition(armPosition);
        }

        arm.setPower(armSpeed);
    }

    public void loop(OpMode opMode) {
        if (opMode.gamepad2.y) {
            lift.setPower(1.0d);
        } else if (opMode.gamepad2.x) {
            lift.setPower(-1.0d);
        } else {
            lift.setPower(0.0d);
        }

        opMode.telemetry.addData("arm", arm.getCurrentPosition());
        opMode.telemetry.addData("lift", lift.getCurrentPosition());
    }

    public void stop() {
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void moveArm(double armSpeed) {
        if (arm.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        this.armSpeed = armSpeed;
    }

    private void moveArmToPosition(ArmMode armMode, int armPosition, double armSpeed) {
        if (arm.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        this.armMode = armMode;
        this.armPosition = armPosition;
        this.armSpeed = armSpeed;
    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Modules.ArmModule;

/**
 * Created by tpowe on 1/3/2019.
 */

@TeleOp(name="liftPosition", group="Pushbot")
public class LiftPosition extends OpMode {
    private ArmModule army = new ArmModule();

    public void init() {
        army.init(hardwareMap);
    }

    public void start() {
        army.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        army.arm.setPower(0.0d);

        army.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        army.lift.setPower(0.0d);
    }

    public void loop() {
        if (gamepad2.y) {
            army.lift.setPower(1.0d);
        } else if (gamepad2.x) {
            army.lift.setPower(-1.0d);
        } else {
            army.lift.setPower(0.0d);
        }

        telemetry.addData("arm", army.arm.getCurrentPosition());
        telemetry.addData("lift", army.lift.getCurrentPosition());
        telemetry.update();
    }

    public void stop() {
        army.stop();
    }
}

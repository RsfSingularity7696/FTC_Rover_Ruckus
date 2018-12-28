package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.EngineModule;
import org.firstinspires.ftc.teamcode.SampleOp_States;

/**
 * Created by tpowe on 12/27/2018.
 */

public class EngineModule {
    public enum EngineMotor {
        BackLeft,
        BackRight,
        FrontLeft,
        FrontRight
    }

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private double maxMotorPower = 1.0d;

    public DcMotor GetMotor(EngineModule.EngineMotor motor) {
        switch (motor) {
            case BackLeft:
                return backLeft;
            case BackRight:
                return backRight;
            case FrontLeft:
                return frontLeft;
            case FrontRight:
                return frontRight;
            default:
                return null;
        }
    }

    public int GetAveragePosition() {
        return (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition() + backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4;
    }

    public int GetPosition(EngineModule.EngineMotor motor) {
        return GetMotor(motor).getCurrentPosition();
    }

    public void Initialize(HardwareMap hardwareMap) {
        this.frontLeft = SetMotor(hardwareMap, "frontLeft", DcMotorSimple.Direction.REVERSE);
        this.frontRight = SetMotor(hardwareMap, "frontRight");
        this.backLeft = SetMotor(hardwareMap, "backLeft", DcMotorSimple.Direction.REVERSE);
        this.backRight = SetMotor(hardwareMap, "backRight");

        Stop();
    }

    public void Move(SampleOp_States.Dpad direction, double motorSpeed) {
        switch (direction) {
            case Down:
                SetSpeed(motorSpeed);
                break;
            case DownLeft:
                SetSpeed(0.0d, -motorSpeed, -motorSpeed, 0.0d);
                break;
            case DownRight:
                SetSpeed(0.0d, motorSpeed, motorSpeed, 0.0d);
                break;
            case Left:
                SetSpeed(-motorSpeed, motorSpeed, motorSpeed, -motorSpeed);
                break;
            case Right:
                SetSpeed(motorSpeed, -motorSpeed, -motorSpeed, motorSpeed);
                break;
            case Up:
                SetSpeed(-motorSpeed);
                break;
            case UpLeft:
                SetSpeed(-motorSpeed, 0.0d, 0.0d, -motorSpeed);
                break;
            case UpRight:
                SetSpeed(motorSpeed, 0.0d, 0.0d, motorSpeed);
                break;
            default:
                SetSpeed(0.0d);
                break;
        }
    }

    public void SetMaxMotorPower(double power) {
        maxMotorPower = power;
    }

    public void SetMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private DcMotor SetMotor(HardwareMap hardwareMap, String motorName) {
        return SetMotor(hardwareMap, motorName, DcMotor.Direction.FORWARD);
    }

    private DcMotor SetMotor(HardwareMap hardwareMap, String motorName, DcMotor.Direction direction) {
        return SetMotor(hardwareMap, motorName, direction, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private DcMotor SetMotor(HardwareMap hardwareMap, String motorName, DcMotor.Direction direction, DcMotor.RunMode runMode) {
        DcMotor motor = hardwareMap.dcMotor.get(motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);

        return motor;
    }

    public void SetSpeed(double power) {
        SetSpeed(power, power);
    }

    public void SetSpeed(double left, double right) {
        SetSpeed(left, left, right, right);
    }

    public void SetSpeed(double backLeft, double frontLeft, double backRight, double frontRight) {
        this.backLeft.setPower(clamp(backLeft));
        this.frontLeft.setPower(clamp(frontLeft));
        this.backRight.setPower(clamp(backRight));
        this.frontRight.setPower(clamp(frontRight));
    }

    public void Stop() {
        SetSpeed(0.0d);
    }

    private double clamp(double value) {
        if (value > maxMotorPower) {
            return maxMotorPower;
        }
        else if (value < -maxMotorPower) {
            return -maxMotorPower;
        }
        else {
            return value;
        }
    }
}

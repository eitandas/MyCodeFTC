package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Utils.PIDFController;

import java.util.Arrays;

public class Arm {
    private final DcMotorEx angleMotor;
    private final DcMotorEx extendMotor;
    private final PIDFController pidf = new PIDFController(0, 0, 0, 0);
    private final double[] angleLevels = {0, 90, 160, 210};
    private final double[] extendLevels = {0, 1000, 2000, 3000};
    private final double tolerance = 0;
    private double position;

    public Arm(OpMode opMode){
        angleMotor = opMode.hardwareMap.get(DcMotorEx.class,"angleMotor");
        extendMotor = opMode.hardwareMap.get(DcMotorEx.class,"extendMotor");
        init();
    }

    public void init(){
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        angleMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(){
    }

    public void setAnglePower(double power){
        angleMotor.setPower(power);
    }

    public void setExtendPower(double power){
        angleMotor.setPower(power);
    }

    public void setArmAngle(int level){
        double position = angleMotor.getCurrentPosition();
        double angle = Math.toRadians(angleLevels[level]);
        double power = pidf.calculate(position, angle);
        setAnglePower(power);
    }

    public void setArmExtend(int level){
        double position = extendMotor.getCurrentPosition();
        double angle = Math.toRadians(extendLevels[level]);
        double power = pidf.calculate(position, angle);
        setExtendPower(power);
    }
}

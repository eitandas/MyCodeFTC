package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utils.PIDFController;
import org.firstinspires.ftc.teamcode.Utils.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.Utils.Poses.Vector2d;

public class DriveTrain {
    private final DcMotor rightFront;
    private final DcMotor rightBack;
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final IMU imu;
    private final PIDFController pidX = new PIDFController(0, 0, 0, 0);
    private final PIDFController pidY = new PIDFController(0, 0, 0, 0);
    private final PIDFController pidZ = new PIDFController(0, 0, 0, 0);
    //TODO change if needed
    private final IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
    private DcMotor leftXEncoder;
    private DcMotor rightXEncoder;
    private DcMotor yEncoder;
    private final double xTolerance = 0;
    private final double yTolerance = 0;
    private final double zTolerance = 0;
    private double xPosition;
    private double yPosition;
    private double angle;

    public DriveTrain(OpMode opMode) {
        rightFront = opMode.hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = opMode.hardwareMap.get(DcMotor.class, "rightBack");
        leftFront = opMode.hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = opMode.hardwareMap.get(DcMotor.class, "leftBack");
        imu = opMode.hardwareMap.get(IMU.class, "IMU");
        init();
    }

    public void init() {
        imu.initialize(parameters);
        imu.resetYaw();
        setMotorDirections();
        encodersSetup();
        setTolerance();
    }

    public void setMotorDirections() {
        //TODO change directions if needed
//        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void encodersSetup() {
        leftXEncoder = leftFront;
        rightXEncoder = rightFront;
        yEncoder = leftBack;
        resetEncoders();
        setEncodersDirections();
    }

    public void setEncodersDirections() {
        //TODO change directions if needed
        //leftXEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightXEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
        //yEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void resetEncoders() {
        leftXEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftXEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightXEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightXEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTolerance() {
        pidX.setTolerance(xTolerance);
        pidY.setTolerance(yTolerance);
        pidZ.setTolerance(zTolerance);
    }

    public void update() {
        xPosition = (leftXEncoder.getCurrentPosition() + rightXEncoder.getCurrentPosition()) / 2;
        yPosition = yEncoder.getCurrentPosition();
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void stop() {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }

    public void setPower(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getAngle();
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1);
        double rightFrontPower = (y - x - z) / denominator;
        double rightBackPower = (y + x - z) / denominator;
        double leftFrontPower = (y + x + z) / denominator;
        double leftBackPower = (y - x + z) / denominator;
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
    }

    public void FieldCentricDrive(Vector2d vec, double z) {
        vec.rotateBy(angle);
        setPower(new Pose2d(vec, z));
    }

    public void autonomusDrive(Pose2d pose, double timeOut) {
        ElapsedTime timer = new ElapsedTime();
        double x = pose.getX();
        double y = pose.getY();
        double z = pose.getAngle();
        pidX.setSetPoint(x);
        pidY.setSetPoint(y);
        pidZ.setSetPoint(z);
        while (!pidX.atSetPoint() && !pidY.atSetPoint() && !pidZ.atSetPoint() || timer.seconds() < timeOut) {
            pidX.calculate(xPosition, x);
            pidY.calculate(yPosition, y);
            pidZ.calculate(angle, z);
        }
    }
}



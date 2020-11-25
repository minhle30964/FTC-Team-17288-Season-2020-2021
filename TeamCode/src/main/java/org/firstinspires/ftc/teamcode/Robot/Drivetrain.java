package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Drivetrain {

    Telemetry telemetry;
    LinearOpMode opMode;

    public Drivetrain(Telemetry teleme, LinearOpMode tempOpMode) {
        telemetry = teleme;
        opMode = tempOpMode;
    }

    public enum Direction {
        FORWARD, BACKWARD
    }

    public enum Turn {
        LEFT, RIGHT
    }

    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    // Gyroscope //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    double kp = 0.027; // test and edit later on //
    double targetAngle = 0;
    double modularAngle;
    double error;
    double Power;

    private final double COUNTS_PER_MOTOR_REV = 1120;    // Andy Mark Motor Encoder
    private final double DRIVE_GEAR_REDUCTION = 1;     // This is < 1.0 if geared UP
    private final double WHEEL_DIAMETER_INCHES = 3.75;     // For figuring circumference
    private final double ONE_ROTATION_INCHES = WHEEL_DIAMETER_INCHES * Math.PI; // circumferences
    private final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / ONE_ROTATION_INCHES;

    private double SCALING_RATIO = 0.75;



    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.dcMotor.get("FL");
        frontRight = hardwareMap.dcMotor.get("FR");
        backLeft = hardwareMap.dcMotor.get("BL");
        backRight = hardwareMap.dcMotor.get("BR");

        // Gyroscope //
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    // Simple drivetrain methods //
    public void drive(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void stopDriving() {
        drive(0);
    }

    public void turn(double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);
    }

    public void Strafe(double power) {
        frontLeft.setPower(power);
        backLeft.setPower(-power);
        frontRight.setPower(-power);
        backRight.setPower(power);
    }

    // complex drivetrain methods //
    public void turnWithEncoder(Turn turn, double distance, double power) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        if (turn == Turn.RIGHT) {
            frontLeft.setTargetPosition(newFrontLeftTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            frontRight.setTargetPosition(-newFrontRightTarget);
            backRight.setTargetPosition(-newBackRightTarget);
        } else if (turn == Turn.LEFT) {
            frontLeft.setTargetPosition(-newFrontLeftTarget);
            backLeft.setTargetPosition(-newBackLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backRight.setTargetPosition(newBackRightTarget);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turn(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opMode.opModeIsActive()) {

        }

        stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void turnWithGyro(double Angle) {

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        targetAngle = Angle;

        if(targetAngle % 180 == 0 || targetAngle != 0)
            modularAngle = 180;
        else
            modularAngle =(targetAngle)%180;

        while (Math.abs(targetAngle - angles.firstAngle) > 1.5 && opMode.opModeIsActive()) {

            error = targetAngle - angles.firstAngle;
            Power = (kp) * (error);

            frontLeft.setPower(-Power);
            frontRight.setPower(Power);
            backLeft.setPower(-Power);
            backRight.setPower(Power);

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("error: ", error);
            telemetry.addData("target: ", targetAngle);
            telemetry.addData("angle: ", angles.firstAngle);
            telemetry.addData("modular angle", modularAngle);
            telemetry.update();

        }

        stopDriving();

    }

    public void strafe(Turn direction, double distance, double power) {
        int Target;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Target = frontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        if (direction == Turn.LEFT) {
            frontLeft.setTargetPosition(-Target);
            frontRight.setTargetPosition(Target);
            backLeft.setTargetPosition(Target);
            backRight.setTargetPosition(-Target);
        } else if (direction == Turn.RIGHT) {
            frontLeft.setTargetPosition(Target);
            frontRight.setTargetPosition(-Target);
            backLeft.setTargetPosition(-Target);
            backRight.setTargetPosition(Target);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opMode.opModeIsActive()) {
        }

        stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void drive(Direction direction, double distance, double power) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        newFrontLeftTarget = frontLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newFrontRightTarget = frontRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackLeftTarget = backLeft.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newBackRightTarget = backRight.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        if (direction == Direction.BACKWARD) {
            frontLeft.setTargetPosition(-newFrontLeftTarget);
            frontRight.setTargetPosition(-newFrontRightTarget);
            backLeft.setTargetPosition(-newBackLeftTarget);
            backRight.setTargetPosition(-newBackRightTarget);
        } else if (direction == Direction.FORWARD) {
            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        drive(power);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy() && opMode.opModeIsActive()){

        }

        stopDriving();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // Telo-Op //
    public void ArcadeDrive(double linear, double rotation, double strafe) {
        frontLeft.setPower((rotation - linear + strafe) * SCALING_RATIO);
        backLeft.setPower((rotation - linear - strafe) * SCALING_RATIO);
        frontRight.setPower((-rotation - linear - strafe) * SCALING_RATIO);
        backRight.setPower((-rotation - linear + strafe) * SCALING_RATIO);
    }

    public void TankDrive(double left, double right, double strafe) {
        frontLeft.setPower((left + strafe) * SCALING_RATIO);
        backLeft.setPower((left - strafe) * SCALING_RATIO);
        frontRight.setPower((right - strafe) * SCALING_RATIO);
        backRight.setPower((right + strafe) * SCALING_RATIO);
    }


}
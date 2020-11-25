package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot.Drivetrain;

@TeleOp (name = "TeleOp")
public class TeleOpProgram extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    Drivetrain drivetrain = new Drivetrain(telemetry, this);

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // Drivetrain //
            // Tank Drive //
            drivetrain.TankDrive(-gamepad1.right_stick_y, -gamepad1.left_stick_y, -gamepad1.left_stick_x);
            // Arcade Drive //
            //drivetrain.ArcadeDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

}

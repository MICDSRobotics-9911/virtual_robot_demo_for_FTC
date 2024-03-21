package org.firstinspires.ftc.teamcode.SampleEducationalPrograms.advanced;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.Drivetrain;

@Autonomous(name="Sample_Gyro_And_Encoder_Auto", group="advanced")
public class Sample_Gyro_And_Encoder_Auto extends OpMode {
    private Drivetrain drivetrain;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
    }
    public void start() {
    }

    @Override
    public void loop() {
        telemetry.addData("Target IMU Angle: ", 180);
        telemetry.addData("Current IMU Angle", drivetrain.getHeading());
        drivetrain.moveForward(0.5, 10);
    }
}

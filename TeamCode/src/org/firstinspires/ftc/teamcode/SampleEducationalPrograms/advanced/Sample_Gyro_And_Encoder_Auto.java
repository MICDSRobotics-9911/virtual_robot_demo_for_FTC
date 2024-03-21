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
        drivetrain.moveForward(0.5, 10);
        drivetrain.turnTo(Math.toRadians(90));
        drivetrain.moveBackward(0.7, 10);
        drivetrain.strafeLeft(0.3, 5);
        drivetrain.strafeRight(0.3, 5);
        drivetrain.turnTo(Math.toRadians(-90));
    }

    @Override
    public void loop() {
        telemetry.addLine("Finished Moving!");
        telemetry.update();
        stop();
    }
}

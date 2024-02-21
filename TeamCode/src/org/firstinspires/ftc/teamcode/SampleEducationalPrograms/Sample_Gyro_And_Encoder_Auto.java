package org.firstinspires.ftc.teamcode.SampleEducationalPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.SampleEducationalPrograms.util.Drivetrain;

@Autonomous(name="Sample_Gyro_And_Encoder_Auto")
public class Sample_Gyro_And_Encoder_Auto extends OpMode {
    private Drivetrain drivetrain;
    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        drivetrain.moveForward(0.5, 20);
        drivetrain.turnTo(Math.toRadians(drivetrain.getHeading() + 90));
        drivetrain.moveForward(0.5, 20);
        drivetrain.turnTo(Math.toRadians(drivetrain.getHeading() + 90));
        drivetrain.moveForward(0.5, 20);
        drivetrain.turnTo(Math.toRadians(drivetrain.getHeading() + 90));
        drivetrain.moveForward(0.5, 20);
    }
}

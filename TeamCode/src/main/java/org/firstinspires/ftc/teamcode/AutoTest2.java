package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helpers.RobotControl;

/**
 * Created by Nathanael on 10/29/2016.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutoTest2", group="Linear Opmode")  // @Autonomous(...) is the other common choice

public class AutoTest2 extends LinearOpMode {

    //RobotControl robot;
    public ElapsedTime runtime;
    RobotControl robot;

    public void runOpMode() throws InterruptedException {

        robot = new RobotControl(this);
        robot.buttonPressEast.setPosition(230/255);
        /*robot.colorEast.setI2cAddress(I2cAddr.create8bit(0x6c));
        robot.colorEast.enableLed(true);*/
        telemetry.addData("Sensor", robot.colorEast.blue());
        telemetry.update();
        waitForStart();
        robot.runtime.reset();
        while (robot.runtime.milliseconds() < 10000) {
            //robot.drive(180,.5,robot.gyroRot());
            //robot.drive(35, 1, 0);
            if (!opModeIsActive()) return;
            telemetry.addData("Sensor", robot.colorEast.blue());
            telemetry.update();


            if (robot.colorEast.blue() >= 2) {
                if (!opModeIsActive()) return;
                robot.buttonPressEast.setPosition(230 / 255);
                telemetry.addData("Sensor", robot.colorEast.blue());
                telemetry.update();
            }


            }


        }


    }



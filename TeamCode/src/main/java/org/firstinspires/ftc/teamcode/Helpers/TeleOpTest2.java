/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.Helpers;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Helpers.RobotControl;




@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Teleop", group="Test")
@Disabled
public class TeleOpTest2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotControl robot = new RobotControl(this);
        robot.hood.setPosition(0.94);


        waitForStart();
        // run until the end of the mane = hardwareMap.dcMotor.get("ne");tch (driver presses STOP)
        double hoodPos = 0;
        double speed = 1;
        double ShooterRPM = 1000;
        double distanceTiles = 6850;
        boolean Endgame = false;
        while (opModeIsActive()) {
            /*if (gamepad1.dpad_up) {
                robot.capBallG.setPower(1);
            } else if (gamepad1.dpad_down) {
                robot.capBallG.setPower(-1);
            } else {
                robot.capBallG.setPower(0);
            }*/
            if (gamepad1.b) {
                speed = 0.625;
            } else if (gamepad1.x) {
                speed = 1;
            } else if (gamepad1.y) {
                speed = 0.5;
            }
            if (gamepad1.back) {
                Endgame = true;
            }

            if (gamepad2.y){
                robot.startFlyWheel(-1);
            }
            //Gamepad 1 will control the movement and harvester.
            robot.setMotors((float) (gamepad1.left_stick_x), (float) (gamepad1.left_stick_y*speed), (float) (gamepad1.right_stick_x*speed));
            if(gamepad1.left_trigger > 0.1){
                robot.harvester.setPower(-1);
            } else if (gamepad1.left_bumper){
                robot.harvester.setPower(1);
            }else{
                robot.harvester.setPower(0);
            }
            //Gamepad 2 will control shooter mechanisms, like flywheel, ball lift and aiming lid servo

             if (gamepad1.right_trigger >= 0.1){
                 robot.flyWheelWest.setPower(0);
                 robot.flyWheelEast.setPower(0);
             } else if (gamepad2.left_bumper) {

                 robot.flyWheelWest.setPower(0);
                 robot.flyWheelEast.setPower(0);
             } else if (gamepad2.y) {
                 robot.flyWheelWest.setPower(-0.25);
                 robot.ballFeeder.setPower(-0.25);
             } else {
                 robot.startFlyWheel(ShooterRPM);
             }
            if(gamepad2.left_trigger > 0.1 || (robot.SaddleSwitch.getVoltage() < .1 && !(gamepad2.right_trigger > 0.1))){
                robot.ballFeeder.setPower(-1);
            }
            else if(gamepad2.right_trigger > 0.1){
                robot.ballFeeder.setPower(1);
            }
            else{
                robot.ballFeeder.setPower(0);
            }
            if(gamepad2.dpad_up){
                hoodPos += .005;
                hoodPos = Range.clip(hoodPos,.4-.34,1);
                robot.hood.setPosition(hoodPos);
            }
            else if(gamepad2.dpad_down){
                hoodPos -= .005;
                hoodPos = Range.clip(hoodPos,.4-.34,1);
                robot.hood.setPosition(hoodPos);
            }
            if (gamepad2.dpad_left) {
                distanceTiles -= 5;
                distanceTiles = Range.clip(distanceTiles,0,1000000);
            }  if (gamepad2.dpad_right) {
                distanceTiles += 5;
                distanceTiles = Range.clip(distanceTiles,0,1000000);
            }
            ShooterRPM = distanceTiles;
            if (Endgame) {
                if (gamepad1.right_trigger >= 0.1) {
                    robot.capBall.setPower(1);
                } else if (gamepad1.right_bumper) {
                    robot.capBall.setPower(-1);
                } else {
                    robot.capBall.setPower(0);
                }
                robot.liftStopEast.setPosition(1);
                robot.liftStopWest.setPosition(0);
            }

            telemetry.addData("Shooter RPM", ShooterRPM);
            telemetry.addData("Wheel Speed", speed);
            telemetry.addData("Servo Position", hoodPos-.34);
            telemetry.addData("Servo Press", robot.buttonPressWest.getPosition());
            telemetry.addData("Endgame", Endgame);

            telemetry.update();
            idle();
        }
    }
}

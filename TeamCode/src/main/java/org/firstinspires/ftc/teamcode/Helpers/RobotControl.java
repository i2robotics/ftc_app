package org.firstinspires.ftc.teamcode.Helpers;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;



import org.firstinspires.ftc.teamcode.Helpers.HW;
import org.firstinspires.ftc.teamcode.TeleOpTest1;

/**
 * Driving functions for a Mecanum drive train in autonomous.
 */
public class RobotControl {
    private final HardwareMap hardwareMap;
    public DcMotor ne, se, sw, nw, harvester, flyWheelEast, flyWheelWest;
    public ColorSensor colorEast, colorWest;
    public CRServo ballFeeder;
    public Servo hood, buttonPressEast;
    public AnalogInput eLineSensor,wLineSensor, wallSensor;
    public ElapsedTime runtime;
    public GyroSensor gyro;
    //wEopd;

    /**
     * Constructor
     *
     * @param hardwareMap from the OpMode. Used to access the hardware.
     */
    public RobotControl(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.ne = this.hardwareMap.dcMotor.get("ne");
        this.se = this.hardwareMap.dcMotor.get("se");
        this.sw = this.hardwareMap.dcMotor.get("sw");
        this.nw = this.hardwareMap.dcMotor.get("nw");
        this.flyWheelEast = this.hardwareMap.dcMotor.get("flyWheelEast");
        this.flyWheelWest = this.hardwareMap.dcMotor.get("flyWheelWest");
        this.harvester = this.hardwareMap.dcMotor.get("harvester");
        this.hood = this.hardwareMap.servo.get("hood");
        this.buttonPressEast = this.hardwareMap.servo.get("buttonPressEast");
        this.ballFeeder = this.hardwareMap.crservo.get("ballFeeder");
        this.colorEast = this.hardwareMap.colorSensor.get("colorEast");
        this.gyro = this.hardwareMap.gyroSensor.get("gyro");
//        this.colorWest = this.hardwareMap.colorSensor.get("colorWest");
        this.wallSensor = this.hardwareMap.analogInput.get("wallSensor");
        this.eLineSensor = this.hardwareMap.analogInput.get("eLineSensor");
        this.wLineSensor = this.hardwareMap.analogInput.get("wLineSensor");
        this.runtime = new ElapsedTime();
        gyro.calibrate();
        while (gyro.isCalibrating()) {
            this.colorEast.enableLed(false);
            buttonPressEast.setPosition(235 / 255);
        }
    }

//    public void drive()

    public void drive(double angle, double power, double rot) {
        setMotors((float)(power*Math.sin(Math.toRadians(angle))), (float)(power*-Math.cos(Math.toRadians(angle))), (float)rot);
    }
    public double scale(double scaled) {
        double scaler = 0.8;
        return (scaler*Math.pow(scaled, 3) + ( 1 - scaler) * scaled);
    }
    public void setHarvester(double pow) {
        harvester.setPower(pow);
    }

    public void setMotors(float x, float y, float rot) {
        double drive = (double) -y, strafe = (double) x, spin = (double) rot;
        double nePower, nwPower, sePower, swPower;
        nwPower = Range.clip(drive + strafe + spin, -1, 1);
        swPower = Range.clip(drive - strafe + spin, -1, 1);
        nePower = Range.clip(drive - strafe - spin, -1, 1);
        sePower = Range.clip(drive + strafe - spin, -1, 1);
        nwPower = scale(nwPower);
        nePower = -scale(nePower);
        swPower = scale(swPower);
        sePower = -scale(sePower);

        //from here on is just setting motor values
        ne.setPower(nePower);
        se.setPower(sePower);
        sw.setPower(swPower);
        nw.setPower(nwPower);

    }
    public void brake(){
        runtime.reset();
        ne.setPower(Range.clip(-ne.getPower()*100,-1,1));
        se.setPower(Range.clip(-se.getPower()*100,-1,1));
        nw.setPower(Range.clip(-nw.getPower()*100,-1,1));
        sw.setPower(Range.clip(-sw.getPower()*100,-1,1));
        while (runtime.milliseconds() <= 150) {
            //Just to add some extra time
        }
        ne.setPower(0);
        se.setPower(0);
        sw.setPower(0);
        nw.setPower(0);
    }
    public void stop(){
        ne.setPower(0);
        se.setPower(0);
        sw.setPower(0);
        nw.setPower(0);
    }
    public void startFlyWheel(float pow){
        flyWheelEast.setPower(-pow);
        flyWheelWest.setPower(pow);
    }
    public void beaconCheckBlue(){
        runtime.reset();
        while(runtime.milliseconds() < 1000){
            drive(95, .75, 0.05);
        }
        stop();
        while(wallSensor.getVoltage() > .7){
            drive(-90, .75, 0);
        }
        /*if(colorEast.blue() >=2){
            buttonPressEast.setPosition(235/255);
        }
        else{
            buttonPressEast.setPosition(155/255);
        }*/

    }
    public void beaconCheckBlue2(){
        runtime.reset();
        while(runtime.milliseconds() < 2000){
            drive(100, .75, 0.15);
        }
        stop();
        /*while(wallSensor.getVoltage() > .55){
            drive(-90, .75, 0);
        }*/
        /*if(colorEast.blue() >=2){
            buttonPressEast.setPosition(235/255);
        }
        else{
            buttonPressEast.setPosition(155/255);
        }*/

    }
    public void lineCheck(){

        while(eLineSensor.getVoltage() < 1.5 || wLineSensor.getVoltage()  < 1.5){
            if(eLineSensor.getVoltage() < 1.5){
                ne.setPower(-.1);
                se.setPower(-.1);
            }
            else{
                ne.setPower(0);
                se.setPower(0);
            }
            if(wLineSensor.getVoltage() < 1.5){
                nw.setPower(.1);
                sw.setPower(.1);
            }
            else {
                nw.setPower(0);
                sw.setPower(0);
            }
            //this.drive(.75,0,0);
        }
        /*while(this.wallSensor.getVoltage() > .15){
            drive(.5, -90, 0);
        }*/
        stop();


    }
    public double gyroRot(){
        if (gyro.getHeading() == 1 || gyro.getHeading() == 359 || gyro.getHeading() == 0) {
            return 0;
        } else {
            return 1 / (.1 * (gyro.getHeading() - 180));
        }
    }


//    public void beaconCheckWest(){
//        if(colorWest.alpha() <=11 || colorEast.alpha() >= 10){
//            //Swivel down relative to color sensor
//        }
//        else{
//            //Swivel up relative to color sensor
//        }
//    }





}

package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.Velocity;

/**
 * JimmyCentral (Autonomous)
 *
 * This is the main code for autonomous. Override main()
 * to add run specific instructions.
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous
@Disabled
public class TobiAuto extends SynchronousOpMode {

    // Hardware
    private DcMotor motor;
    private UltrasonicSensor ultra;

    /* motor constants */
    private final int ENCODER_CPR = 1120;
    private final double GEAR_RATIO = 1;
    private final double WHEEL_DIAMETER = 4.9;
    private final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;


    private double currentDistance = 1000.0;
    private long distanceTime = 0;
    private final double FILTER_FACTOR = 0.5;

    @Override
    protected void main() throws InterruptedException {

        // motors
        motor = hardwareMap.dcMotor.get("motor");

        // sensors
        ultra = hardwareMap.ultrasonicSensor.get("ultra");

        // run certain motors using encoders
        motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        while (true) {
            telemetry.addData("within 10cm?", withinDistance(10.0));
            telemetry.update();
            Thread.sleep(50);
        }

        /*
        driveAcc(50000, 22000, 1, .4);
        Thread.sleep(1000);
        */
    }


    public boolean withinDistance(double distance) {

        // update the current distance
        double newDistance = 0.0;
        for (int i = 0; i < 5; i++) {
            newDistance += ultra.getUltrasonicLevel();
        }
        newDistance = newDistance / 5;
        currentDistance = (currentDistance * (1.0 - FILTER_FACTOR)) + (newDistance * FILTER_FACTOR);
        System.out.println("currentDistance: " + currentDistance);

        if (currentDistance <= distance && distanceTime > System.currentTimeMillis()) {
            distanceTime = System.currentTimeMillis();
        } else if (currentDistance > distance){
            distanceTime = System.currentTimeMillis() + 10000000;
        }

        if (System.currentTimeMillis() > (distanceTime + 200)) {
            System.out.println("It's true!");
            return true;
        }

        return false;
    }


    /* convert inches (distance) to rotations (motor) */
    public int inchesToRotations(double distance) {
        double rotations = distance/CIRCUMFERENCE;
        return (int)(ENCODER_CPR * rotations * GEAR_RATIO);
    }



    /*  drive forward without error correction */
    public void driveAcc(double totalDist, double rampDist, double finalPower, double initPower ) throws InterruptedException {
        //int n = inchesToRotations(distance);
        int start = motor.getCurrentPosition();
        finalPower = Range.clip(finalPower, .2, 1);
        initPower = Range.clip(initPower, .2, 1);
        if(initPower > finalPower) initPower = finalPower;
        if(2*rampDist > totalDist) rampDist = totalDist / 2;

        motor.setPower(initPower);
        int motorPos = motor.getCurrentPosition();
        double newPower = 0;
        double oldPower = 0;

        while (motorPos < (start + totalDist)) {
            if(motorPos < (start + rampDist)) {
                double distThruRamp = ( (motorPos - start) ) / rampDist;
                newPower = initPower + (distThruRamp * (finalPower - initPower));
            } else if (motorPos < (start + totalDist - rampDist)) {
                newPower = finalPower;
            } else {
                double distThruRamp = (totalDist - (motorPos - start)) / rampDist;
                newPower = initPower + (distThruRamp * (finalPower - initPower));
                //newPower = 0.2;
            }
            Thread.sleep(20);

            newPower = Range.clip(newPower, initPower, finalPower);
            if (oldPower != newPower) {
                motor.setPower(newPower);
                oldPower = newPower;
                System.out.println("just set power to " + newPower);
            }
            motorPos = motor.getCurrentPosition();
            telemetry.addData("motorPos", motorPos);
            telemetry.addData("newPower", newPower);
            telemetry.update();
            this.idle();
        }

        motor.setPower(0.0);
    }

    /* drive back without error correction */
    public void driveBack(double power, double distance) throws InterruptedException {
        int n = inchesToRotations(distance);
        int start = motor.getCurrentPosition();

        motor.setPower(-power);

        while (motor.getCurrentPosition() > (start - n)) {
            Thread.sleep(10);
        }

        motor.setPower(0.0);
    }
}
package org.usfirst.ftc.aperturescience;

/**
 * AutoPeopleNormal_RED (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous(name="RED Mountain Man Jimmy", group="Red")
public class AutoRedPosition1 extends JimmyCentral {

    private final double THE_DISTANCE = 31;

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // alliance specific code
        //while (opModeIsActive()) {

        // drive back 20 inches
        /*
        driveBack(.3, 20);
        Thread.sleep(1000);

        // turn left 40 degrees
        turnGyro(-40);
        Thread.sleep(200);
        */

        // pull out the climber arm
        setRedArm();

        // pull out the tapes
        setTapes();

        // drive back 69 inches or until we find the white line
        boolean foundWhite = driveBackToWhite(.5, 87);
        Thread.sleep(200);

        double distance = 0.0;
        if (foundWhite) {   // yeah! found the white line

            // back up 4.5 inches
            driveBack(.3, 4.5);
            Thread.sleep(1000);

            // turn left 55 degrees
            turnGyro(-55);
            Thread.sleep(200);

            // drive back to wall
            distance = driveBackDistance(.2, 6, THE_DISTANCE);

        } else {
            // did not find white line -- let's try to correct

            // drive forward 4.5 inches
            drive(.3, 4.5);

            // turn left 57 degrees
            turnGyro(-57);
            Thread.sleep(500);

            // drive back to wall
            distance = driveBackDistance(.2, 6, THE_DISTANCE);
        }

        autoArmUp();

        // drive out
        sweeperOn();

        drive(.3, 12);

        autoArmDown();

        sweeperOff();

        driveBack(.5, 12);

        // drop the climbers
        //autoArmNoJimmy();

        //stop();

        // error checking
            /*
            Thread.sleep(1000);
            telemetry.addData("Final Heading", getHeading());
            telemetry.update();
            */

        // we're done!
        //break;
        //}
    }
}

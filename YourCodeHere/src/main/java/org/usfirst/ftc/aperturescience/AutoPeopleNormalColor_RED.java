package org.usfirst.ftc.aperturescience;

/**
 * AutoPeopleNormal_RED (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous
public class AutoPeopleNormalColor_RED extends AutoPeopleNormal {

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // alliance specific code
        //while (opModeIsActive()) {

        // drive back 20 inches
        driveBack(.3, 20);
        Thread.sleep(1000);

        // turn left 40 degrees
        turnGyro(-40);
        Thread.sleep(200);

        // pull out the climber arm
        setRedArm();

        // drive back 69 inches or until we find the white line
        boolean foundWhite = driveBackToWhite(.4, 69);
        Thread.sleep(200);

        if (foundWhite) {   // yeah! found the white line

            // back up 4.5 inches
            driveBack(.1, 4.5);
            Thread.sleep(1000);

            // turn left 50 degrees
            turnGyro(-50);
            Thread.sleep(200);

            // drive away from wall looking for red
            boolean foundRed = driveToRed(.2, 18);
            Thread.sleep(500);

            // drive back 14.5 inches
            if (!foundRed) {
                driveBack(.2, 23);
                Thread.sleep(500);
            }

            if (foundRed){
                driveBack(.2, 14.5);
                Thread.sleep(200);
            }

        } else {
            // did not find white line -- let's try to correct

            // drive forward 4.5 inches
            drive(.1, 4.5);

            // turn left 50 degrees
            turnGyro(-50);
            Thread.sleep(200);

            // drive away from wall looking for red line
            boolean foundRed = driveToRed(.2, 12);
            Thread.sleep(500);

            // missed it!
            if (!foundRed) {
                // drive back looking for red or white
                driveBack(.2, 23);
                Thread.sleep(500);
            }

            // found it! drive back 14.5 inches
            if (foundRed) {
                driveBack(.2, 14.5);
                Thread.sleep(200);
            }
        }

        // drop the climbers
        autoArm();

        stop();

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

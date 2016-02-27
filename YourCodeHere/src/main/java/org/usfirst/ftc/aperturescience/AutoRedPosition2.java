package org.usfirst.ftc.aperturescience;

/**
 * AutoPeopleNormal_RED (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
//@org.swerverobotics.library.interfaces.Autonomous
@org.swerverobotics.library.interfaces.Autonomous(name="RED Beach Bum Jimmy", group="Red")
public class AutoRedPosition2 extends JimmyCentral {

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        // alliance specific code
        //while (opModeIsActive()) {


        // pull out the climber arm
        setRedArm();

        // pull out the tapes
        setTapes();

        // drive back 69 inches or until we find the white line
        boolean foundWhite = driveBackToWhite(.4, 99);
        Thread.sleep(200);

        if (foundWhite) {   // yeah! found the white line

            // back up 4.5 inches
            driveBack(.1, 4.5);
            Thread.sleep(1000);

            // turn left 50 degrees
            turnGyro(-47);
            Thread.sleep(200);

            // drive back to wall
            driveBackDistance(.2,10,7);

        } else {
            // did not find white line -- let's try to correct

            // drive forward 4.5 inches
            drive(.1, 4.5);

            // turn left 50 degrees
            turnGyro(-47);
            Thread.sleep(200);

            // drive back to wall
            driveBackDistance(.2, 10, 7);
        }

        // drop the climbers
        autoArmNoJimmy();

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

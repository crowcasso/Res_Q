package org.usfirst.ftc.aperturescience.test;

import org.usfirst.ftc.aperturescience.AutoCommon;

/**
 * AutoPeopleNormal_RED (Autonomous)
 *
 * @author FTC 5064 Aperture Science
 */
@org.swerverobotics.library.interfaces.Autonomous(name="RED Mountain Man Drunk Jimmy", group="Red")
public class MountainMan_Drunk_Jimmy_RED extends AutoCommon {

    @Override
    public void main() throws InterruptedException {

        // everything up to waitForStart
        super.main();

        driveAcc(60,15,1.0,0.2);
        Thread.sleep(1000);

    }
}

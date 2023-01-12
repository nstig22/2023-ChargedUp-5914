/////////////////////////////////////////////////////////////////////
//  File:  Drive.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  Setup for Mecanum drive using the swerve drives on
//            all four corners.  This serves mostly as an example
//            of how to encapsulate the various components and
//            functions.
//
//  Inception Date: 1/10/2023
//
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////

package frc.robot;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Drive  {

    //  Need to assign 8 CAN addresses. 
    int frontLeft_driveID=1;
    int frontRight_driveID=2;
    int rearLeft_driveID=3;
    int rearRight_driveID=4;

    int frontLeft_turnID=5;
    int frontRight_turnID=6;
    int rearLeft_turnID=7;
    int rearRight_turnID=8;

    SwerveDrive frontLeft;
    SwerveDrive frontRight;
    SwerveDrive rearLeft;
    SwerveDrive rearRight;

    MecanumDrive robotDrive;

    private Delay delay;

    ADXRS450_Gyro driveGyro;
    // Initial drive gyro angle.
    public double initDriveGyroAngle;

    //  Default Constructor
    Drive()
    {
        
        //  Create 4 SwerveDrive objects
        frontLeft=new SwerveDrive(frontLeft_turnID,frontLeft_driveID);
        frontRight=new SwerveDrive(frontRight_turnID,frontRight_driveID);
        rearLeft=new SwerveDrive(rearLeft_turnID,rearLeft_driveID);
        rearRight=new SwerveDrive(rearRight_turnID,rearRight_driveID);

        //  Will need to invert 2 of the drives, not sure which of
        //  the motors needs to be inverted
        frontRight.falcon_drive.setInverted(false);
        frontLeft.falcon_drive.setInverted(false);
        rearRight.falcon_drive.setInverted(true);
        rearLeft.falcon_drive.setInverted(true);

        //  A similar logic for the turn motors - not sure at this point
        //  which motors need to be inverted
        frontRight.falcon_turn.setInverted(false);
        frontLeft.falcon_turn.setInverted(false);
        rearRight.falcon_turn.setInverted(true);
        rearLeft.falcon_turn.setInverted(true);
 
        // Create the MecanumDrive
        robotDrive = new MecanumDrive(frontLeft.falcon_drive, rearLeft.falcon_drive,
         frontRight.falcon_drive, rearRight.falcon_drive);

         delay=new Delay();

         //  We assume at this point that the wheels are at zero angle.
         //  All encoders are zeroed.

         //  Drive encoders
        frontRight.falcon_drive.setSelectedSensorPosition(0.0);
        frontLeft.falcon_drive.setSelectedSensorPosition(0.0);
        rearRight.falcon_drive.setSelectedSensorPosition(0.0);
        rearLeft.falcon_drive.setSelectedSensorPosition(0.0);

        //  Turn encoders
         frontRight.falcon_turn.setSelectedSensorPosition(0.0);
         frontLeft.falcon_turn.setSelectedSensorPosition(0.0);
         rearRight.falcon_turn.setSelectedSensorPosition(0.0);
         rearLeft.falcon_turn.setSelectedSensorPosition(0.0);

        // Gyro for auto drive functions.
        driveGyro = new ADXRS450_Gyro(); 

        //  Setup the initial heading as zero after calibration
        initDriveGyroAngle = driveGyro.getAngle();
        driveGyro.calibrate();
        driveGyro.reset();
    }


    /////////////////////////////////////////////////////////////////
    //  Function:  driveFwd_Mecanum(double distance)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose: An attempt to use the Mecanum drive mechanism
    //  to drive the robot straight forward.
    //
    //  Arguments:Accepts a double representing the desired
    //  travel in inches.
    //
    //  Returns:the error between position and target in counts.
    //
    //  Remarks:  Note the use of the while() loop for the first
    //  part of the movement.  Not sure how this works with the
    //  watchdog so that may have to be defeated.  Once the 
    //  while() loop completes if/else is used to detect a slowing
    //  of the motor and ultimately stopping the movement.  This
    //  would need to be massaged to stop at the right point.
    //  One assumes that turn motors are zeroed or that it is 
    //  intentional to "crab" the movement.  If the turn motors
    //  are set to 90 degress the movement would become 'x'
    //  movement.e.g. sideways.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double driveFwd_Mecanum(double distance)
    {
        double error=0.0;
        double counts=0.0;
        double position;
        double target;

        //  Use the leftFRont drive encoder to determine distance
        counts=frontLeft.compute_countsDistance(distance);
        position=frontLeft.falcon_drive.getSelectedSensorPosition();

        target=position+counts;

        error=target-position;

        while(error>5*frontLeft.drive_deadband)
        {
            robotDrive.driveCartesian(0.0,0.5,0.0);
            delay.delay_milliseconds(10);
            position=frontLeft.falcon_drive.getSelectedSensorPosition();
            error=target-position;
        } 
        if((error<5*frontLeft.drive_deadband)&&(error>0.0))
        {
            robotDrive.driveCartesian(0.0,0.2,0.0);
            delay.delay_milliseconds(10);
            position=frontLeft.falcon_drive.getSelectedSensorPosition();
            error=target-position;
        }  else if((error<frontLeft.drive_deadband)||error<0.0) {
            robotDrive.driveCartesian(0.0,0.0,0.0);
            delay.delay_milliseconds(10);
            position=frontLeft.falcon_drive.getSelectedSensorPosition();
            error=target-position;        

        }
         
        return(error);

    }

     ///////////////////////////////////////////////////////////////////////////
    // Function: void moveFwd(double speed,double target)
    ///////////////////////////////////////////////////////////////////////////
    //
    // Purpose: Uses on_board gyro to drive the robot straight forward
    // given a target angle as an argument.
    //
    // Arguments: double speed. Must be between -1.0 and 1.0.
    // double heading - the target angle.
    //
    // Returns: void
    //
    // Remarks: 1/11/2023:  Resurrected, not sure if it will work.
    //
    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    public void moveFwd(double speed, double heading) {

        double corr = 0.2;
        double angle = 0;
        double delta; // The difference between the target and measured angle

        angle = driveGyro.getAngle();

        delta = angle - heading;

        robotDrive.driveCartesian(0, speed, -corr * delta);


    }


    /////////////////////////////////////////////////////////////////
    //  Function:
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////
    //  Function:
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    
}

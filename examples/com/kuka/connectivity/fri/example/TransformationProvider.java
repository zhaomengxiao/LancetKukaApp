package com.kuka.connectivity.fri.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.World;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;

/**
 * Creates a FRI Session.
 */
public class TransformationProvider extends RoboticsAPIApplication
{
    private Controller _lbrController;
    private LBR _lbr;
    private String _clientName;
    private boolean on  = true;
    
    
    // Tool Data
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

   
    
    @Override
    public void initialize()
    {
        _lbrController = (Controller) getContext().getControllers().toArray()[0];
        _lbr = (LBR) _lbrController.getDevices().toArray()[0];
        // **********************************************************************
        // *** change next line to the FRIClient's IP address                 ***
        // **********************************************************************
        _clientName = "172.31.1.148";
        
        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        _loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(_lbr.getFlange());

    }

    @Override
    public void run()
    {
    	// move to start pose
        _lbr.move(ptp(.0, .0, .0, Math.toRadians(90), .0, .0, .0));
        //Create some frames
        Frame probeFrame = new Frame(_lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame())); //change by client
       
        getLogger().info(probeFrame.toStringInWorld());
        //Use SmartServo
        boolean doDebugPrints = true;

        SmartServo aSmartServoMotion = new SmartServo(
                _lbr.getCurrentJointPosition());

        aSmartServoMotion.useTrace(true);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(5e-3);

        getLogger().info("Starting SmartServo motion in position control mode");
        
        //move use smartServo
        _toolAttachedToLBR.moveAsync(aSmartServoMotion);

        getLogger().info("Get the runtime of the SmartServo motion");
        ISmartServoRuntime theServoRuntime = aSmartServoMotion
                .getRuntime();

        //Frame aFrame = theServoRuntime.getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());
        Frame aFrame = _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame());

        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);

        // Send period from LBR to client
        friConfiguration.setSendPeriodMilliSec(10);

        // Send period multiply with integer gives the receive period from client to robot controller. 
        // In this example it is 30 milliseconds.
        friConfiguration.setReceiveMultiplier(3);

        // Select the frame from the scene graph whose transformation is changed by the client application.
        friConfiguration.registerTransformationProvider("PBase", probeFrame);

        getLogger().info("Creating FRI connection to " + friConfiguration.getHostName());
        getLogger().info("SendPeriod: " + friConfiguration.getSendPeriodMilliSec() + "ms |"
                + " ReceiveMultiplier: " + friConfiguration.getReceiveMultiplier());

        FRISession friSession = new FRISession(friConfiguration);

        try
        {
            friSession.await(10, TimeUnit.SECONDS);
        }
        catch (TimeoutException e)
        {
            getLogger().error(e.getLocalizedMessage());
            friSession.close();
            return;
        }

        getLogger().info("FRI connection established.");
     

        // excute loop
     // do a cyclic loop
        // Do some timing...
        // in nanosec
        long startTimeStamp = System.nanoTime();
        while (on)
        {
            // Insert your code here
            // e.g Visual Servoing or the like
        	// compute a new commanded position
            Frame destFrame = aFrame.copyWithRedundancy();
            destFrame.setX(probeFrame.getX());
            destFrame.setY(probeFrame.getY());
            destFrame.setZ(probeFrame.getZ());
        	
            // Synchronize with the realtime system
            theServoRuntime.updateWithRealtimeSystem();

            // Get the measured position 
            Frame msrPose = theServoRuntime
                    .getCurrentCartesianDestination(_toolAttachedToLBR.getDefaultMotionFrame());

//            if (doDebugPrints)
//            {
//                getLogger().info("Current cartesian goal " + aFrame);
//                getLogger().info("Current joint destination "
//                        + theServoRuntime.getCurrentJointDestination());
//            }

            // Do some Computation
            // emulate some computational effort - or waiting for external
            // stuff
            ThreadUtil.milliSleep(1000);
            if (doDebugPrints)
            {
                getLogger().info("New cartesian goal " + destFrame);
                getLogger().info("LBR position "
                        + _lbr.getCurrentCartesianPosition(_toolAttachedToLBR.getDefaultMotionFrame()).toStringInWorld());
                getLogger().info("Measured cartesian pose from runtime "
                        + msrPose);

//                if ((i % 100) == 0)
//                {
//                    // Some internal values, which can be displayed
//                    getLogger().info("Simple cartesian test " + theServoRuntime.toString());
//                }
            }
//            try
//            {
//                theServoRuntime.setDestination(destFrame);
//            }
//            catch (Exception e)
//            {
//                getLogger().warn(e.toString());
//                //e.printStackTrace();
//            }
        }
        
        // done
        //Print statistics and parameters of the motion
        getLogger().info("Simple cartesian test " + theServoRuntime.toString());

        getLogger().info("Stop the SmartServo motion");
        theServoRuntime.stopMotion();
        
        friSession.close();
    }

    /**
     * main.
     * 
     * @param args
     *            args
     */
    public static void main(final String[] args)
    {
        final TransformationProvider app = new TransformationProvider();
        app.runApplication();
    }

}

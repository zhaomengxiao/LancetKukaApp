package com.kuka.connectivity.fri.example;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.fastRobotInterface.FRIConfiguration;
import com.kuka.connectivity.fastRobotInterface.FRISession;
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
    private static final double[] TRANSLATION_OF_TOOL = { 200, 200, 200 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 200, 200, 200 };

   
    
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
        //Create some frames
        Frame objectBase = new Frame(World.Current.getRootFrame());
        Frame objectTip = new Frame(objectBase);
        objectTip.transform(Transformation.ofDeg(10.0, 10.0, 10.0, 45.0, 45.0, 45.0));

        // configure and start FRI session
        FRIConfiguration friConfiguration = FRIConfiguration.createRemoteConfiguration(_lbr, _clientName);

        // Send period from LBR to client
        friConfiguration.setSendPeriodMilliSec(10);

        // Send period multiply with integer gives the receive period from client to robot controller. 
        // In this example it is 30 milliseconds.
        friConfiguration.setReceiveMultiplier(3);

        // Select the frame from the scene graph whose transformation is changed by the client application.
        friConfiguration.registerTransformationProvider("PBase", objectBase);

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
     // move to start pose
        _lbr.move(ptp(.0, .0, .0, Math.toRadians(90), .0, .0, .0));

        // Output
        getLogger().info("Transformation from World of");
        while (on)
        {
            //ThreadUtil.milliSleep(150);
        	try{
        		_toolAttachedToLBR.move(ptp(objectBase).setJointVelocityRel(0.2));
        	}
        	catch (IllegalStateException e){
        		getLogger().error(e.getLocalizedMessage());
        	}
        	
            // async move with overlay ...
//            _lbr.moveAsync(ptp(Math.toRadians(-90), .0, .0, Math.toRadians(90), .0, Math.toRadians(-90), .0)
//                    .setJointVelocityRel(0.2)
//                    .addMotionOverlay(jointOverlay)
//                    .setBlendingRel(0.1)
//                    );
//            getLogger().info("Frame objectBase:\n" + objectBase.toStringInWorld());
//            getLogger().info("Frame objectTip:\n" + objectTip.toStringInWorld());
        }

        // done
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

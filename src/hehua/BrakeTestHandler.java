package hehua;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import com.kuka.med.controllerModel.MedController;
import com.kuka.med.cyclicBrakeTest.BrakeTestDiskSpaceState;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitor;
import com.kuka.med.cyclicBrakeTest.BrakeTestMonitorEvent;
import com.kuka.med.cyclicBrakeTest.BrakeTestOutcome;
import com.kuka.med.cyclicBrakeTest.CyclicMonitoringState;
import com.kuka.med.cyclicBrakeTest.IBrakeTestMonitorListener;
import com.kuka.med.cyclicBrakeTest.OverallBrakeTestResult;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.roboticsAPI.uiModel.IApplicationUI;
import com.kuka.task.ITaskLogger;

/**
 * Implementation of a sample application handling the cyclic brake test. As soon as the application is started, the
 * user is asked to manage the initial state of the cyclic monitoring, e.g. start the brake test execution or continue
 * with the application. Then, a continuous motion of the robot between two joint configurations is started to simulate
 * the normal application workflow. The IBrakeTestMonitorListener is used to receive events about state changes of the
 * cyclic brake test monitoring and react accordingly.
 */
public class BrakeTestHandler implements IBrakeTestMonitorListener
{

    /**
     * User action for interaction with application workflow.
     * */
    private enum UserAction
    {
        NO_ACTION,
        START_BRAKETEST,
        POSTPONE_BRAKETEST,
        EXIT_APPLICATION;
    }

    private MedController _controller = null;

    
    private ITaskLogger _logger;
    
    private IApplicationUI appUi = null;

    private LBRMed _lbr;

    private BrakeTestMonitor _brakeTestMonitor;
    private CyclicMonitoringState _currentStatus = CyclicMonitoringState.SYSTEM_STARTED;
    private boolean _isPostponementAcceptable = false;
    private boolean _isBrakeTestExecutable = true;
    private BrakeTestDiskSpaceState _diskStatus = BrakeTestDiskSpaceState.UNKNOWN;

    private int _responseTime = 65;
    private double _vel = 0.2;
    private JointPosition _jointPos;
    private IMotionContainer _mc;

    /** User commanded action in the application dataflow. */
    private UserAction _action = UserAction.NO_ACTION;

    /** Additional thread to handle BrakeTestMonitorEvents with dialog boxes for user feedback. */
    private ExecutorService _workerToHandleEvent = Executors.newSingleThreadExecutor();

    public BrakeTestHandler(LBRMed rb, ITaskLogger logger, IApplicationUI ui)
    {
    	_lbr = rb;
    	appUi = ui;
    	_logger = logger;
    	_controller = (MedController)rb.getController();
    	
    	initialize();
    }
    
    public void initialize()
    {
        _controller = (MedController) _lbr.getController();
        _brakeTestMonitor = new BrakeTestMonitor(_controller);
        _brakeTestMonitor.addMonitoringListener(this);

        _responseTime = _brakeTestMonitor.setResponseTime(_responseTime);
        _logger.info("response time changed to " + _responseTime);
        _logger.info("cur state " + _brakeTestMonitor.getCurrentState());

        _jointPos = new JointPosition(0, 0, 0, Math.toRadians(90), 0, 0, 0);
        _brakeTestMonitor.setInitialPositionForBrakeTest(_jointPos, _lbr);
        
    }

    public ProtocolResult run()
    {
        // check the current status of the cyclic brake test because an event could have been missed due to
        // late start of the application
        _currentStatus = _brakeTestMonitor.getCurrentState();
        _logger.info("current state: " + _currentStatus);
        _diskStatus = _brakeTestMonitor.checkFreeDiskSpace();
        _logger.info("current disk space state: " + _diskStatus);
        // check whether a postponement is possible
        _isPostponementAcceptable = _brakeTestMonitor.isPostponementPossible();
        _isBrakeTestExecutable = _brakeTestMonitor.isBrakeTestExecutable();

        // handle the brake test execution at start of the application
       //  handleInitialBraketestExecution();
        ProtocolResult ret = new ProtocolResult();
		ret.setOperateType("Brake");
		ret.setResultMsg("Brake");
        if(_isBrakeTestExecutable)
        {
            BrakeTestOutcome outcome = _brakeTestMonitor.executeBrakeTest(_lbr);
            _logger.info("The brake test result is " + outcome.getOverallResult().toString());
            if (_brakeTestMonitor.getCurrentState().equals(CyclicMonitoringState.OK))
            {
            	 _logger.info("brake test is done,infomation is" + outcome.getOverallResult().toString());
            }
            int code = (outcome.getOverallResult() == OverallBrakeTestResult.SUCCESSFUL) ? 0 : -1;       
    		ret.setResultCode(code);	
    		
        }
		ret.setResultCode(-1);	
		return ret;
        
    }

    /**
     * Implementation of the callback method to listen for monitor state events. An additional single thread is required
     * to use dialog boxes for user feedback without blocking the listener callback.
     * */
    @Override
    
    public void onMonitoringStateChanged(BrakeTestMonitorEvent event)
    {
        if (!event.getCurrentCyclicMonitorState().equals(_currentStatus))
        {
            _currentStatus = event.getCurrentCyclicMonitorState();
            _isPostponementAcceptable = event.isPostponementAcceptable();
            _isBrakeTestExecutable = event.isBrakeTestExecutable();
            _diskStatus = event.getStateOfFreeDiskSpace();

            _logger.info("BrakeTestState changed to " + _currentStatus);

            // Run the handling of the event with dialog boxes in an additional Thread
            _workerToHandleEvent.execute(new Runnable()
            {
                @Override
                public void run()
                {
                    handleCyclicMonitoring();
                }
            });
        }
    }
    
    public boolean isBrakeTestOK() {
    	
    	return _currentStatus == CyclicMonitoringState.OK;
    	
    }
    private void logInfo()
    {
    	System.out.println("cur state " + _currentStatus.toString() + " " + _isPostponementAcceptable + " " + _isBrakeTestExecutable + " " + _diskStatus.toString());
    }

    private void handleInitialBraketestExecution()
    {
        String text = "";
        if (_isBrakeTestExecutable)
        {
         //   text = "Do you want to start a brake test execution now?";
         //   if (0 == appUi.displayModalDialog(ApplicationDialogType.QUESTION, text, "Yes", "No"))
            {
                _action = UserAction.START_BRAKETEST;
            }
        }
        else
        {
            text = "The brake test is not executable! The system is in " + _currentStatus.toString() + " state. "
                    + (_currentStatus.equals(CyclicMonitoringState.FATAL_ERROR) ? "All robot motions from the "
                            + "application are locked. Please contact customer support!" : "");
            if (0 == appUi.displayModalDialog(ApplicationDialogType.ERROR, text, "Exit application"))
            {
                // terminate application
                _action = UserAction.EXIT_APPLICATION;
            }
        }
    }

    private void handleCyclicMonitoring()
    {
        String text = "";
        if (0 < _diskStatus.compareTo(BrakeTestDiskSpaceState.ENOUGH_SPACE))
        {
            text = "The state of the free disk space is " + _diskStatus + ". What would you like to do?";
            if (0 != appUi.displayModalDialog(ApplicationDialogType.QUESTION, text, "continue"))
            {
                _logger.error("That should not be possible");
            }
        }

        if (_currentStatus.equals(CyclicMonitoringState.FEEDBACK_REQUIRED))
        {
            handleFeedbackRequiredState();
        }
        else if (_currentStatus.equals(CyclicMonitoringState.ERROR))
        {
            handleErrorState();
        }
        else if (_currentStatus.equals(CyclicMonitoringState.FATAL_ERROR))
        {
            text = "The system is in FATAL_ERROR state. All robot motions from the application are locked. Please "
                    + "contact customer support!";
            if (0 == appUi.displayModalDialog(ApplicationDialogType.ERROR, text, "Exit application"))
            {
                // terminate application
                _action = UserAction.EXIT_APPLICATION;
            }
        }
    }

    private void handleErrorState()
    {
        String text = "";
        if (_diskStatus.equals(BrakeTestDiskSpaceState.NOT_ENOUGH_SPACE))
        {
            text = "A brake test is overdue, but the disk space is not enough! Please free disk space on hard "
                    + "disk or contact Service Support!";
            if (0 == appUi.displayModalDialog(ApplicationDialogType.ERROR, text, "Exit application"))
            {
                // terminate application
                _action = UserAction.EXIT_APPLICATION;
            }
        }
        else
        {
            text = "A brake test is overdue! The robot can only be used after a successful brake test. "
                    + "Do you want to start it now?";
            if (0 == appUi.displayModalDialog(ApplicationDialogType.ERROR, text, "Yes"))
            {
                _action = UserAction.START_BRAKETEST;
            }
        }
    }

    private void handleFeedbackRequiredState()
    {
        boolean postponeUnacceptable = false;
        String text = "";
        text = "A brake test is due in " + _brakeTestMonitor.getTimeTillBraketestOverdue() + "seconds. "
                + "What would you like to do?";
        int response = appUi.displayModalDialog(ApplicationDialogType.QUESTION,
                text, "start", "postpone");
        if (0 == response)
        {
            _action = UserAction.START_BRAKETEST;
        }
        else if (1 == response)
        {
            _action = UserAction.POSTPONE_BRAKETEST;
            if (_isPostponementAcceptable)
            {
                _brakeTestMonitor.postponeBrakeTest();
            }
            else
            {
                postponeUnacceptable = true;
            }
        }
        else
        {
            _logger.error("That should not be possible (2)");
        }
        if (postponeUnacceptable)
        {
//            text = "A postponement is not possible anymore! Do you want to start the brake test now?";
//            if (0 == appUi.displayModalDialog(ApplicationDialogType.QUESTION,
//                    text, "Yes"))
//            {
                _action = UserAction.START_BRAKETEST;
//            }
        }
    }

    //@Override
    public void dispose()
    {
        _brakeTestMonitor.removeMonitoringListener(this);

        // Dispose the additional single thread used to collect user feedback with dialog boxes.
        if (_workerToHandleEvent != null)
        {
            try
            {
                _workerToHandleEvent.shutdown();
                _workerToHandleEvent.awaitTermination(5, TimeUnit.SECONDS);
            }
            catch (InterruptedException e)
            {
                _logger.warn("Interruption in shutting down the event worker!", e);
            }
            _workerToHandleEvent.shutdownNow();
        }

        //super.dispose();
    }
}

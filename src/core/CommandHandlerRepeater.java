package core;

import application.TCPServerSendDataApplication;
import factorys.CommandFactory;
import factorys.CommandParameterFactory;
import units.AbstractCommand;
import units.AbstractCommandParameter;

public class CommandHandlerRepeater {

    public CommandFactory commandFactory = new CommandFactory();
    public CommandParameterFactory commandParameterFactory = new CommandParameterFactory();

    public boolean PushCommandToHandler(String commandStreamString) {

    	if(TCPServerSendDataApplication.isDebug) {
    		System.out.println("[INFO] Try to parse and process the target command" + commandStreamString);
    	}
        String product = this.AnalysisCommandNameOfString(commandStreamString);
        // Make command object.
        AbstractCommand abstractCommand = commandFactory.MakeProduct(product);

    	if(TCPServerSendDataApplication.isDebug) {
    		System.out.println("[INFO] Attempt to create a processing object for the target command; string " + product);
    	}
        if(null != abstractCommand) {
        	if(TCPServerSendDataApplication.isDebug) {
        		System.out.println("[INFO] Successfully created the processing object of the target command; string " + product);
        	}
            AbstractCommandParameter abstractCommandParameter = commandParameterFactory.MakeParameter(abstractCommand.GetNameString(), commandStreamString);
            if(null != abstractCommandParameter) {

            	if(TCPServerSendDataApplication.isDebug) {
            		System.out.println("[INFO] Successfully created the processing parameter object of the target command; string " + product);
            	}
            	abstractCommandParameter.SetInputString(commandStreamString);
                abstractCommand.SetParameterObject(abstractCommandParameter);
            }
            abstractCommand.Execute(abstractCommandParameter);
            // TODO: The execution return results need to be fed back to the upper computer.
            return true;
        } else {
        	if(TCPServerSendDataApplication.isDebug) {
        		System.out.println("[WARNING] Failed to create the processing object of the target command; string " + product);
        	}
        }

        return false;
    }

    /**
     * @param command
     * @return
     */
    private String AnalysisCommandNameOfString(String command) {
        String[] commandSplit = command.split(",");

        if(0 < commandSplit.length) {
            return commandSplit[0];
        }
        return "Non";
    }
}

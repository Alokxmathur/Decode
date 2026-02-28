package org.firstinspires.ftc.teamcode.operations;

import java.util.Date;

/**
 * Created by Silver Titans on 10/29/17.
 *
 * This class defines operations that the robot can perform.
 *
 * An enumeration called TYPE lists the types of operations that can be used.
 */

public abstract class Operation {
    protected boolean operationIsBeingProcessed = false;

    public boolean isAborted() {
        return isAborted;
    }

    public void setAborted(boolean aborted) {
        isAborted = aborted;
    }

    protected boolean isAborted = false;

    private Date startTime;
    String title;
    public String getTitle() {
        return title;
    }
    public boolean getOperationIsBeingProcessed() {
        return this.operationIsBeingProcessed;
    }

    public void setOperationBeingProcessed() {
        this.operationIsBeingProcessed = true;
        this.startTime = new Date();
    }

    public Date getStartTime() {
        return this.startTime;
    }

    public abstract boolean isComplete();
    public abstract void startOperation();
    public abstract void abortOperation();
}

package org.firstinspires.ftc.teamcode.operations;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Match;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.Date;

/**
 * Created by alokmathur on 10/29/17.
 */

public class OperationThread extends Thread {
    private final Object threadLock = new Object();
    private final String title;
    //stack of operationsQueue to perform
    private final ArrayList<Operation> operationsQueue = new ArrayList<>();
    private final LinearOpMode opMode;

    public OperationThread(Robot robot, String title, LinearOpMode opMode) {
        this.opMode = opMode;
        this.title = title + " Operation Thread";
        Match.log(title + " created");
    }

    public void run() {
        Match.log(title + " started");
        while (opMode.opModeIsActive()) {
            synchronized (threadLock) {
                //if we have performed the operation successfully,
                // we need to remove our current operation
                if (!this.operationsQueue.isEmpty()) {
                    Operation operation = this.operationsQueue.get(0);
                    if (operation.getOperationIsBeingProcessed()) {
                        if (operation.isAborted()) {
                            Match.log(title + ": Aborted operation: " + operation
                                    + " in " + (new Date().getTime() - operation.getStartTime().getTime())
                                    + " mSecs");
                        }
                        else if (operation.isComplete()) {
                            this.operationsQueue.remove(0);
                            Match.log(title + ": Completed operation: " + operation.toString()
                                    + " in " + (new Date().getTime() - operation.getStartTime().getTime())
                                    + " mSecs");
                        }
                    }
                }
                if (!this.operationsQueue.isEmpty()) {
                    Operation operation = this.operationsQueue.get(0);
                    //if we haven't already started this operation, start it
                    if (!operation.getOperationIsBeingProcessed()) {
                        Match.log(title + ": Starting operation: " + operation);
                        operation.setOperationBeingProcessed();
                        operation.startOperation();
                    }
                }
            }
            try {
                Thread.yield();
            }
            catch (Throwable ignored) {}
        }
    }

    public void queueUpOperation(Operation operation) {
        synchronized (threadLock) {
            this.operationsQueue.add(operation);
        }
    }

    public void abort() {
        synchronized (threadLock) {
            //if we performing an operation - abort it
            if (!this.operationsQueue.isEmpty()) {
                Operation operation = this.operationsQueue.get(0);
                if (operation.getOperationIsBeingProcessed()) {
                    Match.log(title + ": Aborting operation: " + operation.getTitle());
                    operation.abortOperation();
                    operation.setAborted(true);
                }
            }
            this.operationsQueue.clear();
        }
    }

    public boolean hasEntries() {
        synchronized (threadLock) {
            return !this.operationsQueue.isEmpty();
        }
    }
}

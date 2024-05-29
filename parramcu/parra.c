using System;
using System.Collections.Generic;
using System.Threading;

public class Controller
{
    private List<Stepper> steppers;
    private Timer schedulerTimer;

    public Controller()
    {
        steppers = new List<Stepper>();

        // Initialize and start the scheduler timer
        schedulerTimer = new Timer(SchedulerCallback, null, TimeSpan.Zero, TimeSpan.FromMilliseconds(10)); // Adjust the interval as needed
    }
    public void AddStepper(Stepper stepper)
	{
		steppers.Add(stepper);
	}


    private void SchedulerCallback(object state)
    {
        // Perform scheduling tasks here

        // For example:
        // 1. Read from serial
        // 2. Control steppers
        // 3. Write to serial

        // Example:
        // Read from serial
        // SerialReadIn();

        // Control steppers
        ControlSteppers();

        // Write to serial
        // SerialReadOut();
    }

    private void ControlSteppers()
    {
        // Iterate through each stepper and control it
        foreach (var stepper in steppers)
        {
            // Control the stepper here
            stepper.Control();
        }
    }

    // Add methods for SerialReadIn, SerialReadOut, and any other operations as needed
}



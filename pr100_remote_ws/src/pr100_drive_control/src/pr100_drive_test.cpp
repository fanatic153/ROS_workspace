
#include "../include/pr100_drive_control/pr100_drive.hpp"


/* ----- main ------- */
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pr100_drive");
    
    Pr100Drive *pr100_drive = new Pr100Drive();

    pr100_drive->init();

    ros::Rate loop_rate(125);

    srand( time(NULL) );
    int count = 0;
    
    int prevState = pr100_drive->currentState;
    pr100_drive->updatePose();
    while (ros::ok())
    {
        count++;
        if (count >= 50)
        {
            pr100_drive->showSensorData();
            pr100_drive->showState();

            count = 0;
        }

        // state control
        pr100_drive->controlLoop();
        // state machine
        switch (pr100_drive->currentState)
        {
            case STATE_NORMAL:
                pr100_drive->stateNormal();
                break;

            case STATE_TURN:
                pr100_drive->stateTurn();
                break;

            case STATE_NARROW:
                pr100_drive->stateNarrow();
                break;

            case STATE_ESCAPE:
                pr100_drive->stateEscape();
                break;

            default:
                break;
        }

        // show switched state
        if (prevState != pr100_drive->currentState)
        {
            pr100_drive->showState();
        }
        // saving state
        prevState = pr100_drive->currentState;

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::shutdown();
    return 0;
}

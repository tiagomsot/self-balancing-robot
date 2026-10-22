#include "RobotCore.h"

namespace robot {
// EDIT THIS FUNCTION to change what your robot does.
// These calls build a queue; they never pause the balance controller.
void defineRoutine(Routine& robot) {
    const RoutineSettings& settings=robot.settings();

    // 1. Require continuous settled balance before starting.
    robot.waitUntilBalanced(settings.settle_time);

    // 2. Drive forward: default 0.50 metres, up to 0.25 m/s.
    // To set your own values directly, use: robot.move(0.30, 0.15);
    robot.move(settings.leg_distance, settings.speed);

    // 3. Brake, settle and hold balance for the configured 2 seconds.
    robot.stop(settings.settle_time);

    // 4. Turn left 180 degrees, up to 50 degrees/s. Negative degrees turn right.
    robot.rotate(180.0, settings.turn_rate*180.0/pi);

    // 5. Hold the new heading while balancing (default 1 second).
    robot.wait(settings.turn_settle_time);

    // 6. Repeat from the move step: drive back, stop and turn again.
    // Gazebo cycles:=1 ends after two moves and their stops; 0 repeats forever.
    // Remove this line to finish the sequence once and remain balancing.
    robot.repeat();
}
}

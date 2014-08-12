function [linVel_L,linVel_R,angVel_L,angVel_R,wristVel_L,wristVel_R,grip_L,grip_R] = initializeParams(baxterP)

    % Initialize left arm params
    linVel_L = [0;0;0];
    angVel_L = [0;0;0];
    wristVel_L = [];
    grip_L = [];
    baxterP.calibrateGripper('left');
    pause(2);
    baxterP.setGripperHoldForce('left',100);
    baxterP.setGripperMoveForce('left',100);

    % Initialize right arm params
    linVel_R = [0;0;0];
    angVel_R = [0;0;0];
    wristVel_R = [];
    grip_R = [];
    baxterP.calibrateGripper('right');
    pause(2);
    baxterP.setGripperHoldForce('right',100);
    baxterP.setGripperMoveForce('right',100);

end
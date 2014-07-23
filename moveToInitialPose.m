function moveToInitialPose(baxterJ)

    baxterJ.setControlMode(uint8(0));
    baxterJ.setJointCommand('left',[-0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
    baxterJ.setJointCommand('right',[0.7854;-1.0472;0;2.0944;0;-1.0472;-0]);
    pause(7);
    baxterJ.setControlMode(uint8(1));

end
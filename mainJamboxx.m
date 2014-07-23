% This is the main file for Jamboxx based control of Baxter.

%% Robot Raconteur Connections

% Baxter joint
jointServerPort = textread('jServer.txt');
try
    baxterJ = RobotRaconteur.Connect(['tcp://localhost:',num2str(jointServerPort),'/BaxterJointServer/Baxter']);
catch err
    disp(['CANNOT CONNECT TO BAXTER JOINT SERVER:',err.message]);
end

% Baxter peripherals
peripheralServerPort = textread('pServer.txt');
try
    baxterP = RobotRaconteur.Connect(['tcp://localhost:',num2str(peripheralServerPort),'/BaxterPeripheralServer/BaxterPeripherals']);
catch err
    disp(['CANNOT CONNECT TO BAXTER PERIPHERALS SERVER:',err.message]);
end

% Jamboxx
jamboxxIP = '192.168.139.1';
try
    jamboxx = RobotRaconteur.Connect(['tcp://',jamboxxIP,':5318/{0}/Jamboxx']);
catch err
    disp(['CANNOT CONNECT TO JAMBOXX SERVER:',err.message]);
end

% Xbox 360 controller
try
    xbox = RobotRaconteur.Connect(['tcp://',jamboxxIP,':5437/Xbox_controllerServer/xbox_controller']);
catch err
    disp(['CANNOT CONNECT TO XBOX SERVER:',err.message]);
end

% Wheelchair
try
    w = RobotRaconteur.Connect('tcp://localhost:3400/{0}/WheelChair');
catch err
    disp(['CANNOT CONNECT TO WHEELCHAIR:',err.message]);
end

% Voice
try
    voiceObj = udp(jamboxxIP,9094,'LocalPort',9094);
catch err
    disp(['CANNOT CONNECT TO VOICE HOST:',err.message]);
end
fopen(voiceObj);


%% Main

setBaxterConstants;
[linVel_L,linVel_R,angVel_L,angVel_R,wristVel_L,wristVel_R,grip_L,grip_R] = initializeParams(baxterP);
moveToInitialPose(baxterJ);
command = 'baxter';
jamsterMode = 'baxterTrans';

calibrateJamboxx(jamboxx);
clc; disp('READY...');

while(1) 
   
    command = readVoiceCommand(voiceObj);
    disp(command);
    switch command
        case 'wheelchair'
            jamsterMode = 'wheelchair';
        case 'baxter'
            jamsterMode = 'baxterTrans';
        case 'translation'
            jamsterMode = 'baxterTrans';
        case 'rotation'
            jamsterMode = 'baxterRot';
        case 'stop'
            w.Stop();
        case 'open right'
            baxterP.setGripperPosition('right',double(100));	
        case 'close right'
            baxterP.setGripperPosition('right',double(0));
        case 'open left'
            baxterP.setGripperPosition('left',double(100));
        case 'close left'
            baxterP.setGripperPosition('left',double(0));
        case 'pose'
            moveToInitialPose(baxterJ);
        otherwise
    end

    % Gather joint information
    jointAngles = baxterJ.joint_positions;
    jointAnglesLeft = jointAngles(1:7);
    jointAnglesRight = jointAngles(8:14);

    % Calculate full jacobian for both arms
    leftJ = jacobian(baxterConst.leftArm,jointAnglesLeft);
    rightJ = jacobian(baxterConst.rightArm,jointAnglesRight);
    
    % Set desired input using Jamboxx
    [linVel_L,linVel_R,angVel_L,angVel_R,wristVel_L,wristVel_R,grip_L,grip_R] = getUserInput(jamboxx,jamsterMode,w); 
    
    % Desired velocities frame correction
        % Left arm
        [rotToolLeft,~] = fwdKin(baxterConst.leftArm,jointAnglesLeft);
        linVelCorrect_L = rot([0;0;1],pi/2)*rotToolLeft*linVel_L;
        angVelCorrect_L = rot([0;0;1],pi/2)*rotToolLeft*angVel_L;
        allVel_L = [angVelCorrect_L;linVelCorrect_L];
        % Right arm
        [rotToolRight,~] = fwdKin(baxterConst.rightArm,jointAnglesRight);
        linVelCorrect_R = rot([0;0;1],pi/2)*rotToolRight*linVel_R;
        angVelCorrect_R = rot([0;0;1],pi/2)*rotToolRight*angVel_R;
        allVel_R = [angVelCorrect_R;linVelCorrect_R];
    
    % Calculate desired joint angle velocities
        % Left arm
        dampCoeff_L = 0.1;
        qDot_L = [0;0;0;0;0;0;0];
        if any(allVel_L)
            qDot_L = leftJ'*pinv(leftJ*leftJ' + dampCoeff_L^2*eye(6,6))*allVel_L; %Damped least squares
        end
        if ~isempty(wristVel_L)
            qDot_L(5:7) = wristVel_L;
        end

        % Right arm
        dampCoeff_R = 0.1;
        qDot_R = [0;0;0;0;0;0;0];
        if any(allVel_R)
            qDot_R = rightJ'*pinv(rightJ*rightJ' + dampCoeff_R^2*eye(6,6))*allVel_R; %Damped least squares
        end   
        if ~isempty(wristVel_R)
            qDot_R(5:7) = wristVel_R;
        end 
    
    % Publish joint position
    baxterJ.setControlMode(uint8(1));
	baxterJ.setJointCommand('left',qDot_L);
	baxterJ.setJointCommand('right',qDot_R);
        
    % Publish grip position
        % Left gripper
        if ~isempty(grip_L)
            baxterP.setGripperPosition('left',grip_L);
        end
        %Right gripper
        if ~isempty(grip_R)
            baxterP.setGripperPosition('right',grip_R);
        end

   pause(0.1); 

end

clc; msgbox('Program stopped!');

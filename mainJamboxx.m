% This is the main file for Jamboxx based control of Baxter.

clear all; close all; clc;
addpath('./functions');

%% Robot Raconteur Connections

% Baxter joint
try
    jointServerPort = textread('jServer.txt');
catch err
    jointServerPort = input('Enter joint server port:');
end
try
    baxterJ = RobotRaconteur.Connect(['tcp://localhost:',num2str(jointServerPort),'/BaxterJointServer/Baxter']);
catch err
    error(['ERROR:CANNOT CONNECT TO BAXTER JOINT SERVER: ',err.message]);
end

% Baxter peripherals
try
    peripheralServerPort = textread('pServer.txt');
catch err
    peripheralServerPort = input('Enter peripheral server port:');
end
try
    baxterP = RobotRaconteur.Connect(['tcp://localhost:',num2str(peripheralServerPort),'/BaxterPeripheralServer/BaxterPeripherals']);
catch err
    disp(['WARNING:CANNOT CONNECT TO BAXTER PERIPHERALS SERVER: ',err.message]);
end

% Jamboxx
jamboxxIP = '192.168.139.1';
try
    jamboxx = RobotRaconteur.Connect(['tcp://',jamboxxIP,':5318/{0}/Jamboxx']);
catch err
    disp(['WARNING:CANNOT CONNECT TO JAMBOXX SERVER: ',err.message]);
end

% Xbox 360 controller
try
    xbox = RobotRaconteur.Connect(['tcp://',jamboxxIP,':5437/Xbox_controllerServer/xbox_controller']);
catch err
    disp(['WARNING:CANNOT CONNECT TO XBOX SERVER: ',err.message]);
end

% Wheelchair
try
    w = RobotRaconteur.Connect('tcp://localhost:3400/{0}/WheelChair');
catch err
    disp(['WARNING:CANNOT CONNECT TO WHEELCHAIR: ',err.message]);
    w = 0;
end

% Voice
voiceObj = udp(jamboxxIP,9097,'LocalPort',9097);
try
    fopen(voiceObj);
catch err
    disp(['WARNING:CANNOT CONNECT TO VOICE HOST: ',err.message]);
    voiceObj = 'false';
end


%% Main

setBaxterConstants;

if exist('baxterP','var')
    [linVel_L,linVel_R,angVel_L,angVel_R,wristVel_L,wristVel_R,grip_L,grip_R] = initializeParams(baxterP);
end
disp(' '); disp('Initializing robot...');
moveToInitialPose(baxterJ);

command = 'baxter translate left';
jamsterMode = 'baxterTransLeft';

disp(' '); disp('Calibrating jamboxx!');
if exist('jamboxx','var')
    %calibrateJamboxx(jamboxx);
end

clc; disp('READY...');

while(1) 
   
    if ~strcmp(voiceObj,'false')
        command = readVoiceCommand(voiceObj,command);
        disp(command);
    end
    parsedCommand = splitstring(command);
    [jamsterMode,grip_L,grip_R] = determineMode(parsedCommand,w,baxterJ);
    
    if strcmp(parsedCommand{1},'wheelchair')
        baxterJ.SetControlMode('wheelchair');
    elseif strcmp(parsedCommand{1},'baxter') && (strcmp(parsedCommand{2},'translate')||strcmp(parsedCommand{2},'rotate'))
	if strcmp(parsedCommand{3},'left');
	    baxterJ.SetActiveArm('left');
        elseif strcmp(parsedCommand{3},'right');
	    baxterJ.SetActiveArm('right');
	end
	baxterJ.SetControlMode('baxter');
    end

    % Gather joint information
    jointAngles = baxterJ.joint_positions;
    jointAnglesLeft = jointAngles(1:7);
    jointAnglesRight = jointAngles(8:14);

    % Calculate full jacobian for both arms
    leftJ = jacobian(baxterConst.leftArm,jointAnglesLeft);
    rightJ = jacobian(baxterConst.rightArm,jointAnglesRight);
    
    % Set desired input using Jamboxx
    [linVel_L,linVel_R,angVel_L,angVel_R,wristVel_L,wristVel_R] = getUserInput(jamboxx,jamsterMode,w); 
    
    % Desired end effector velocities
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
    
    if exist('baxterP','var')
    % Publish grip position
        % Left gripper
        if ~isempty(grip_L)
            baxterP.setGripperPosition('left',grip_L);
        end
        %Right gripper
        if ~isempty(grip_R)
            baxterP.setGripperPosition('right',grip_R);
        end
    end

   pause(0.1); 

end

clc; msgbox('Program stopped!');

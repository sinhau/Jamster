function [jamsterMode,grip_L,grip_R] = determineMode(command,w,baxterJ)

    jamsterMode = 'baxterTransLeft';
    commandType = command{1};
    try
        commandType2 = command{2};
    catch err
    end

    grip_L = []; grip_R = [];

    switch commandType
        case 'wheelchair'
            jamsterMode = 'wheelchair';
        case 'baxter'
            switch commandType2
                case 'translate'
                    if strcmp(command{3},'left')
                        jamsterMode = 'baxterTransLeft';
                    else
                        jamsterMode = 'baxterTransRight';
                    end
                case 'rotate'
                    if strcmp(command{3},'left')
                        jamsterMode = 'baxterRotLeft';
                    else
                        jamsterMode = 'baxterRotRight';
                    end
                case 'wrist'
                    jamsterMode = 'baxterWrist';
            end
        case 'gripper'
            switch commandType2
                case 'open'
                    if strcmp(command{3},'left')
                        grip_L = double(100);
                    else
                        grip_R = double(100);
                    end
                case 'close'
                    if strcmp(command{3},'left')
                        grip_L = double(0);
                    else
                        grip_R = double(0);
                    end
            end
        case 'stop'
            %w.Stop();
        case 'pose'
            moveToInitialPose(baxterJ);
        otherwise
    end

end
function command = readVoiceCommand(voiceObj,commandOld)
    
    if(voiceObj.BytesAvailable)
        fieldWidth = ['%',num2str(voiceObj.BytesAvailable),'c'];
        command = fscanf(voiceObj,fieldWidth);
    else
	command = commandOld
    end
    
end

function [] = turnLedOnOff(status,imgLedOn,imgLedOff)
%This function will turn the left LED on the crossing gate on or off

if(status)
    %Turn the LED on if it is off
    if(strcmp(imgLedOn.Visible,'off'))
        imgLedOn.Visible = 'on';
        imgLedOff.Visible = 'off';
    end
else
    %Turn the LED off if it is on
    if(strcmp(imgLedOff.Visible,'off'))
        imgLedOff.Visible = 'on';
        imgLedOn.Visible = 'off';
    end
end

end


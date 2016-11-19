delete(timerfindall);
clear all;
close all;
delete(instrfindall);
clc;
%%
a=arduino_sim();
a.pinMode(14,'output')
a.pinMode(15,'output')
a.digitalWrite(15,1)
a.digitalWrite(14,1)
rLed=14;
lLed=15;


%1 is servo port number
a.servoAttach(1)

%a.servoWrite(X,?) where X is servo # and ? is angle of gate
a.servoWrite(1,65)
%a.servoWrite(1,170)
approach=2;
departure=3;
dep= a.analogRead(departure);
app= a.analogRead(approach);

a.motorRun(1,'forward') % cant move unless motorspeed command is run
a.motorSpeed(1,170) % X is the speed from 170-255 m/s

flag=1;
while 1
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
    if a.analogRead(approach)>300 && flag==1
        tic;
        flag=0;
    end
    a.analogRead(departure);
    a.analogRead(departure);
    a.analogRead(departure);
    a.analogRead(departure);
    a.analogRead(departure);
    if a.analogRead(departure)>200 && flag==0
        a.motorSpeed(1,255);
        time=toc
        flag=1;
        speed_inps=22.5*pi*.5/time
        speed_mph=22.5*pi*.5/time/12/5280*3600
    end 
  
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
    if a.analogRead(approach)>300 && flag==1
        tic;
        flag=0;
    end
end

%tic;

%while dep < 300
%   if ((toc-floor(toc))<0.5)
%        a.digitalWrite(rLed,1)
%    else
%       a.digitalWrite(rLed,0)
%       a.digitalWrite(rLed,1)
%    end
    
%    if (toc >1.5)
%        a.servoWrite(1,170)
%    end
%end
%%
a= arduino_sim();
a.servoAttach(1)
a.servoWrite(1,44)
a.pinMode(14,'output')
a.pinMode(15,'output')
a.digitalWrite(15,0);
a.digitalWrite(14,0);
rLED=14;
lLED=15;
a.motorRun(1,'forward');
a.motorSpeed(1,255);
approach=2;
departure=3;
Flag=0;
town = input('Press 1 for Urban or 2 for rural: ')
if town==1
    while 1
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
        if a.analogRead(approach)>250
           a.motorSpeed(1,170);
           rLight=0;
           lLight=0.4;
           Flag=1;
           tic;
        end
    a.analogRead(departure);
    a.analogRead(departure);
    a.analogRead(departure);
    a.analogRead(departure);
        if a.analogRead(departure)> 200
           a.motorSpeed(1,255);
           Flag=0;
        end
        if Flag
            time=toc;
            if time>lLight
                a.digitalWrite(15,1);
                a.digitalWrite(14,0);
                lLight=lLight + 1;
            elseif time>rLight
                a.digitalWrite(15,0);
                a.digitalWrite(14,1);
                rLight=rLight+1;
            end
            if time>=1
                a.servoWrite(1,170);
            end
        else
            a.digitalWrite(15,0);
            a.digitalWrite(14,0);
            a.servoWrite(1,75);
        end
    end
end
if town==2
    while 1
          a.analogRead(approach);
          a.analogRead(approach);
          a.analogRead(approach);
          a.analogRead(approach);
            if a.analogRead(approach)>250
               a.motorSpeed(1,170);
               rLight=0;
               lLight=0.4;
               Flag=1;
               tic;
            end
          a.analogRead(departure);
          a.analogRead(departure);
          a.analogRead(departure);
          a.analogRead(departure);
            if a.analogRead(departure)>200
               a.motorSpeed(1,255);
               Flag=0;
            end
            if Flag
            time=toc;
                if time>lLight
                   a.digitalWrite(15,1);
                   a.digitalWrite(14,0);
                   lLight=lLight + 1;
                elseif time>rLight
                   a.digitalWrite(15,0);
                   a.digitalWrite(14,1);
                   rLight=rLight+1;
                end
                if time>=0.5
                   a.servoWrite(1,170);
                end
            else
                a.digitalWrite(15,0);
                a.digitalWrite(14,0);
                a.servoWrite(1,75);
            end
    end
end
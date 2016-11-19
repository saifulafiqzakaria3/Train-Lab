%Code for Train Lab
fprintf ('Group:Q Minh, Kevin, Wan, Jake.')
fprintf('\nHadri Power Gila\n')
fprintf ('Engineering 1181 1:50-2:45 Schrock')
%Clear out and delete previous setups and variables
delete(timerfindall);
clear all;
close all;
delete(instrfindall);
clc;
%Attatch arduino 
a=arduino_sim();
%Attach crossing gate
a.servoAttach(1);
%Setup initial gate angle
a.servoWrite(1,65);
%Attatch the lights and Turn on lights
a.pinMode(14,'output');
a.pinMode(15,'output');
a.digitalWrite(15,0);
a.digitalWrite(14,0);
%Assign lights and gates to variables
rLED = 15; 
lLED = 14;
approach=2;
departure=3;
%Control trains velocity
a.motorRun(1,'forward');
a.motorSpeed(1,170);

Flag=0;
town = input('Press 1 for Urban or 2 for rural: ')
%Take multiple approach readings
%a.analogRead(approach);
%a.analogRead(approach);
%a.analogRead(approach);
%a.analogRead(approach);
%a.analogRead(approach);
%Set up flag system for approach and departure
%flag=1;
%Infinite while loop for train
if town==1
    while 1
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
    a.analogRead(approach);
        if a.analogRead(approach)>250
           a.motorSpeed(1,170);
           rLight=0;
           lLight=0.8;
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
   



%For actual lab testing,
%Change arduino COM, approach, and departure, angles, 

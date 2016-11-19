% This class creates a simulator environment that takes a student's code for
% the train project, parses through it, and builds a simulator in a Matlab
% figure window. This simulator will run (or not run) based on the student's
% code.

% This class uses global variables to be able to use the same objects
% between different functions.

% Created by Ryan Stutzman, 06/15/2015

% Based off of a previous simulator created by Roman Kazachkov and edited by
% Joshua Branch in 2013

classdef arduino_sim < handle
    % This class is a simulator object for the EEIC Train Lab
    % This class was created based on the arduino class made by MathWorks, Inc.
    
    properties (SetAccess=private,GetAccess=private)
        tmrMain;
        tmrRandomGateLEDs;
        theta = 20;
        radius = 1;
        simFigure;
        
        motorDirections = zeros(1,1);
        motorSpeeds = zeros(1,1);
        
        servosAttached = zeros(1,1);
        servoValues = zeros(1,1);
        
        digitalDir = zeros(1,15);
        digitalState = zeros(1,15);
        
        lastUpdateTime = clock;
        
        analogReads = zeros(1,1);
        
    end
    
    methods
        % constructor, creates the figure window, draws the track, and
        % creates and starts the timer that continuously replots everything
        function this=arduino_sim()
            % Create the figure that this simulator will be using
            this.simFigure = figure(...
                'Name','Train Lab Simulation',...
                'NumberTitle','off',...
                'MenuBar','none',...
                'ToolBar','none',...
                'CloseRequestFcn',{@this.windowClosing});
            
            % Let the user know we're getting the simulator ready
            disp(' ');
            disp('SIMULATOR: Initializing...');
            disp(' ');
            
            % Draw the track and everything else that only needs to be
            % drawn once
            this.initializeFigure();
            
            % Start the simulation
            this.startSimulation();
            
            % Let the user know the simulator is running
            disp(' ');
            disp('SIMULATOR: Running...');
            disp(' ');

        end % arduino_sim_rs

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %               Arduino functions for the simulator               %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % pin mode, changes pin mode
        function pinMode(this,pin,str)

            % a.pinMode(pin,str); specifies the pin mode of a digital pins.
            % The first argument before the function name, a, is the arduino object.
            % The first argument, pin, is the number of the digital pin (2 to 19).
            % The second argument, str, is a string that can be 'input' or 'output',
            % Called with one argument, as a.pin(pin) it returns the mode of
            % the digital pin, called without arguments, prints the mode of all the
            % digital pins. Note that the digital pins from 0 to 13 are located on
            % the upper right part of the board, while the digital pins from 14 to 19
            % are better known as "analog input" pins and are located in the lower
            % right corner of the board.
            %
            % Examples:
            % a.pinMode(11,'output') % sets digital pin #11 as output
            % a.pinMode(10,'input')  % sets digital pin #10 as input
            % val=a.pinMode(10);     % returns the status of digital pin #10
            % a.pinMode(5);          % prints the status of digital pin #5
            % a.pinMode;             % prints the status of all pins
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % If the student correctly stated that the pin would be used for
            % output (they shouldnt be inputting any analog signals), then
            % the data_hub will receive the pin value to change. Since we
            % are initializing, the data_hub will know to set the value to 1
            % so it is not necessary to pass that in. 

            if (isreal(pin) && isscalar(pin))
                if strcmp(str,'output')
                    this.digitalDir(pin) = 1;
                else
                    this.digitalDir(pin) = 0;
                end
            end

        end % pinMode
        
        % digital write
        function digitalWrite(this,pin,val)

            % a.digitalWrite(pin,val); performs digital output on a given pin.
            % The first argument before the function name, a, is the arduino object.
            % The second argument, pin, is the number of the digital pin (2 to 19)
            % where the digital output needs to be performed.
            % The third argument, val, is the value (either 0 or 1) for the output
            % Note that the digital pins from 0 to 13 are located on the upper right part
            % of the board, while the digital pins from 14 to 19 are better known as
            % "analog input" pins and are located in the lower right corner of the board.
            %
            % Examples:
            % a.digitalWrite(13,1); % sets pin #13 high
            % a.digitalWrite(13,0); % sets pin #13 low
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % This will pass in the pin# and whether that pin is HIGH or LOW to
            % data_hub. The nonsense variable is because digitalWrite doesnt care
            % for a return value. 

            if (isreal(pin) && isscalar(pin)) && (isreal(val) && isscalar(val))
                this.digitalState(pin) = val;
            end

        end % digitalWrite
        
        % digital read
        function val=digitalRead(this,pin)
            
            % val=a.digitalRead(pin); performs digital input on a given arduino pin.
            % The first argument before the function name, a, is the arduino object.
            % The argument pin, is the number of the digital pin (2 to 69)
            % where the digital input needs to be performed. On the Arduino Uno
            % board the digital pins from 0 to 13 are located on the upper right part
            % while the digital pins from 14 to 19 are better known as "analog input"
            % pins and are located in the lower right corner of the board.
            %
            % If the argument pin is outside of 1 - 15, the value of -1
            % will be returned.
            %
            % Example:
            % val=a.digitalRead(4); % reads pin #4
            %
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % check nargin
            if nargin~=2,
                error('Function must have the "pin" argument');
            end
            
            if (isreal(pin) && isscalar(pin) && (pin >= 1 && pin <= 15))
                val = this.digitalState(pin);
            else
                val = -1;
            end
            
        end % digitalread

        % analog read
        function val=analogRead(this,pin)

            % val=a.analogRead(pin); Performs analog input on a given arduino pin.
            % The first argument before the function name, a, is the arduino object.
            % The second argument, pin, is the number of the analog input pin (0 to 5)
            % where the analog input needs to be performed. The returned value, val,
            % ranges from 0 to 1023, with 0 corresponding to an input voltage of 0 volts,
            % and 1023 to a reference value that is typically 5 volts (this voltage can
            % be set up by the analogReference function). Therefore, assuming a range
            % from 0 to 5 V the resolution is .0049 volts (4.9 mV) per unit.
            % Note that the analog input pins 0 to 5 are also known as digital pins
            % from 14 to 19, and are located on the lower right corner of the board.
            % Specifically, analog input pin 0 corresponds to digital pin 14, and analog
            % input pin 5 corresponds to digital pin 19. Performing analog input does
            % not affect the digital state (high, low, digital input) of the pin.
            %
            % Example:
            % val=a.analogRead(0); % reads analog input pin # 0
            %


            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % check arguments if a.chkp is true

            %%%%%%%%%%%%%%%%%%%%%%%%% PERFORM ANALOG INPUT %%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Here we pass in the gate pin and get whether or not that gate is
            % obstructed. All of the logic is in data_hub and all we get at val is
            % either 50 for unobstructed ot 400 for obstructed. We need a return
            % value here bacause it is passed back to the original student's code so
            % they can use it in their logic to see whether or not their gate is
            % obstructed. 
            newVal = 0;

            if pin == 2
                if this.theta >= 350 || this.theta <= 10
                    newVal = round(rand(1)*25 + 385);
                else
                    newVal = round(rand(1)*20 + 50);
                end
            elseif pin == 3
                if this.theta >=170 && this.theta <= 190
                    newVal = round(rand(1)*25 + 385);
                else
                    newVal = round(rand(1)*20 + 50);
                end
            end

            val = this.analogReads(1);

            for i = 1:length(this.analogReads) - 1
                this.analogReads(i) = this.analogReads(i + 1);
            end

            this.analogReads(length(this.analogReads)) = newVal;

            pause(0.001);
        end % analogRead

        % servo attach
        function servoAttach(this,num)

            % a.servoAttach(num); attaches a servo to the corresponding pwm pin.
            % The first argument before the function name, a, is the arduino object.
            % The second argument, num, is the number of the servo, which can be either 1
            % (top servo, uses digital pin 10 for pwm), or 2 (bottom servo, uses digital
            % pin 9 for pwm). Returns Random results if motor shield is not connected.
            %
            % Example:
            % a.servoAttach(1); % attach servo #1
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % check arguments if a.chkp is true

            % Passes in which servo to attach. No return needed. 
            if (isreal(num) && isscalar(num))
                this.servosAttached(num) = 1;
            end
        end % servoAttach

        % servo detach
        function servoDetach(this,num)

            % a.servoDetach(num); detaches a servo from its corresponding pwm pin.
            % The first argument before the function name, a, is the arduino object.
            % The second argument, num, is the number of the servo, which can be either 1
            % (top servo, uses digital pin 10 for pwm), or 2 (bottom servo, uses digital
            % pin 9 for pwm). Returns random results if motor shield is not connected.
            %
            % Examples:
            % a.servoDetach(1); % detach servo #1
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % check arguments if a.chkp is true

            % Detaches the servo. 
            if (isreal(num) && isscalar(num))
                this.servosAttached(num) = 0;
            end

        end % servoDetach

        % servo status
        function val=servoStatus(this,num)

            % a.servoStatus(num); Reads the status of a servo (attached/detached)
            % The first argument before the function name, a, is the arduino object.
            % The second argument, num, is the number of the servo, which can be either 1
            % (top servo, uses digital pin 10 for pwm), or 2 (bottom servo,
            % uses digital pin 9 for pwm).
            % The returned value is either 1 (servo attached) or 0 (servo detached),
            % Called without output arguments, the function prints a string specifying
            % the status of the servo. Called without input arguments, the function
            % either returns the status vector or prints the status of each servo.
            % Returns Random results if motor shield is not connected.
            %
            % Examples:
            % val=a.servoStatus(1); % return the status of servo #1
            % a.servoStatus(1); % prints the status of servo #1
            % a.servoStatus; % prints the status of both servos
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % check nargin if a.chkp is true
            if (isreal(num) && isscalar(num))
                val = this.servosAttached(num);
            end

        end % servoStatus

        % servo read
        function val=servoRead(this,num)

            % val=a.servoRead(num); reads the angle of a given servo.
            % The first argument before the function name, a, is the arduino object.
            % The second argument, num, is the number of the servo, which can be either
            % 1 (top servo, uses digital pin 10 for pwm), or 2 (bottom servo, uses
            % digital pin 9 for pwm). The returned value is the angle in degrees,
            % typically from 0 to 180. Returns Random results if motor shield is not
            % connected.
            %
            % Example:
            % val=a.servoRead(1); % reads angle from servo #1
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % check arguments if a.chkp is true

            %%%%%%%%%%%%%%%%%%%%%%%%% READ SERVO ANGLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if (isreal(num) && isscalar(num))
                val = this.servoValues(num);
            end

        end % servoRead

        % servo write
        function servoWrite(this,num,val)

            % a.servoWrite(num,val); writes an angle on a given servo.
            % The first argument before the function name, a, is the arduino object.
            % The second argument, num, is the number of the servo, which can be
            % either 1 (top servo, uses digital pin 10 for pwm), or 2 (bottom servo,
            % uses digital pin 9 for pwm). The third argument is the angle in degrees,
            % typically from 0 to 180. Returns Random results if motor shield is not
            % connected.
            %
            % Example:
            % a.servoWrite(1,45); % rotates servo #1 of 45 degrees
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % check arguments if a.chkp is true

            % No return necessary. Passes in the angle to data_hub which will set
            % the servo to up or down accordingly. 
            if (isreal(num) && isscalar(num)) && (isreal(val) && isscalar(val))
                this.servoValues(num) = val;
            end

        end % servoWrite

        % motor speed
        function val=motorSpeed(this,num,val)

            % val=a.motorSpeed(num,val); sets the speed of a DC motor.
            % The first argument before the function name, a, is the arduino object.
            % The second argument, num, is the number of the motor, which can go
            % from 1 to 4 (the motor ports are numbered on the motor shield).
            % The third argument is the speed from 0 (stopped) to 255 (maximum), note
            % that depending on the motor speeds of at least 60 might be necessary
            % to actually run it. Called with one argument, as a.motorSpeed(num),
            % it returns the speed at which the given motor is set to run. If there
            % is no output argument it prints the speed of the motor.
            % Called without arguments, itprints the speed of each motor.
            % Note that you must use the command a.motorRun to actually run
            % the motor at the given speed, either forward or backwards.
            % Returns Random results if motor shield is not connected.
            %
            % Examples:
            % a.motorSpeed(4,200)      % sets speed of motor 4 as 200/255
            % val=a.motorSpeed(1);     % returns the speed of motor 1
            % a.motorSpeed(3);         % prints the speed of motor 3
            % a.motorSpeed;            % prints the speed of all motors
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % check arguments if a.chkp is true

            % Sets the train speed (angle increment size)

            if (isreal(num) && isscalar(num)) && (isreal(val) && isscalar(val))
                this.motorSpeeds(num) = max(min(val,255),0);
            end

        end % motorSpeed

        % motor run
        function motorRun(this,num,dir)

            % a.motorRun(num,dir); runs a given DC motor.
            % The first argument before the function name, a, is the arduino object.
            % The second argument, num, is the number of the motor, which can go
            % from 1 to 4 (the motor ports are numbered on the motor shield).
            % The third argument, dir, should be a string that can be 'forward'
            % (runs the motor forward) 'backward' (runs the motor backward)
            % or 'release', (stops the motor). Note that since version 3.0,
            % a +1 is interpreted as 'forward', a 0 is interpreted
            % as 'release', and a -1 is interpreted as 'backward'.
            % Returns Random results if motor shield is not connected.
            %
            % Examples:
            % a.motorRun(1,'forward');      % runs motor 1 forward
            % a.motorRun(3,'backward');     % runs motor 3 backward
            % a.motorRun(2,-1);             % runs motor 2 backward
            % a.motorRun(1,'release');      % releases motor 1
            %

            %%%%%%%%%%%%%%%%%%%%%%%%% ARGUMENT CHECKING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


            % Initializes the motor 
            if strcmp(dir,'forward')
                this.motorDirections(num) = 1;
            elseif strcmp(dir,'backward')
                this.motorDirections(num) = -1;
            else
                this.motorDirections(num) = 0;
            end


        end % motorRun
        
        % function random gate LEDs
        function randomGateLeds(this,val)
            % We need to check to make sure the correct arguments were entered
            % check nargin
            if nargin~=2,
                error('Function must have the "val" argument, 0=off or 1=on');
            end
            
            % check val
            errstr=arduino.checknum(val,'random gate led state',0:1);
            if ~isempty(errstr), error(errstr); end
            
            % Arguments have been entered correctly! Now either start or
            % stop the random gate led timer.
            if (val==0)
                this.stopRandomGateLEDs();
            else
                this.startRandomGateLEDs();
            end
            
        end % randomGateLeds
    end
    
    methods(Access=private)
        % Starts the redrawing of the simulator to allow the figure window
        % to simulate the train moving.
        function startSimulation(this)
            this.tmrMain = timer();
            set(this.tmrMain, 'executionMode', 'fixedSpacing');
            set(this.tmrMain, 'TimerFcn', {@this.update});
            set(this.tmrMain, 'Period', 0.1);
            start(this.tmrMain);
        end
        
        % Stops the redrawing of the simulator to allow the figure window
        % to close.
        function stopSimulation(this)
            try
                stop(this.tmrMain);
                delete(this.tmrMain);
            catch
            end
        end
        
        % Starts the random changing of the approach and departure gate
        % LEDs
        function startRandomGateLEDs(this)
            this.tmrRandomGateLEDs = timer();
            set(this.tmrRandomGateLEDs, 'executionMode', 'fixedSpacing');
            set(this.tmrRandomGateLEDs, 'TimerFcn', {@this.executeRandomGateLED});
            set(this.tmrRandomGateLEDs, 'Period', 4.5);
            start(this.tmrRandomGateLEDs);
        end
        
        % Stops the random changing of the approach and departure gate
        % LEDs
        function stopRandomGateLEDs(this)
            try
                stop(this.tmrRandomGateLEDs);
                delete(this.tmrRandomGateLEDs);
                % Turn off all gate LEDs
                this.digitalState(6) = 0;
                this.digitalState(7) = 0;
                this.digitalState(8) = 0;
                this.digitalState(9) = 0;
            catch
            end
        end
        
        % This is the actual function that gets executed for the random
        % gate LED changes
        function executeRandomGateLED(this, ~, ~)
            % Get a random 1x2 matrix of 1 and 0
            vals = rand(1,2) <= 0.5;
            
            % If initialized, turn ON/OFF Approach Sensor Red LED
            if (this.digitalDir(6) == 1)
                % Set the value of the LED
                this.digitalState(6) = ~vals(1);
            else
                disp('SIMULATOR: Approach Gate Red LED (pin 6) not initialized!');
            end
            
            % If initialized, turn ON/OFF Approach Sensor Green LED
            if (this.digitalDir(7) == 1)
                % Set the value of the LED
                this.digitalState(7) = vals(1);
            else
                disp('SIMULATOR: Approach Gate Green LED (pin 7) not initialized!');
            end
            
            % If initialized, turn ON/OFF Departure Sensor Red LED
            if (this.digitalDir(8) == 1)
                % Set the value of the LED
                this.digitalState(8) = ~vals(2);
            else
                disp('SIMULATOR: Departure Gate Red LED (pin 8) not initialized!');
            end
            
            % If initialized, turn ON/OFF Departure Sensor Green LED
            if (this.digitalDir(9) == 1)
                % Set the value of the LED
                this.digitalState(9) = vals(2);
            else
                disp('SIMULATOR: Departure Gate Green LED (pin 9) not initialized!');
            end
        end
        
        % This function occurs when the user tries to close the figure
        % window.
        function windowClosing(this, ~, ~)
            % Before we show the dialog box we want to stop the 
            this.stopSimulation();
            
            % Flag to remember if this is running or not before we kill it
            isRandomGateLedRunning = 0;
            % Do we need to stop the random gate led timer?
            if (~isempty(this.tmrRandomGateLEDs) &&...
                    isa(this.tmrRandomGateLEDs,'timer') &&...
                    strcmp(get(this.tmrRandomGateLEDs, 'Running'), 'on'))
            
                % Looks like the random gate led timer is running...stop it
                this.stopRandomGateLEDs();
                % Set the flag
                isRandomGateLedRunning = 1;
            end
            
            % Make sure the user actually wants to close the simulator
            selection = questdlg('Close Simulator?', 'Are you sure?', 'Yes', 'No', 'Yes');
            
            switch selection
                case 'Yes'
                    delete(this.simFigure);
                    delete(this);
                    disp(' ');
                    disp('SIMULATOR: Shutdown');
                    disp(' ');
                case 'No'
                    % Continue the simulation
                    this.startSimulation();
                    % If the random gate led timer was running, restart it
                    if (isRandomGateLedRunning)
                        this.startRandomGateLEDs();
                    end
            end
        end
        
        % Draws the main track and some images to the figure window so they
        % only get drawn once
        function initializeFigure(this)
            global approachLine;
            global departLine;
            global speedText;
            global imgTrainLeft;
            global imgTrainRight;
            global imgGateUp;
            global imgGateDn;
            
            % Declare the global variables for the LED lights
            % (there's quite a few of them)
            global imgLEDGateLeftOff;
            global imgLEDGateLeftOn;
            global imgLEDGateRightOff;
            global imgLEDGateRightOn;
            global imgLEDDepartGreenOff;
            global imgLEDDepartGreenOn;
            global imgLEDDepartRedOff;
            global imgLEDDepartRedOn;
            global imgLEDApproachGreenOff;
            global imgLEDApproachGreenOn;
            global imgLEDApproachRedOff;
            global imgLEDApproachRedOn;
            
            % Load the needed global images
            imgTrainRight = imread('osu_train.jpg');
            imgTrainLeft = fliplr(imread('osu_train.jpg'));
            imgGateDn = imread('gate_down.jpg');
            imgGateUp = imread('gate_up.jpg');
            
            % LED Images
            imgLEDGateLeftOff = imread('circle_white.jpg');
            imgLEDGateLeftOn = imread('circle_red.jpg');
            imgLEDGateRightOff = imread('circle_white.jpg');
            imgLEDGateRightOn = imread('circle_red.jpg');
            imgLEDDepartGreenOff = imread('circle_white.jpg');
            imgLEDDepartGreenOn = imread('circle_green.jpg');
            imgLEDDepartRedOff = imread('circle_white.jpg');
            imgLEDDepartRedOn = imread('circle_red.jpg');
            imgLEDApproachGreenOff = imread('circle_white.jpg');
            imgLEDApproachGreenOn = imread('circle_green.jpg');
            imgLEDApproachRedOff = imread('circle_white.jpg');
            imgLEDApproachRedOn = imread('circle_red.jpg');
            
            % Load the needed local images
            imgCow = imread('cow.jpg');
            imgBarn = imread('barn.jpg');
            imgSign = imread('sign.jpg');
            
            numRungs = 18;
            rungx1(numRungs) = 0;
            rungx2(numRungs) = 0;
            rungy1(numRungs) = 0;
            rungy2(numRungs) = 0;
            x1(360) = 0;
            x2(360) = 0;
            y1(360) = 0;
            y2(360) = 0;
            rung_angle = 360 / numRungs;
            rung_shrink = 0.95;
            rung_extend = 1.05;

            % This will generate the inside circle and create coordinates along that
            % circle that will be used to create the wooden "rungs" that are in between
            % the two rails.
            
            j = 1;
            for theta2 = 0:1:359
                x1(theta2 + 1) = this.radius *0.9 * cosd(theta2);
                y1(theta2 + 1) = this.radius * 0.9 * sind(theta2);
                if (rem(theta2,rung_angle) == 0)
                    rungx1(j) = x1(theta2 + 1) * rung_shrink;
                    rungy1(j) = y1(theta2 + 1) * rung_shrink;
                    j = j + 1;
                end
            end

            % This will generate that outer rail and will store the coordinates for the
            % rungs again.
            
            j = 1;
            for theta2 = 0:1:359
                x2(theta2 + 1) = this.radius * 1.1 * cosd(theta2);
                y2(theta2 + 1) = this.radius * 1.1 * sind(theta2);
                if (rem(theta2,rung_angle) == 0)
                    rungx2(j) = x2(theta2 + 1) * rung_extend;
                    rungy2(j) = y2(theta2 + 1) * rung_extend;
                    j = j + 1;
                end
            end
            
            clf
            hold on
            % Plot inner circle
            plot(x1,y1,'-','LineWidth',3, 'color', [0.5 0.5 0.5]);
            axis equal
            % Plot outer circle
            plot(x2,y2,'-','LineWidth',3, 'color', [0.5 0.5 0.5]);
            
            % Plot the rungs
            for i = 1:length(rungx1)
                if (i == 1)
                    approachLine = plot([rungx1(i) rungx2(i)], [rungy1(i), rungy2(i)],'-','LineWidth',6,'color',[1 0 0]);
                elseif (i-1 == 180 / rung_angle)
                    departLine = plot([rungx1(i) rungx2(i)], [rungy1(i), rungy2(i)],'-','LineWidth',6,'color',[1 0 0]);
                else
                    plot([rungx1(i) rungx2(i)], [rungy1(i), rungy2(i)],'-','LineWidth',6,'color',[0.521 0.34 0.137]);
                end
            end
            
            % Set the axis to square and slightly larger than the radius of the
            % circle.
            axis([-1.2*this.radius 1.2*this.radius -1.2*this.radius 1.2*this.radius]);
            
            % Show the text that shows the train's speed.
            speedText = text(0, -.2, 'Speed (deg/sec) ~= 0', 'HorizontalAlignment', 'center');
            
            % Draw the images that don't get redrawn
            image([-0.07*this.radius 0.07*this.radius], [1.07*this.radius 0.93*this.radius], imgSign);
            image([-0.3*this.radius -0.1*this.radius], [-0.4*this.radius -0.6*this.radius], imgCow);
            image([0 0.3*this.radius], [-0.4*this.radius -0.6*this.radius], imgBarn);
            
            % Time to find the train's location
            x = this.radius * cosd(this.theta);
            y = this.radius * sind(this.theta);
            
            % Create the imgaes that will be used in the simulator so we
            % don't have to continually draw them
            imgTrainRight = image([x-this.radius*0.1 x+this.radius*0.1], [y+this.radius*0.1 y-this.radius*0.1], imgTrainRight);
            imgTrainLeft = image([x-this.radius*0.1 x+this.radius*0.1], [y+this.radius*0.1 y-this.radius*0.1], imgTrainLeft);
            imgGateUp = image([-0.1*this.radius 0.3*this.radius], [0.5*this.radius 0.1*this.radius], imgGateUp);
            imgGateDn = image([-0.1*this.radius 0.3*this.radius], [0.5*this.radius 0.1*this.radius], imgGateDn);
            
            % LED Images
            imgLEDGateLeftOff = image([-0.02*this.radius -0.12*this.radius], [0.6*this.radius 0.7*this.radius], imgLEDGateLeftOff);
            imgLEDGateLeftOn = image([-0.02*this.radius -0.12*this.radius], [0.6*this.radius 0.7*this.radius], imgLEDGateLeftOn);
            imgLEDGateRightOff = image([0.02*this.radius 0.12*this.radius], [0.6*this.radius 0.7*this.radius], imgLEDGateRightOff);
            imgLEDGateRightOn = image([0.02*this.radius 0.12*this.radius], [0.6*this.radius 0.7*this.radius], imgLEDGateRightOn);
            imgLEDApproachGreenOff = image([0.6*this.radius 0.7*this.radius], [0.05*this.radius -0.05*this.radius], imgLEDApproachGreenOff);
            imgLEDApproachGreenOn = image([0.6*this.radius 0.7*this.radius], [0.05*this.radius -0.05*this.radius], imgLEDApproachGreenOn);
            imgLEDApproachRedOff = image([0.45*this.radius 0.55*this.radius], [0.05*this.radius -0.05*this.radius], imgLEDApproachRedOff);
            imgLEDApproachRedOn = image([0.45*this.radius 0.55*this.radius], [0.05*this.radius -0.05*this.radius], imgLEDApproachRedOn);
            imgLEDDepartGreenOff = image([-0.55*this.radius -0.45*this.radius], [0.05*this.radius -0.05*this.radius], imgLEDDepartGreenOff);
            imgLEDDepartGreenOn = image([-0.55*this.radius -0.45*this.radius], [0.05*this.radius -0.05*this.radius], imgLEDDepartGreenOn);
            imgLEDDepartRedOff = image([-0.7*this.radius -0.6*this.radius], [0.05*this.radius -0.05*this.radius], imgLEDDepartRedOff);
            imgLEDDepartRedOn = image([-0.7*this.radius -0.6*this.radius], [0.05*this.radius -0.05*this.radius], imgLEDDepartRedOn);
            
            % Hide the images that aren't used at the beginning of the
            % simulator
            imgGateUp.Visible = 'off';
            imgGateDn.Visible = 'off';
            imgTrainRight.Visible = 'off';
            
            % LED Images
            imgLEDGateLeftOff.Visible = 'off';
            imgLEDGateLeftOn.Visible = 'off';
            imgLEDGateRightOff.Visible = 'off';
            imgLEDGateRightOn.Visible = 'off';
            imgLEDDepartGreenOff.Visible = 'off';
            imgLEDDepartGreenOn.Visible = 'off';
            imgLEDDepartRedOff.Visible = 'off';
            imgLEDDepartRedOn.Visible = 'off';
            imgLEDApproachGreenOff.Visible = 'off';
            imgLEDApproachGreenOn.Visible = 'off';
            imgLEDApproachRedOff.Visible = 'off';
            imgLEDApproachRedOn.Visible = 'off';
        end
        
        function update(this, ~, ~)
            this.move_train();
        end
        
        % This is the meat of the code and is what actually moves the
        % train. Actually this code does all of the plotting and puts
        % everything into the figure that the students look at. I will
        % break it down as we move through. The reason that this function
        % has a return value is to keep track of the angle of the train in
        % order to provide students with a correct response to any
        % "anlogReads" that they use on the break beam sensors.
        function move_train(this)
            global approachLine;
            global departLine;
            global speedText;
            global imgTrainLeft;
            global imgTrainRight;
            global imgGateUp;
            global imgGateDn;
            
            % Declare the global variables for the LED lights
            % (there's quite a few of them)
            global imgLEDGateLeftOff;
            global imgLEDGateLeftOn;
            global imgLEDGateRightOff;
            global imgLEDGateRightOn;
            global imgLEDDepartGreenOff;
            global imgLEDDepartGreenOn;
            global imgLEDDepartRedOff;
            global imgLEDDepartRedOn;
            global imgLEDApproachGreenOff;
            global imgLEDApproachGreenOn;
            global imgLEDApproachRedOff;
            global imgLEDApproachRedOn;
            
            % Some extra train properties
            lowEnd = 100;
            maxSpeed = 50;

            currentTime = clock;
            dt = etime(currentTime, this.lastUpdateTime);
            this.lastUpdateTime = currentTime;

            trainSpeed = max(min(maxSpeed*max((this.motorSpeeds(1) - lowEnd)/(255 - lowEnd), 0)^(0.7), maxSpeed), 0);

            % This will increment the angular position of the train by the
            % "train speed" that is passed into this function.
            this.theta = this.theta + min(max(trainSpeed * dt * this.motorDirections(1),-10), 10);
            
            % Now if theta is larger than 360, it means we have completed an
            % entire circuit and thus we need to reset it back to 0. 
            if (this.theta >= 360)
                this.theta = this.theta - 360;
            elseif (this.theta < 0)
                this.theta = this.theta + 360;
            end
            
            % Time to find the train's new location
            trainX = this.radius * cosd(this.theta);
            trainY = this.radius * sind(this.theta);
            
            % Move the train to its new position around the track
            if (this.motorDirections(1) ~= 0)
                if (this.theta < 180)
                    % This will simply change the direction of the train picture to
                    % make it look more realistic.
                    if (strcmp(imgTrainLeft.Visible, 'off'))
                        imgTrainLeft.Visible = 'on';
                        imgTrainRight.Visible = 'off';
                    end
                    imgTrainLeft.XData = [trainX-this.radius*0.1 trainX+this.radius*0.1];
                    imgTrainLeft.YData = [trainY+this.radius*0.1 trainY-this.radius*0.1];
                else
                    % This will simply change the direction of the train picture to
                    % make it look more realistic.
                    if (strcmp(imgTrainRight.Visible, 'off'))
                        imgTrainRight.Visible = 'on';
                        imgTrainLeft.Visible = 'off';
                    end
                    imgTrainRight.XData = [trainX-this.radius*0.1 trainX+this.radius*0.1];
                    imgTrainRight.YData = [trainY+this.radius*0.1 trainY-this.radius*0.1];
                end
            end
            
            % Set the speed text to show the correct speed of the train
            speedText.String = ['Speed (deg/sec) ~= ', num2str(trainSpeed)];
            
            % Change the colors of the approach line depending on what the
            % current theta angle is
            if (this.theta >= 350 || this.theta <= 10)
                % Only set the color property if it needs changed
                if (isequal(approachLine.Color, [1 0 0]))
                    approachLine.Color = [0 1 0];
                end
            else
                % Only set the color property if it needs changed
                if (isequal(approachLine.Color, [0 1 0]))
                    approachLine.Color = [1 0 0];
                end
            end
            
            % Change the colors of the departure line depending on what the
            % current theta angle is
            if (this.theta >=170 && this.theta <= 190)
                % Only set the color property if it needs changed
                if (isequal(departLine.Color, [1 0 0]))
                    departLine.Color = [0 1 0];
                end
            else
                % Only set the color property if it needs changed
                if (isequal(departLine.Color, [0 1 0]))
                    departLine.Color = [1 0 0];
                end
            end
            
            % If initialized, set the correct crossing gate image to
            % visible.
            if (this.servosAttached(1) == 1)
                if (this.servoValues(1) < 100)
                    if (strcmp(imgGateUp.Visible, 'off'))
                        imgGateUp.Visible = 'on';
                        imgGateDn.Visible = 'off';
                    end
                else
                    if (strcmp(imgGateDn.Visible, 'off'))
                        imgGateUp.Visible = 'off';
                        imgGateDn.Visible = 'on';
                    end
                end
            end
            
            % Here is where we do the LEDs. First we check to see if the
            % light is initialized. If the LED has never been initialized
            % then it will not turn on no matter how many times you make
            % the digitalWrite call. Once the LED is initialized, we call
            % the function that will actually turn the led on or off and
            % pass it the images of the led being on and off.
            %
            % The pins for the LEDs are as follows:
            % |-----------------------------------|
            % | PIN | LED                         |
            % |-----------------------------------|
            % |   6 | Approach Sensor Red LED     |
            % |   7 | Approach Sensor Green LED   |
            % |   8 | Departure Sensor Red LED    |
            % |   9 | Departure Sensor Green LED  |
            % |  15 | Left Crossing Gate LED      |
            % |  14 | Right Crossing Gate LED     |
            % |-----------------------------------|
            
            % If initialized, turn ON/OFF Approach Sensor Red LED
            if (this.digitalDir(6) == 1)
                turnLedOnOff(this.digitalState(6), imgLEDApproachRedOn, imgLEDApproachRedOff);
            end
            
            % If initialized, turn ON/OFF Approach Sensor Green LED
            if (this.digitalDir(7) == 1)
                turnLedOnOff(this.digitalState(7), imgLEDApproachGreenOn, imgLEDApproachGreenOff);
            end
            
            % If initialized, turn ON/OFF Departure Sensor Red LED
            if (this.digitalDir(8) == 1)
                turnLedOnOff(this.digitalState(8), imgLEDDepartRedOn, imgLEDDepartRedOff);
            end
            
            % If initialized, turn ON/OFF Departure Sensor Green LED
            if (this.digitalDir(9) == 1)
                turnLedOnOff(this.digitalState(9), imgLEDDepartGreenOn, imgLEDDepartGreenOff);
            end
            
            % If initialized, turn ON/OFF Left Crossing Gate LED
            if (this.digitalDir(15) == 1)
                turnLedOnOff(this.digitalState(14), imgLEDGateLeftOn, imgLEDGateLeftOff);
            end
            
            % If initialized, turn ON/OFF Right Crossing Gate LED
            if (this.digitalDir(14) == 1)
                turnLedOnOff(this.digitalState(15), imgLEDGateRightOn, imgLEDGateRightOff);
            end
        end
    end
end


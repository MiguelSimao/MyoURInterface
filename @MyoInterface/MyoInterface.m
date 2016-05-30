classdef MyoInterface < handle
    % MyoInterface Myo-UR10 interfacing object
    %   This class retrieves posture data from a Myo object and determines
    %   the commands to be sent through the UR10Controller object to the
    %   physical robot.
    %
    % Usage example:
    %
    %  h = MyoInterface;    % instantiate the interface 
    %  initialize(h);       % attempts to connect to the devices
    %  toggle(h)            % controls whether commands are sent or not
    %  terminate(h);
    
    properties
        verbose = false
        cruiseTimeout = 5
        
    end 
    properties (SetAccess = private)
        hMyoMex
        % Handle to the MyoMex-SDK interface
        hMyoData
        % Handle to the MyoData object
        hUR10
        % Handle to the UR10 object
        
        allowCommands = false
        % Controls the sending of commands to the UR object
        
        pose_log
        % Saves all poses
        time_log
        % Saves all timestamps
        event_log
        % Saves pose change events, their timestamp and some attributes
        outputGestures
        % Saves gesture history
        
        % METADATA
        pitch
        urHeading = 0 % rad
        speedHeading = 0.0872 % rad/s | Def: 0.0872 (5 deg/s)
        % saves the output gestures, which may be a combination of poses

    end
    properties (Access = private, Hidden = true)
        
        urState
        headingUpdateTimer 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % The following variables should not be changed:
        poseRate = 0.04 %seconds
        logSize  = 10   %seconds
        newPose = false
        
        gesture_labels ={...
                        'rest',...
                        'long-fist',...
                        'long-left',...
                        'long-right',...
                        'open-hand',...
                        'double-tap',...
                        'double-fist',...
                        'double-left',...
                        'double-right'}
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end
    %% Events
    events
        newCommand
        gestureTimeout
    end%events
            
    methods
        %% Object Management
        function this = MyoInterface
            % MyoInterface Construter
            %
            % Inputs:
            % 
            % Outputs:
            %   this - Handle to MyoInterface instance
            %
            N = this.logSize / this.poseRate;
            this.pose_log = zeros(N,1);
            this.time_log = zeros(N,1);
            this.event_log = zeros(50,4); %sampleNumber x (time, pose, duration, duration-attribute)
            this.outputGestures = zeros(50,2);
            
            addlistener(this,'newCommand',@this.newCommandFcn);
            addlistener(this,'gestureTimeout',@this.gestureTimeoutFcn);
            
            this.hUR10 = UR10Controller;
        end
        function terminate( obj )
            obj.hMyoData.stopStreaming();
            pause(0.1);
            obj.hMyoMex.delete();
            clear obj.hMyoMex
         end
        function toggle(obj)
            obj.allowCommands = ~obj.allowCommands;
            if obj.allowCommands
                fprintf('Now sending commands.');
            else
                fprintf('Halted sending commands.');
            end
        end
        %% VARIABLE SETTING
        function set.cruiseTimeout(obj,value)
            
        end
        %% INITIALIZATION ROUTINE
        function initialize( this )
            % I have to initialize both the Myo and the UR
            % Launch MyoMex with 1 Myo:
            this.hMyoMex = MyoMex(1);
            % Get data handle:
            this.hMyoData = this.hMyoMex.myoData(1);
            this.hMyoData.newDataFcn = @(src,evt)newDataFcn(src,evt,this);
            pause(0.2);
            this.hMyoData.startStreaming();

            % Initialize UR10Controller:
            % initialize(this.hUR10);
            
            %% MYO ACQUISITION DATA FUNCTION
            function newDataFcn(objMyo,~,objInt,~)
                % This function is called by the Myo Object every 0.04s
                % (0) pose_rest_log
                % (1) pose_fist_log
                % (2) pose_wave_in_log
                % (3) pose_wave_out_log
                % (4) pose_fingers_spread_log
                % (5) pose_double_tap_log
                % (6) pose_unknown_log                
                % Some variables:
                % pose_log stores every pose acquired
                % event_log stores all the gesture transitions

                if objMyo.pose > 6 
                    return; end % Remove initial frames w/ unknown gestures
                
                % Update pose_log and time_log, circular buffers. First
                % frame is the newest:
                objInt.pose_log(2:end) = objInt.pose_log(1:end-1);
                objInt.pose_log(1) = objMyo.pose;
                objInt.time_log(2:end) = objInt.time_log(1:end-1);
                objInt.time_log(1) = objMyo.timeIMU;
                
                % Defining transition events:
                if objInt.pose_log(1) ~= objInt.event_log(1,2)
                    % Occurs when a NEW POSE is detected. Pose_log(1) is
                    % the first frame of a new gesture. Event_log(1,:) is
                    % the last frame of the previous gesture.
                    % 1. Update the event_log with a new line
                    objInt.event_log(2:end,:) = objInt.event_log(1:end-1,:);
                    % 2. Save new pose on (1,1)
                    objInt.event_log(1,2) = objInt.pose_log(1);
                    % 3. Save the new gesture timestamp on (1,2)
                    objInt.event_log(1,1) = objInt.time_log(1);
                    % 4. Estimate the old gesture duration and save on (2,3)
                    % objInt.event_log(2,3) = objInt.event_log(1,3) - objInt.event_log(2,3);
                    
                    %  I could also be here without if the system didn't
                    %  find a long gesture.
                    if objInt.newPose
                        % Being here means that the previous gesture wasn't
                        % a long one. Therefore, it was a short one. Have
                        % to save it in the output buffer.
                        %  outputGestures (x,1) - pose number
                        %  outputGestures (x,2) - duration attribute
                        %                           0 - short
                        %                           1 - long
                        % Update outputGestures circular buffer:
                        objInt.outputGestures(2:end,:) = objInt.outputGestures(1:end-1,:);
                        objInt.outputGestures(1,1) = objInt.event_log(2,2);
                        objInt.outputGestures(1,2) = 0;
                        objInt.newPose = false;
                        
                        notify(objInt,'newCommand');
                        % I can check here if I have a short gesture
                        % sequence, by looking into outputGesture(1:3,1:2).
                        % A double tap means that three specific short
                        % gestures occured.
                    end
                    objInt.newPose = true;
                    
                else
                    % This occurs MID-GESTURE.
                    % Current duration    = currentTime - newPoseTime
                    objInt.event_log(1,3) = objInt.time_log(1) - objInt.event_log(1,1);
                    % Update duration attribute:
                    objInt.event_log(1,4) = objInt.event_log(1,3) >= 0.5; % seconds
                    % If (1,3) > 0.8, the gesture is long, otherwise it's
                    % short.
                    
                    if ( objInt.newPose && objInt.event_log(1,4) ) 
                        % Short to long transition, which means this
                        % gesture is a long gesture.
                        %  outputGestures (x,1) - pose number
                        %  outputGestures (x,2) - duration attribute
                        %                           0 - short
                        %                           1 - long
                        
                        % The following 3 lines update the circular buffer
                        objInt.outputGestures(2:end,:) = objInt.outputGestures(1:end-1,:);
                        objInt.outputGestures(1,1) = objInt.event_log(1,2);
                        objInt.outputGestures(1,2) = 1;
                        objInt.newPose = false;
                        
                        notify(objInt,'newCommand');

                    end
                end %newPose

            end %newDataFcn
        end
        function newCommandFcn( obj, ~, ~ )
            % This is a function that is called when a new gesture is made.
            % 'obj' is the MyoInterface handle.
            % obj.outputGestures(1,1) gives the latest gesture made.
            % obj.outputGestures(1,2) gives the gesture attribute, that is
            % short or long.
            
            % Action discrimination:
            if min(obj.outputGestures(1,1:2) == [0 1]) == 1
                % Gesture 1: rest
                % fprintf('%s\n',obj.gesture_labels{1});
                gesture = 1;
            elseif min(obj.outputGestures(1,1:2) == [1 1]) == 1    
                % Gesture 2: long-fist
                % fprintf('%s\n',obj.gesture_labels{2});
                gesture = 2;
            elseif min(obj.outputGestures(1,1:2) == [3 1]) == 1    
                % Gesture 3: long-left
                % fprintf('%s\n',obj.gesture_labels{3});
                gesture = 3;
            elseif min(obj.outputGestures(1,1:2) == [2 1]) == 1    
                % Gesture 4: long-right
                % fprintf('%s\n',obj.gesture_labels{4});
                gesture = 4;
            elseif obj.outputGestures(1,1) == 4
                % Gesture 5: open-hand
                % fprintf('%s\n',obj.gesture_labels{5});
                gesture = 5;
            elseif obj.outputGestures(1,1) == 5
                % Gesture 6: double-tap
                % fprintf('%s\n',obj.gesture_labels{6});
                gesture = 6;
            elseif min(obj.outputGestures(1:3,1:2) == [1 0; 0 0; 1 0]) == 1
                % Gesture 7: double-fist
                % fprintf('%s\n',obj.gesture_labels{7});
                gesture = 7;
            elseif min(obj.outputGestures(1:3,1:2) == [3 0; 0 0; 3 0]) == 1
                % Gesture 8: double-left
                % fprintf('%s\n',obj.gesture_labels{8});
                gesture = 8;
            elseif min(obj.outputGestures(1:3,1:2) == [2 0; 0 0; 2 0]) == 1
                % Gesture 9: double-right
                % fprintf('%s\n',obj.gesture_labels{9});
                gesture = 9;
            end
            fprintf('%s\n',obj.gesture_labels{gesture});
            
            if obj.verbose
                if obj.outputGestures(1,2)
                    fprintf(' Long: %i | %.2f \n',obj.outputGestures(1,1),...
                                obj.event_log(1,3));
                else
                    fprintf('Short: %i | %.2f \n',obj.outputGestures(1,1),...
                                obj.event_log(2,3));
                end
            end
            
            if obj.allowCommands
                sendCommand; end
            
            function sendCommand
                % The goal here is to actually use the detected gestures
                
                switch gesture
                    case 1 % rest
                    case 2 % long-fist
                        
                    case 3 % long-left
                    case 4 % long-right
                    case 5 % open-hand
                    case 6 % double-tap
                    case 7 % double-fist
                    case 8 % double-left
                    case 9 % double-right
                end
            end
            
        end %event:callback
%% CALLBACK FUNCTIONS
        function gestureTimeoutFcn(src,~,~,~)
            % Function to be called when an action-command times out.
            
        end
        function headingUpdate(src,~,~,~,dir)
            src.urHeading = src.urHeading + dir * src.speedHeading;
        end
        
    end %methods
end %classdef


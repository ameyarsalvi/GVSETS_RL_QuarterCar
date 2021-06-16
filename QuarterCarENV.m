classdef QuarterCarENV < rl.env.MATLABEnvironment
    
    
    %% Properties (set properties' attributes accordingly)
    properties
        % Initialize system state [x,dx,theta,dtheta]'
        State = [0;0;1;0];
        Reward = 0;
        steps = 1;
        episodes = 1;
        
        
        TrackHorizontalVel = 20;
        TrackTorque = 700;
        TrackStiff =1;
        h;
        
        track_error = 15;
        torqueerror = 120;
        stifferror = 0;
        OscError = 0;
        stif;
        
        hamp3;
        
    end
    
    properties(Access = protected)
        % Initialize internal flag to indicate episode termination
        IsDone = false        
    end

    %% Necessary Methods
    methods              
        % Contructor method creates an instance of the environment
        % Change class name and constructor name accordingly
        function this = QuarterCarENV()
            % Initialize Observation settings
            ObservationInfo = rlNumericSpec([12 1]);
            ObservationInfo.Name = 'Sprung Mass States';
            ObservationInfo.Description = 'h_X_vector, x, dx, y, dy, Desired V';
            
            % Initialize Action settings   
            ActionInfo = rlNumericSpec([2 1],'LowerLimit',[0;-10000] ,'UpperLimit',[1000;10000]);
            %ActionInfo = rlNumericSpec(1,'LowerLimit',1 ,'UpperLimit',5);
            ActionInfo.Name = 'Wheel Torque';
            ActionInfo.Description = 'T';
            
            
            % The following line implements built-in functions of RL env
            this = this@rl.env.MATLABEnvironment(ObservationInfo,ActionInfo);
            
            % Initialize property values and pre-compute necessary values
            %updateActionInfo(this);
        end
        
        % Apply system dynamics and simulates the environment with the 
        % given action for one step.
        function [Observation,Reward,IsDone,LoggedSignals] = step(this,Action)
            LoggedSignals = [];
            
            
            % Get action
            Force = double(Action(1));
            stiff = double(Action(2));
            this.stif = stiff;
            
 
            
            
            Torque = Force;
            Stiffness = stiff;
            
            if this.State(1) > 300
                this.hamp3 = 0;
            else
                this.hamp3 = 0;
            end
            
            new_states=QuarterCar(this.State,Torque,Stiffness,this.hamp3);
            this.steps =this.steps +1;
            new_states=real(new_states(end,:));
            x = new_states(1);
            %if new_states(2)>0
            x_vec = [x x+0.1 x+0.2 x+0.3 x+0.4 x+0.5];
            %else
             %   x_vec = [x x-0.01 x-0.02 x-0.03 x-0.04 x-0.05];
            %end
            %h_x = 0.1*cos(0.5*x_vec)';
            h_x = 0.3*cos(0.4082*x_vec) + 0.1*cos(0.752*x_vec) +this.hamp3*cos(0.503*x_vec) +0.2*cos(0.8164*x_vec);
            h_x =h_x';
            %h_x = (0.1*cos(0.5*x_vec) + 1*cos(0.05*x_vec))';
            this.h = h_x(1);
            x_dot = new_states(2);
            y = new_states(3);
            y_dot = new_states(4);
            
            if this.steps > 0 && this.steps < 500
               this.TrackTorque =700;
               this.TrackHorizontalVel = 20;
            elseif this.steps >= 500
               this.TrackTorque =120;
                this.TrackHorizontalVel = 10;
            end
            
            this.track_error = abs(this.TrackHorizontalVel -x_dot);
            %OscError = abs(((this.h+0.4+0.5-((300*9.8)/(this.stif*25000))) - this.State(3)));
            
            %Y = this.h + 0.9 - ((9.8*300)/(10000 + Stiffness)) - this.State(3);
            Y = 1- this.State(3);
            this.OscError = Y;
            %this.OscError = this.State(4);
            this.torqueerror = abs(Force - this.TrackTorque);
            this.stifferror = abs(stiff - this.TrackStiff);
            
            
            
            % Update Observation to be fed to agent for training
            %Observation = [h_x;x;x_dot;y;y_dot;this.track_error;this.OscError];
            Observation = [h_x;x;x_dot;y;y_dot;this.track_error;this.OscError];

            % Update system states
            this.State = [x;x_dot;y;y_dot];
            
            % Get reward
            Reward = getReward(this);
            
            fprintf('x_dot = %0.2f \tx_dot_track = %0.2f \ty_e = %0.2f \tT = %0.2f \tK = %0.2f \n',x_dot,this.TrackHorizontalVel,this.OscError,Torque,Stiffness)
            
            % Check terminal condition
            
            IsDone = this.steps > 1000;
            this.IsDone = IsDone;

        end
        
        % Reset environment to initial state and output initial observation
        function InitialObservation = reset(this)
           
            x = 0;
            x_vec = [x x+0.1 x+0.2 x+0.3 x+0.4 x+0.5];
            %h_x = (0.1*cos(0.5*x_vec))';
            %h_x = (0.1*cos(0.5*x_vec) + 1*cos(0.05*x_vec))';
            h_x = 0.3*cos(0.4082*x_vec) + 0.1*cos(0.752*x_vec) +0*cos(0.503*x_vec) +0.2*cos(0.8164*x_vec);
            h_x =h_x';
            x_dot = 0;
            %y = h_x(1) + 0.9 - ((9.8*300)/(10000));
            y=1;
            y_dot = 0;
            
            
            Observation = [h_x;x;x_dot;y;y_dot;this.track_error;this.OscError];
            InitialObservation = Observation;
            this.State = [x;x_dot;y;y_dot];
            this.Reward = 0;
            this.steps = 1;
            this.episodes = this.episodes +1;
            
            this.track_error = 20;
            this.torqueerror = 120;
            this.stifferror = 0;
            this.OscError = 0;
            
            
        end
    end
    %% Optional Methods (set methods' attributes accordingly)
    methods               
        % Reward function
        function Reward = getReward(this)
            
            %reward = -0.01*((this.torqueerror).^2 + (this.track_error).^2);
            reward = -0.01*(this.track_error.^2 + 1000*this.OscError.^2 + (this.torqueerror).^2) ;
            %reward = -1*(this.OscError.^2);
            this.Reward = reward;
            Reward = this.Reward;
        end   
    end
    
    methods (Access = protected)
        end
    end


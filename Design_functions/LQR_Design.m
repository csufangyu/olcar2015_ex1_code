function [LQR_Controller, Cost] = LQR_Design(Model,Task)
% LQR_DESIGN Creates an LQR controller to execute the Task
% Controller: Controller structure
%             u(x) = theta([t]) ' * BaseFnc(x)
%             .theta:   Parameter matrix (each column is associated to one
%                       control input)
%             .BaseFnc
%             .time

%% Problem 1.1: Find optimal feedback gains according to an LQR controller
% Make use of the elements in Task.cost for linearization points and cost
% functions.

% Define the state input around which the system dynamics should be linearized    
% x_lin = ...
% u_lin = ... % assume that most of the time the quadrotor flies similar to hovering (Fz != 0)
  
% The linearized system dynamics matrices A_lin B_lin describe the dynamics
% of the system around the equilibrium state. See exercise sheet Eq.(2) for 
% usage example
% A_lin = ...
% B_lin = ...

% Compute optimal LQR gain (see command 'lqr')
% K = ...

x_lin = Task.goal_x;%目标状态
u_lin = Task.cost.u_eq;%假设重力加速度为9.81
A_lin = Model.Alin{1}(x_lin,u_lin,Model.param.syspar_vec);
B_lin = Model.Blin{1}(x_lin,u_lin,Model.param.syspar_vec);
[K,P,r] = lqr(A_lin,B_lin,Task.cost.Q_lqr,Task.cost.R_lqr);
%% Design the actual controller using the optimal feedback gains K
% The goal of this task is to correctly fill the matrix theta.
LQR_Controller.BaseFnc = @Augment_Base;
LQR_Controller.time    = Task.start_time:Task.dt:(Task.goal_time-Task.dt);
% The quadrotor controller produces inputs u from a combination of 
% feedforward uff and feedback elements as follows:
%
%   u = [Fz, Mx, My, Mz]' = uff + K'(x_ref - x)
%                         = uff + K'x_ref - K'x
%                         = [uff + K'x_ref, -K' ]' * [1,x']'
%                         =        theta'          * BaseFnc

lqr_type = 'goal_state';  % Choose 'goal_state or 'via_point'
fprintf('LQR controller design type: %s \n', lqr_type);
Nt = ceil((Task.goal_time - Task.start_time)/Task.dt+1);   
switch lqr_type
    case 'goal_state'
        %% Problem 1.2: Drive system to Task.goal_x with LQR Gain + feedforward
        % Define equilibrium point x_eq correctly and use it to generate
        % feedforward torques (see exercise sheet Eq.(3)).
        % x_ref = ...          % reference point the controller tries to reach
        % theta = ...          % dimensions (13 x 4)
        
        % stack constant theta matrices for every time step Nt
        theta =  [ Task.cost.u_eq + K * Task.goal_x , -K]';
        LQR_Controller.theta = repmat(theta,[1,1,Nt]);
        
    case 'via_point'  
        %% Problem 1.3: Drive system to Task.goal_x through via point p1.
        % (see exercise sheet Eq.(3))
        
        t1 = Task.vp_time;
        p1 = Task.vp1;           % steer towards this input until t1
        x_lin_p1 = p1;%目标状态
        u_lin_p1 = Task.cost.u_eq;%假设重力加速度为9.81
        A_lin_p1 = Model.Alin{1}(x_lin_p1,u_lin_p1,Model.param.syspar_vec);
        B_lin_p1 = Model.Blin{1}(x_lin_p1,u_lin_p1,Model.param.syspar_vec);
        [K_p1,P,r] = lqr(A_lin_p1,B_lin_p1,Task.cost.Q_lqr,Task.cost.R_lqr);
        for t=1:Nt-1
        %    ...
        %    % varying theta matrices for every time step Nt 
            if t*Task.dt<=Task.vp_time
                LQR_Controller.theta(:,:,t) =[Task.cost.u_eq + K_p1 * p1 ,-K_p1]';
            else
                LQR_Controller.theta(:,:,t) = [ Task.cost.u_eq + K * Task.goal_x , -K]';
            end
        end
        % for t=1:Nt-1
        %    ...
        %    % varying theta matrices for every time step Nt 
        %    LQR_Controller.theta(:,:,t) = ...
        % end
    case 'two_point'  
        %%
        % (see exercise sheet Eq.(3))
        
        t1 = Task.vp_time;
        p1 = Task.vp1;           % steer towards this input until t1
        x_lin_p1 = p1;%目标状态
        u_lin_p1 = Task.cost.u_eq;%假设重力加速度为9.81
        A_lin_p1 = Model.Alin{1}(x_lin_p1,u_lin_p1,Model.param.syspar_vec);
        B_lin_p1 = Model.Blin{1}(x_lin_p1,u_lin_p1,Model.param.syspar_vec);
        [K_p1,P,r] = lqr(A_lin_p1,B_lin_p1,Task.cost.Q_lqr,Task.cost.R_lqr);
        t2 = Task.goal_time/2;
        p2 = Task.vp11;           % steer towards this input until t1
        x_lin_p2 = p2;%目标状态
        u_lin_p2 = Task.cost.u_eq;%假设重力加速度为9.81
        A_lin_p2 = Model.Alin{1}(x_lin_p2,u_lin_p2,Model.param.syspar_vec);
        B_lin_p2 = Model.Blin{1}(x_lin_p2,u_lin_p2,Model.param.syspar_vec);
        [K_p2,P,r] = lqr(A_lin_p2,B_lin_p2,Task.cost.Q_lqr,Task.cost.R_lqr);
        for t=1:Nt-1
            all_time_now=t*Task.dt;
        %    ...
        %    % varying theta matrices for every time step Nt 
            if all_time_now<=Task.vp_time
                LQR_Controller.theta(:,:,t) =[Task.cost.u_eq + K_p1 * p1 ,-K_p1]';
            elseif Task.vp_time<all_time_now && all_time_now<t2
                LQR_Controller.theta(:,:,t) =[Task.cost.u_eq + K_p2 * p2 ,-K_p2]';
            else
                LQR_Controller.theta(:,:,t) = [ Task.cost.u_eq + K * Task.goal_x , -K]';
            end
        end
        
    otherwise 
           error('Unknown lqr_type');
end
% Calculate cost of rollout
sim_out = Quad_Simulator(Model, Task, LQR_Controller);
Cost = Calculate_Cost(sim_out, Task);
end


function x_aug = Augment_Base(t,x)
% AUGMENT_BASE(t,x) Allows to incorporate feedforward and feedback
%
%   Reformulating the affine control law allows incorporating
%   feedforward term in a feedback-like fashion:
%   u = [Fz, Mx, My, Mz]' = uff + K(x - x_ref)
%                         = uff - K*x_ref + K*x
%                         = [uff - K*x_ref, K ] * [1,x']'
%                         =        theta'       * BaseFnc 

number_of_states = size(x,2);      % multiple states x can be augmented at once
x_aug = [ones(1,number_of_states); % augment row of ones
                    x           ];
end


function Cost = Calculate_Cost(sim_out, Task)
% Calculate_Cost(.) Asses the cost of a rollout sim_out 
%
%   Be sure to correctly define the equilibrium state (X_eq, U_eq) the 
%   controller is trying to reach.
X = sim_out.x;
U = sim_out.u;
Q = Task.cost.Q_lqr;
R = Task.cost.R_lqr;
X_eq = repmat(Task.cost.x_eq,1,size(X,2)-1); % equilibrium state LQR controller tries to reach
U_eq = repmat(Task.cost.u_eq,1,size(U,2));   % equilibrium input LQR controller tries to reach
Ex = X(:,1:end-1) - X_eq;                    % error in state
Eu = U - U_eq;                               % error in input

Cost = Task.dt * sum(sum(Ex.*(Q*Ex),1) + sum(Eu.*(R*Eu),1));
end






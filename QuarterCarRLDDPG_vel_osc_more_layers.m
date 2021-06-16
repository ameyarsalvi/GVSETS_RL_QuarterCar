clc
clear all
env=QuarterCarENV();
validateEnvironment(env);


observationInfo = getObservationInfo(env);
numObservations = observationInfo.Dimension(1);
actionInfo = getActionInfo(env);
numActions = actionInfo.Dimension(1);
%rng(0)

%numObs = prod(observationInfo.Dimension);
%observationInfo.Name = 'observations';

%numAct = prod(actionInfo.Dimension);
%actionInfo.Name = 'thrusts';

%load('VelocityOscAgent2.mat');
%load('VelocityOscAgent_myQC_Fy.mat')
%%


L = 100; % number of neurons
statePath = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(L,'Name','fc2')
    additionLayer(2,'Name','add')
    reluLayer('Name','relu2')
    fullyConnectedLayer(100,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(100,'Name','fc4')
    reluLayer('Name','relu4')
    fullyConnectedLayer(75,'Name','fc5')
    reluLayer('Name','relu5')
    fullyConnectedLayer(50,'Name','fc6')
    reluLayer('Name','relu6')
    fullyConnectedLayer(25,'Name','fc7')
    reluLayer('Name','relu7')
    fullyConnectedLayer(10,'Name','fc8')
    reluLayer('Name','relu8')
    fullyConnectedLayer(1,'Name','fc9')];

actionPath = [
    featureInputLayer(numActions,'Normalization','none','Name','action')
    fullyConnectedLayer(L,'Name','fc10')];

criticNetwork = layerGraph(statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
    
criticNetwork = connectLayers(criticNetwork,'fc10','add/in2');

%%

figure
plot(criticNetwork)

%%

criticOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',1e-4);

critic = rlQValueRepresentation(criticNetwork,observationInfo,actionInfo,...
    'Observation',{'observation'},'Action',{'action'},criticOptions);

%%

actorNetwork = [
    featureInputLayer(numObservations,'Normalization','none','Name','observation')
    fullyConnectedLayer(L,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(100,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(100,'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(75,'Name','fc4')
    reluLayer('Name','relu4')
    fullyConnectedLayer(50,'Name','fc5')
    reluLayer('Name','relu5')
    fullyConnectedLayer(25,'Name','fc6')
    reluLayer('Name','relu6')
    fullyConnectedLayer(10,'Name','fc7')
    reluLayer('Name','relu7')
    fullyConnectedLayer(numActions,'Name','fc8')
    tanhLayer('Name','tanh1')
    scalingLayer('Name','ActorScaling1','Scale',[500;10000],'Bias',[500;0])];
%%
figure
plot(actorNetwork)
%%

actorOptions = rlRepresentationOptions('LearnRate',1e-4,'GradientThreshold',1,'L2RegularizationFactor',1e-4);
actor = rlDeterministicActorRepresentation(actorNetwork,observationInfo,actionInfo,...
    'Observation',{'observation'},'Action',{'ActorScaling1'},actorOptions);

%%

agentOptions = rlDDPGAgentOptions(...
    'SampleTime',0.02,...
    'TargetSmoothFactor',1e-3,...
    'ExperienceBufferLength',10000,...
    'DiscountFactor',0.99,...
    'MiniBatchSize',64);

agentOptions.NoiseOptions.Variance = [750;7000];
agentOptions.NoiseOptions.VarianceDecayRate = 1e-5;
agentOptions.ResetExperienceBufferBeforeTraining = true;
%%
%agentOptions.ResetExperienceBufferBeforeTraining = false;

agent = rlDDPGAgent(actor,critic,agentOptions);
%%
maxepisodes = 750;
maxsteps = 1000;
trainingOpts = rlTrainingOptions('MaxEpisodes',maxepisodes,'MaxStepsPerEpisode',maxsteps,'Verbose',true,'StopTrainingCriteria','EpisodeReward','StopTrainingValue',10,'Plots',"training-progress");

%'SaveAgentCriteria',"EpisodeReward",....
 %   'SaveAgentValue',-5000,....
  %  'SaveAgentDirectory', "/home/asalvi/Work/VIPR GS/GVSETS Paper/DDPG Simplified/DDPG_C_T/Paper" + "\run1\Agents");

%%
trainingStats = train(agent,env,trainingOpts);

%%

save("VelocityOscAgentFreq2_neg2vel.mat",'agent')

%%
%load('VelocityOscAgentP1.mat')

simOpts = rlSimulationOptions('MaxSteps',1000);
experience = sim(env,agent,simOpts);
%%
timeData = experience.Observation.SprungMassStates.Time;
stateData = experience.Observation.SprungMassStates.Data;
actionData = experience.Action.WheelTorque.Data;  

%load('sim_states.mat')
%load('time.mat')
%load('actionData.mat')

save('sim_states_T_C2.mat','stateData')
save('time_T_C2.mat','timeData')
save('actionData_T_C2.mat','actionData')

%%


figure
plot(stateData(7,:))
title('Horizontal Displacement')
grid on
ylabel('m')
xlabel('steps (total 100 Seconds)')

for i=1:1000
    if i<500
        Ter(i) = 26;
    else
        Ter(i) = 11;
    end
end

figure
plot(stateData(8,:))
title('Horizontal velocity')
grid on
ylabel('m/s')
xlabel('steps (total 100 Seconds)')
hold on
plot(Ter)

h=0.1.*cos(0.5*stateData(7,:));

%h=0.1.*cos(0.5*stateData(7,:)) + 1.*cos(0.05*stateData(7,:));
%SD =(300*9.8/());
%SD =[SD,0.1176];
Y = 1- stateData(9,:);

figure
plot(Y)
title('Vertical Displacement')
grid on
ylabel('m')
xlabel('steps (total 100 Seconds)')



figure
plot(h)
title('Terrain')
grid on


figure
plot(stateData(10,:))
title('Vertial Velocity')
grid on
ylabel('m/s')
xlabel('steps (total 100 Seconds)')


figure
plot(actionData(1,:))
title('Input Torque')
grid on
%axis([0 1000 0 1000])
ylabel('Nm')
xlabel('steps (total 100 Seconds)')

figure
plot(15000 + actionData(2,:))
title('Stiffness')
grid on
%axis([0 1000 5000 25000])
ylabel('N/m')
xlabel('steps (total 100 Seconds)')

%{
figure
plot(actionData(3,:))
title('Damping')
grid on
%}
%%

h=0.1.*cos(0.5*stateData(7,:));
SD =(300*9.8)./(actionData(2,:).*25000);
SD =[SD,0.1176];
Y = h + 0.9 -SD - stateData(9,:);


%%
save("sim_states.mat",'stateData')
save("time.mat",'timeData')
save("actionData.mat",'actionData')
%}
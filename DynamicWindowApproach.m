function [u,trajDB]=DynamicWindowApproach(x,model,goal,evalParam,ob,R)
%Function to calculate the input value by DWA

%Dynamic Window[vmin,vmax,ƒÖmin,ƒÖmax]
Vr=CalcDynamicWindow(x,model);
%Calculation of Merit Function
[evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam);

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

%Normalization of each merit function
evalDB=NormalizeEval(evalDB);

%Calculation of final rating value
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;evalParam(1:3)*evalDB(id,3:5)'];
end
evalDB=[evalDB feval];

[maxv,ind]=max(feval);%Calculate the index of the input value with the highest evaluation value
u=evalDB(ind,1:2)';%Return input value with high evaluation value
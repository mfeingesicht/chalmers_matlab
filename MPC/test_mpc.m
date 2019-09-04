mpcObj = mpc(ss(A,B,C,D),Ts);
mpcObj.OutputVariables(1).Min = deg2rad(-30);mpcObj.OutputVariables(1).Max = deg2rad(30);
mpcObj.OutputVariables(2).Min = deg2rad(-30);mpcObj.OutputVariables(2).Max = deg2rad(30);
mpcObj.OutputVariables(3).Min = deg2rad(-15);mpcObj.OutputVariables(3).Max = deg2rad(15);
mpcObj.ManipulatedVariables.Min = deg2rad(-15);mpcObj.ManipulatedVariables.Max = deg2rad(15);

mpcObj.Weights.ManipulatedVariables = 1;
mpcObj.Weights.ManipulatedVariablesRate = 0;
mpcObj.Weights.OutputVariables = [1 1 1];

mpcObj

% exmpcObj_range = generateExplicitRange(mpcObj);
% exmpcObj_range.State.Min = [deg2rad(-90) deg2rad(-90) deg2rad(-90)];
% exmpcObj_range.State.Max = [deg2rad(90) deg2rad(90) deg2rad(90)];
% exmpcObj_range.Reference.Min = [deg2rad(-30) deg2rad(-30) deg2rad(-30)];
% exmpcObj_range.Reference.Max = [deg2rad(30) deg2rad(30) deg2rad(-30)];
% exmpcObj_range.MeasuredDisturbance.Min = [];
% exmpcObj_range.MeasuredDisturbance.Max = [];
% exmpcObj_range.ManipulatedVariable.Min = [deg2rad(-30)];
% exmpcObj_range.ManipulatedVariable.Max = [deg2rad(30)];
% 
% exmpcObj = generateExplicitMPC(mpcObj,exmpcObj_range)
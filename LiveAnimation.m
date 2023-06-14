% Copyright Maciej Lacki 2019
% All rights reserved 

function [s] = LiveAnimation()
%LIVEANIMATION Enables animations in Live Scripts 
s = settings;
s.matlab.editor.AllowFigureAnimation.PersonalValue = true;
end


% Copyright Maciej Lacki 2018
% All rights reserved 

function[out] = HalfSub(eq, var, t, simp)
%HALFSUB   Half tangent substitution and simplification
%   HALFSUB(Equation,Variable) replaces all instances of cos or sin 
%   Variable, with half tangent subtitution. By deafult the substituting 
%   varaible will be t, and the output will be simplified. 
%
%   HALFSUB(Equation,Variable,Substitute) Substitute is a symbolic variable
%   used instead of deafult t.
%
%   HALFSUB(Equation,Variable,Substitute,Simpl) Simpl is a boolean
%   controlling the simplification of the output expression. If boolean is
%   0 the equation will not be simplified, otherwise it will be simplified
%   with Simpl steps
if nargin == 1
    disp("Insuficient input")
elseif nargin == 2
    syms t;
    simp = 100;
elseif nargin == 3
    simp = 100;
end
eqe = expand(eq); 
sinsub = subs(eqe,sin(var),2*t/(1+t^2));
cossub = subs(sinsub, cos(var),(1-t^2)/(1+t^2));
tansub = subs(cossub, tan(var), tan(var/2));
if simp ~= 0
    out = simplify(expand(tansub),'steps',simp);
else
    out = tansub;
end

end
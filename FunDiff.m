% Copyright Maciej Lacki 2018
% All rights reserved 

function [eq] = FunDiff(eqIn,Var)
% FUNDIFF returns a partial derrivitive of EQIN in terms of VAR
syms u(t) 
assume(u(t),{'real'})
eqt = subs(eqIn,Var,u);
eqd = functionalDerivative(eqt,u);
eq = subs(eqd,u,Var);
end


function [xnext, y] = idm_dt_nl(~,x, u, v0, T_h, s0, a, b, delta, Llead, Ts,varargin)
% States: x=[xE; vE]; Inputs: u=[xL; vL]; Output: y=vE
xE = x(1);
vE = x(2);
xL = u(1); 
vL = u(2);

% gap and closing rate
s  = xL - xE - Llead; %gap
dv = vE - vL;

% IDM using continuius idm to get discrete time state update for sys id toolbox
p = struct('v0',v0,'T',T_h,'s0',s0,'a',a,'b',b,'delta',delta);
acc = idm_dvdt(vE, max(s,1e-3), dv, p);

% Euler step
vE2 = max(0, vE + Ts*acc);
xE2 = xE + Ts*vE;

xnext = [xE2; vE2];
y = vE2;
end
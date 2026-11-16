% GEN6323 - Chapitre 4, seance 4 : arret par MPC a horizon glissant.
% Necessite Optimization Toolbox (quadprog). Unites SI, Ts = 1 s.
% Pour reproduire les diapositives, comparer N = 1 et N = 2.
clear; clc; close all;
N = 2;                  % Horizon : 1 ou 2 (peut etre augmente ici)
Ts = 1; rho = 0.1; vref = 2;
x = 0; v = 2; xlim = 2;
assert(N >= 1 && N == floor(N) && rho > 0);
assert(exist('quadprog','file') == 2, ...
    'Ce script demande quadprog (Optimization Toolbox). Le HTML fonctionne sans licence.');
S = Ts * tril(ones(N));
P = zeros(N);
for i = 1:N
    for j = 1:i
        P(i,j) = Ts^2 * (i-j+0.5);
    end
end
% V = v*ones(N,1) + S*a ; X = x + (1:N)'*Ts*v + P*a.
H = 2*(S'*S + rho*eye(N));
C = [P; -S];
lb = -ones(N,1); ub = ones(N,1);
options = optimoptions('quadprog','Display','off');
history = [0 x v NaN];
fprintf('  k        x (m)       v (m/s)      a appliquee (m/s^2)\n');
for k = 0:9
    g = 2*S'*((v-vref)*ones(N,1));
    h = [xlim-x-(1:N)'*Ts*v; v*ones(N,1)];
    [a,~,exitflag] = quadprog(H,g,C,h,[],[],lb,ub,[],options);
    if exitflag <= 0 || isempty(a) || ...
            any(C*a > h+1e-7) || any(a < lb-1e-7) || any(a > ub+1e-7)
        fprintf('k=%d : aucune commande admissible validee; simulation interrompue.\n', k);
        fprintf('Etat conserve pour inspection : x=%.6f, v=%.6f.\n',x,v);
        fprintf('Cet arret du calcul ne constitue pas un freinage physique.\n');
        break;
    end
    fprintf('%3d     %9.5f     %9.5f        %9.5f\n',k,x,v,a(1));
    % Appliquer seulement la premiere acceleration, constante sur ce pas.
    xnext = x + Ts*v + 0.5*Ts^2*a(1);
    vnext = v + Ts*a(1);
    x = xnext; v = vnext;
    history(end+1,:) = [(k+1)*Ts x v a(1)]; %#ok<SAGROW>
    if abs(v) < 1e-7 && abs(x-xlim) < 1e-7
        fprintf('Arret a la limite : x=%.6f, v=%.6f.\n',x,v);
        break;
    end
end
figure('Color','w');
subplot(2,1,1); plot(history(:,1),history(:,2),'o-','LineWidth',1.5);
yline(xlim,'--','Limite'); ylabel('x (m)'); grid on;
subplot(2,1,2); plot(history(:,1),history(:,3),'o-','LineWidth',1.5);
xlabel('Temps (s)'); ylabel('v (m/s)'); grid on;
% N=1 : a0=0, (x1,v1)=(2,2), puis QP infaisable.
% N=2 : a0=-1, puis a1=-1 apres recalcul; arrivee en (2,0).
% Aucune contrainte terminale; pas de garantie generale de faisabilite recursive.

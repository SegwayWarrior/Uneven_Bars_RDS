%% plot_energy.m
% this function is plot the kinetic energy, potential energy and total
% energy with time
% written by Jiabin Liu
function plot_energy(x,t,params)

kin_energy = []; % empty array for kinetic energy
pot_energy = []; % empty array for potential energy
for i = 1:size(x,2)
    kin_new = KE(x(:,i),params);
    kin_energy = [kin_energy, kin_new];
    pot_new = PE(x(:,i),params);
    pot_energy = [pot_energy, pot_new];
end

tot_energy = kin_energy + pot_energy;

figure;
% kinetic energy vs. time
subplot(2,2,1), plot(t,kin_energy(1,:));
xlabel('$time$');
ylabel('$Kinetic Energy$');

% potential energy vs. time
subplot(2,2,2), plot(t,pot_energy(1,:));
xlabel('$time$');
ylabel('$Potetial Energy$');

% total energy vs. time
subplot(2,2,3), plot(t,tot_energy(1,:));
xlabel('$time$');
ylabel('$Total Energy$');

end
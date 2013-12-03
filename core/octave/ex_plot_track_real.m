% Plot track line of synthethic rotation
% 		Jose David Tascón V.
%		Dec 2 2013
close all; clear; clc;

load('plot/real/gin_rot_lin.txt')
load('plot/real/gin_tr_lin.txt')
load('plot/real/gin_tr_opt.txt')
load('plot/real/gin_rot_opt.txt')

h1 = plot_track(gin_rot_opt, gin_tr_opt, '--');
h2 = plot_track(gin_rot_lin, gin_tr_lin);

legend([h1 h2],'opt', 'lin');
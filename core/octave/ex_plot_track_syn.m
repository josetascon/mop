% Plot track line of synthethic rotation
% 		Jose David Tascón V.
%		Dec 2 2013
close all; clear; clc;

load('plot/synthetic/syn_rot36.txt')
load('plot/synthetic/syn_tr36.txt')
load('plot/synthetic/syn_rot_lin36.txt')
load('plot/synthetic/syn_tr_lin36.txt')
load('plot/synthetic/syn_tr_opt36.txt')
load('plot/synthetic/syn_rot_opt36.txt')

h1 = plot_track(syn_rot36, syn_tr36, '.', 1); % remove offset
h2 = plot_track(syn_rot_opt36, syn_tr_opt36, '--');
h3 = plot_track(syn_rot_lin36, syn_tr_lin36);

%  load('plot/synthetic/syn_rot144.txt')
%  load('plot/synthetic/syn_tr144.txt')
%  load('plot/synthetic/syn_rot_lin144.txt')
%  load('plot/synthetic/syn_tr_lin144.txt')
%  load('plot/synthetic/syn_tr_opt144.txt')
%  load('plot/synthetic/syn_rot_opt144.txt')
%  
%  h1 = plot_track(syn_rot144, syn_tr144, '.', 1); % remove offset
%  h2 = plot_track(syn_rot_opt144, syn_tr_opt144, '--');
%  h3 = plot_track(syn_rot_lin144, syn_tr_lin144);



legend([h1 h2 h3],'ori', 'opt', 'lin');
%  legend([h1 h3],'ori', 'lin');
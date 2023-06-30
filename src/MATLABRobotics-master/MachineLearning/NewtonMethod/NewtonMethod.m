% -------------------------------------------------------------------------
%
% File : NewtonMethod.m
%
% Discription : Newton Method Method
% 
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

function [] = NewtonMethod()
close all;
clear all;

disp('Optimization simulation with Newton method starts!!')
%�V�~�����[�V������Ԃ̍ő�ŏ��͈�
maxxy=5;
minxy=-5;

%���b�V���f�[�^�̐���
[x1range,x2range,yrange]=CreateMeshData(minxy,maxxy);

%�����p�����[�^�̐���[x1 x2 y]
param=InitSearchParameter(minxy,maxxy);

finalParam=NewtonMethodOptimization(param,x1range,x2range,yrange);

function finalParam=NewtonMethodOptimization(param,x1range,x2range,yrange)
%�ŋ}�~���@�ɂ��œK��

iterationMax=50;%�C�^���[�V�����̍ő��
alpha=1;%�w�K��
result=[];%�p�����[�^�̗���
for i=1:iterationMax
    
    %���R�r�s��̎擾
    J=CalcJacobi(param(1),param(2));
    %�w�b�Z�s��̎擾
    H=CalcHessian(param(1),param(2));
    
    %�j���[�g���@�ɂ��p�����[�^�̍X�V
    param(1:2)=param(1:2)-(alpha*inv(H)*J')';
    param(3)=f(param(1),param(2));
    
    if sum(abs(alpha*J))<=0.01
        disp('Converge');
        i
        break;
    end
    
    %�V�~�����[�V�������ʂ̕`��
    figure(1)
    hold off;
    contour(x1range,x2range,yrange,100); hold on;
    result=[result;param];
    plot3(result(:,1),result(:,2),result(:,3),'.-k','markersize',10);
    xlabel('x1');
    ylabel('x2');
    view(2)
    drawnow;
    pause(0.5);
     
end

finalParam=param(end,:);

function param=InitSearchParameter(minxy,maxxy)
%�����p�����[�^���쐬����֐�

x1=maxxy - (maxxy-minxy).*rand(1,1);
x2=maxxy - (maxxy-minxy).*rand(1,1);
y=f(x1,x2);
param=[x1' x2' y'];

function [x1range,x2range,yrange]=CreateMeshData(minxy,maxxy)
%�V�~�����[�V������Ԃ̃��b�V���f�[�^���쐬����֐�

%���b�V���f�[�^�̍쐬
[x1range,x2range]=meshgrid(minxy:0.3:maxxy);
yrange=f(x1range,x2range);

function y = f(x1,x2)
% Himmelblau's function
% see Himmelblau's function - Wikipedia, the free encyclopedia 
% http://en.wikipedia.org/wiki/Himmelblau%27s_function
y=(x1.^2+x2-11).^2+(x1+x2.^2-7).^2;

function J= CalcJacobi(x,y)
% jacobi matrix of Himmelblau's function
dx=4*x^3+4*x*y-44*x+2*x+2*y^2-14;
dy=2*x^2+4*x*y+4*y^3-26*y-22;
J=[dx dy];

function H= CalcHessian(x,y)
% Hessian matrix of Himmelblau's function
dxx=12*x^2+4*y-42;
dxy=4*x+4*y;
dyy=4*x+12*y^2-26;
H=[dxx dxy;
   dxy dyy];
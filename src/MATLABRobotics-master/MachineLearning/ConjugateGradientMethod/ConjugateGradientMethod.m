% -------------------------------------------------------------------------
%
% File : ConjugateGradientMethod.m
%
% Discription : Non-Linear optimizion with Conjugate Gradient Method
% 
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : Modified BSD Software License Agreement
% -------------------------------------------------------------------------

function [] = ConjugateGradientMethod()
close all;
clear all;

disp('Optimization simulation with conjugate gradient method starts!!')
%�V�~�����[�V������Ԃ̍ő�ŏ��͈�
maxxy=5;
minxy=-5;

%���b�V���f�[�^�̐���
[x1range,x2range,yrange]=CreateMeshData(minxy,maxxy);

%�����p�����[�^�̐���[x1 x2 y]
param=InitSearchParameter(minxy,maxxy);

finalParam=ConjugateGradientMethodOptimization(param,x1range,x2range,yrange);
disp('Final Param is ');
finalParam

function finalParam=ConjugateGradientMethodOptimization(param,x1range,x2range,yrange)
%�ŋ}�~���@�ɂ��œK��

iterationMax=50;%�C�^���[�V�����̍ő��
result=param;%�p�����[�^�̗���

%�������z�@�p�p�����[�^������
d=-CalcJacobi(param(1),param(2));%���z�x�N�g���̏�����
preJ=CalcJacobi(param(1),param(2));

for i=1:iterationMax
    
    %���R�r�s��̎擾
    J=CalcJacobi(param(1),param(2));
    
    %���z�̃x�N�g���̍X�V
    beta=max(0,((J-preJ)*J')/(preJ*preJ'));
    d=-J+beta*d;
    
    %�w�K���̐��`�T��
    alpha=GoldenSection(0,0.5,param,d);
    
    % �������z�̌v�Z
    param(1:2)=param(1:2)+alpha*d;
    param(3)=f(param(1),param(2));%�]���֐��̌v�Z
    preJ=J;%��O�̃��R�r�s����X�g�A
    
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
    
    %��������
    if sum(abs(alpha*J))<=0.01
        disp(['Converge:',i]);
        break;
    end
end

finalParam=param(end,:);

function [minX]=GoldenSection(a,b,param,d)
%���������@�ɂ����`�T���֐�

%������
GOLDEN_RATIO = 1.6180339887498948482045868343656;
 
%�����_�̌v�Z
x1 = (a-b)/(GOLDEN_RATIO + 1.0) + b;
x2 = (a-b)/GOLDEN_RATIO + b;
%�]���֐��𗼕��v�Z����͍̂ŏ�����
paramTmp=param(1:2)+x1*d;
f1=f(paramTmp(1),paramTmp(2));
paramTmp=param(1:2)+x2*d;
f2=f(paramTmp(1),paramTmp(2));
while 1
    %���[�v���񂵂ė��_���X�V
    if f1 < f2
        a = x2;
        x2 = x1;
        f2 = f1;
        x1 = (a - b)/(GOLDEN_RATIO + 1.0) + b;
        paramTmp=param(1:2)+x1*d;
        f1=f(paramTmp(1),paramTmp(2));
    else
        b = x1;
        x1 = x2;
        f1 = f2;
        x2 = (a - b)/GOLDEN_RATIO + b;
        paramTmp=param(1:2)+x2*d;
        f2=f(paramTmp(1),paramTmp(2));
    end
    %��������
    if abs(a-b)<=10^-3
        minX=(a+b)/2;
        break
    end
end

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
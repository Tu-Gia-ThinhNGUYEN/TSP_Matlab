% -------------------------------------------------------------------------
%
% File : EMAlgorithmSample.m
%
% Discription : Sample code to estimate probablistic parameters 
%               with EM algorithm.
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : GPL Software License Agreement
% -------------------------------------------------------------------------

function EMAlgorithmSample()
close all;
clear all;

%���K���z�̃p�����[�^1
muTrue1=5;
siTrue1=2;
nData1=300;

%���K���z�p�����[�^����T���v���f�[�^���쐬
r1 = muTrue1 + siTrue1.*randn(nData1,1);

%���K���z�̃p�����[�^1
muTrue2=10;
siTrue2=1;
nData2=300;

%���K���z�p�����[�^����T���v���f�[�^���쐬
r2 = muTrue2 + siTrue2.*randn(nData2,1);

data=[r1;r2];%���f�[�^

%�p�����[�^�����l
mu=[5 15];%���ϒl�̏����l
sigma=[std(data) std(data)];%�W���΍��̏����l �f�[�^����v�Z
z=0.5;%������

%EMAlgorithm�ɂ�鐳�K���z�p�����[�^�̐���
oddsResults=[];
for i=1:100
    burdenRates=EStep(data,mu,sigma,z);
    [mu,sigma,z]=MStep(data, burdenRates);
    odds = CalcLogOdds(data,mu,sigma,z);
    oddsResults=[oddsResults odds];
end

figure(1)
plot(oddsResults)
grid on;

figure(2)

disp('Estimated Mean');
mu
disp('True Mean');
[muTrue1 muTrue2]

disp('Estimated STD');
sigma
disp('True Std');
[siTrue1 siTrue2]

disp('Weight Vector');
z

[f,x]=hist(data,10);hold off;
bar(x,f/trapz(x,f));hold on;

est=[];
for id=0:0.05:max(data)
    est=[est; [id (1-z)*Gauss(id,mu(1),sigma(1))+z*Gauss(id,mu(2),sigma(2))]];
end

plot(est(:,1),est(:,2),'-r');hold on;
grid on;


function [mu,sigma,z]=MStep(data, burdenRates)
% MStep�̌v�Z

%���ς̌v�Z
mu=[0 0];
sumb1=sum(1-burdenRates);
sumby1=(1-burdenRates)*data;

mu(1)=sumby1/sumb1;

sumb2=sum(burdenRates);
sumby2=(burdenRates)*data;

mu(2)=sumby2/sumb2;

%�W���΍��̌v�Z
sigma=[0 0];
sumbyu1=(1-burdenRates)*((data-mu(1)).^2);
sigma(1)=sqrt(sumbyu1/sumb1);

sumbyu2=burdenRates*((data-mu(2)).^2);
sigma(2)=sqrt(sumbyu2/sumb2);

z=sum(burdenRates./length(data(:)));


function burdenRates=EStep(data,mu,sigma,z)
%���S�����v�Z����֐�
burdenRates=[];%���S��

for i=1:length(data(:,1))
    
    %�����m���̌v�Z
    n = z * Gauss(data(i), mu(2), sigma(2));
    d = (1 - z) * Gauss(data(i), mu(1), sigma(1)) + n;
    burdenRates=[burdenRates n/d];
end

function odds = CalcLogOdds(data,mu,sigma,z)
%�ΐ��I�b�Y���v�Z����֐�
odds=0;
for i=1:length(data)
    g1=Gauss(data(i),mu(1),sigma(1));
    g2=Gauss(data(i),mu(2),sigma(2));
    odds=odds+log((1-z)*g1+z*g2);
end

function si=UpdateSigma(e,r,mu)
%�W���΍����X�V����֐�
tmp=0;
for ie=1:length(e)
    tmp=tmp+(e(ie)*(r(ie)-mu)^2);
end
si=sqrt(1/sum(e)*tmp);

function p=Gauss(x,mu,sigma)
%�K�E�X���z�ɂ�����m�����v�Z����֐�
p=(1/sqrt(2*pi*sigma^2)*exp(-(x-mu)^2/(2*sigma^2)));

% -------------------------------------------------------------------------
%
% File : kmeansSample.m
%
% Discription : Sample code for k-means clustering
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2015 Atsushi Sakai
%
% License : GPL Software License Agreement
% -------------------------------------------------------------------------

function kmeansSample()
close all;
clear all;

disp('k-means clustering sample start!!');

data=GetRawData();%���f�[�^���擾����֐�

%k-means�@�ɂ��N���X�^�����O
nCluster=2;%�N���X�^�̐�
[result,means]=kmeansClustering(data,nCluster);
%�N���X�^�����O�̌��ʂ�\��
ShowClusteringResult(result,means,nCluster);

function [result,means]=kmeansClustering(data,nCluster)
%k-means���g���ăN���X�^�����O�����{������@

%�����N���X�^�����O �����_���ɃN���X��U�蕪����
result=[data randi(nCluster,[length(data(:,1)),1])];

while 1
    means=[];
    for i=1:nCluster
        %�e�N���X�^�̕��ϒl���v�Z
        means=[means;mean(result(result(:,3)==i,1:2))];
    end
    
    %�N���X�^�����O�̌��ʂ�\��
    ShowClusteringResult(result,means,nCluster);
    
    %�e�f�[�^�𕽋ϒl����߂����ɍăN���X�^�����O
    nUpdate=0;%�N���X�^���X�V���ꂽ��
    for j=1:length(result(:,1))
        
        %�e�f�[�^�Ɗe���ϒl�܂ł̋������v�Z
        d=[];
        for k=1:nCluster
            d=[d norm(result(j,1:2)-means(k,:))];
        end
        [c,i]=min(d);%��ԋ����̏������N���X�^��ID���v�Z
        if result(j,3)~=i 
            result(j,3)=i;%�N���X�^���X�V
            nUpdate=nUpdate+1;
        end
    end
    
    if nUpdate==0
        break;%�N���X�^���ω����Ȃ��Ȃ�����I��
    end
    
    pause;
end

function ShowClusteringResult(result,means,nCluster)
%�N���X�^�����O�̌��ʂ��O���t�ɕ`�悷��֐�
hold off;

%�O���t�̐F�p�̃f�[�^���쐬
cc=hsv(nCluster);

%�e�N���X�^���Ƀf�[�^��`��
for i=1:nCluster
    data=result(result(:,3)==i,1:2);
    plot(data(:,1),data(:,2),'.','Color',cc(i,:)); hold all;
    plot(means(i,1),means(i,2),'x','Color',cc(i,:));hold all;
end
axis equal;
grid on;
hold on;
	

function data=GetRawData()
%�[���f�[�^�̃N���X�^�[�̒��S�ƌ덷��
nSample=[0 0 2;
        10 10 5];
ndata=30;%��̃N���X�^�Ɋւ���f�[�^�_
data=[];

for nc=1:length(nSample(:,1))
    for i=1:ndata
        xy=nSample(nc,1:2)+randn(1,2)*nSample(nc,3);
        data=[data;xy];
    end
end
data=sortrows(data,2);%�f�[�^���\�[�g���č�����
    



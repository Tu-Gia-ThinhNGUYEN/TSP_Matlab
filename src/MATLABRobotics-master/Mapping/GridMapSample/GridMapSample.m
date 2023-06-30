% -------------------------------------------------------------------------
%
% File : GridMapSample.m
%
% Discription : Sample code to build grid map
%
% Environment : Matlab
%
% Author : Atsushi Sakai
%
% Copyright (c): 2014 Atsushi Sakai
%
% License : GPL Software License Agreement
% -------------------------------------------------------------------------

function GridMapSample()
close all;
clear all;

%Grid Map�̃p�����[�^
gm.CENTER=[0 0];%�O���b�h�}�b�v�̒��S���W[x y]
gm.RESO=1;%�𑜓x
gm.WIDTH=100;
gm.HEIGHT=100;
gm.nGrid=gm.WIDTH*gm.HEIGHT;
%data�ɂ͕��̂̑��݊m��[0,1]���i�[����B
%�����l�͕s���Ȃ̂�0.5�Ƃ���
gm.data=zeros(1,gm.nGrid)+0.5;

pose=[0 0 0];%���{�b�g�̌��ݎp��[x,y,yaw]

z=GetObservation();%�Z���T�̊ϑ��_�̎擾

%�V�~�����[�V����1:End Point Update
gm1=HitGridUpdate(gm,z);
figure(1);
ShowGridMap(gm1,z);%�O���t�\��

%�V�~�����[�V����2:Likelihood Update
gm2=LikelihoodUpdate(gm,z);
figure(2);
ShowGridMap(gm2,z);%�O���t�\��

%�V�~�����[�V����3:Ray Casting Update
gm3=RayCastingUpdate(gm,z);
figure(3);
ShowGridMap(gm3,z);%�O���t�\��

function gm=RayCastingUpdate(gm,z)
%���C�L���X�e�B���O�ɂ��Grid�̍X�V

%���O���C�L���X�e�B���O���f���̍쐬
gm=PreCasting(gm,z.ANGLE_TICK);

rayId=0;
%���O���C�L���X�e�B���O���f���ɏ]���ăO���b�h�̊m���̍X�V
for iz=1:length(z.data(:,1))%���ꂼ��̊ϑ��_�ɑ΂���
    range=z.data(iz,1);
    
    rayId=rayId+1;%���C�L���X�e�B���O�N���X�^�ɂ�����f�[�^ID
    %�e�ϑ��_�͂��ꂼ��̃N���X�^����擾�ł���Ƃ���B
    
    %�N���X�^���̊egrid�̃f�[�^����r�[�����f���ɂ��update
    for ir=1:length(gm.precasting(rayId).grid(:,1))
        grange=gm.precasting(rayId).grid(ir,1);
        gid=gm.precasting(rayId).grid(ir,5);
        
        if grange<(range-gm.RESO/2) %free
            gm.data(gid)=0;
        elseif grange<(range+gm.RESO/2) %hit
            gm.data(gid)=1;
        end %����ȏ�̋�����grid��unknown�Ȃ̂ŉ������Ȃ�
    end
end

function gm=PreCasting(gm,angleTick)
%���O���C�L���X�e�B���O���f���̍쐬

%�e�p�x�ɂ��đΉ�����O���b�h��ǉ����Ă���
precasting=[];%�v���L���X�e�B���O�̌��� [�ŏ��p�x,�ő�p�x,���ɓ���grid�̃f�[�^]
for ia=(0-angleTick/2):angleTick:(360+angleTick/2)
    %�p�x�͈͂̕ۑ�
    ray.minAngle=ia;
    ray.maxAngle=ia+angleTick;
    grid=[];%�p�x�͈͂ɓ������O���b�h�̃f�[�^
    for ig=1:(gm.nGrid)
        %�e�O���b�h��xy�l���擾
        gxy=GetXYFromDataIndex(ig,gm);
        range=norm(gxy);
        angle=atan2(gxy(2),gxy(1));
        if angle<0 %[0 360]�x�ɕϊ�
            angle=angle+2*pi;
        end
        if ray.minAngle<=toDegree(angle) && ray.maxAngle>=toDegree(angle)
            grid=[grid;[range,angle,gxy,ig]];
        end
    end
    %range�̒l�Ń\�[�e�B���O���Ă���
    if ~isempty(grid)
        ray.grid=sortrows(grid,1);
    end
    precasting=[precasting;ray];
end
gm.precasting=precasting;%Grid Map�f�[�^�ɒǉ�


function gm=LikelihoodUpdate(gm,z)
%�ޓx���GridMap�����֐�

for ig=1:(gm.nGrid-1)
    gxy=GetXYFromDataIndex(ig,gm);%���ꂼ��̃O���b�hxy�C���f�b�N�X���擾
    zxy=FindNearest(gxy,z);%�ŋߖT�̊ϑ��l�̎擾
    p=GaussLikelihood(gxy,zxy);%�K�E�V�A���ޓx�̌v�Z
    gm.data(ig)=p*10;%�O���b�h�ւ̊i�[
end

function p=GaussLikelihood(gxy,zxy)
%�K�E�X���z�̖ޓx���v�Z����֐�
Sigma=diag([3,3]);%�����U�s��
p=det(2*pi*Sigma)^(-0.5)*exp(-0.5*(gxy-zxy)*inv(Sigma)*(gxy-zxy)');

function zxy=FindNearest(xy,z)
%������W�lxy�Ɉ�ԋ߂�z�̒l��Ԃ��֐�

%���ׂĂ�z��xy�̍����v�Z
d=z.data(:,3:4)-repmat(xy,length(z.data(:,1)),1);

%�m���������̍ŏ��l�̃C���f�b�N�X���擾
min=inf;%�ŏ��l
minid=0;
for id=1:length(d(:,1))
    nd=norm(d(id,:));
    if min>nd
        min=nd;
        minid=id;
    end
end
zxy=z.data(minid,3:4);

function xy=GetXYFromDataIndex(ig,gm)
%Grid�̃f�[�^�C���f�b�N�X����,���̃O���b�h��x,y���W���擾����֐�

%x,y�C���f�b�N�X�̎擾
indy=rem(ig,gm.WIDTH)-1;
indx=fix(ig/gm.WIDTH);

x=GetXYPosition(indx,gm.WIDTH,gm.RESO);
y=GetXYPosition(indy,gm.HEIGHT,gm.RESO);
xy=[x y];

function position=GetXYPosition(index, width, resolution)
%x-y�C���f�b�N�X�̒l����A�ʒu���擾����֐�
position=resolution*(index-width/2)+resolution/2;

function gm=HitGridUpdate(gm,z)
%�ϑ��_���q�b�g�����O���b�h�̊m����1�ɂ���֐�

for iz=1:length(z.data(:,1))
    zx=z.data(iz,3);
    zy=z.data(iz,4);
    ind=GetDBIndexFromXY(zx,zy,gm);
    gm.data(ind)=1.0;
end
gm.data=Normalize(gm.data);%���K��

function ind=GetDBIndexFromXY(x,y,gm)
%xy�̒l����Ή�����O���b�h�̃x�N�g���C���f�b�N�X���擾����֐�
%�O���b�h�}�b�v���S���W�n�ɕϊ�
x=x-gm.CENTER(1);
y=y-gm.CENTER(2);

%�O���b�h�}�b�v�̃C���f�b�N�X���擾
indX=GetXYIndex(x,gm.WIDTH, gm.RESO);%x��������index
indY=GetXYIndex(y,gm.HEIGHT,gm.RESO)+1;%y��������index
ind=GetDBIndexFromIndex(indX,indY,gm);%DB�̃C���f�b�N�X

function ind=GetDBIndexFromIndex(indX,indY,gm)
%�f�[�^�x�N�g���̃C���f�b�N�X���v�Z����֐�
%-1�͖����C���f�b�N�X��\���B

%�S�̃C���f�b�N�X
ind=gm.WIDTH*indX+indY;
%X�����̃C���f�b�N�X�`�F�b�N
if(indX>=gm.WIDTH)
    ind=-1;
%Y�����̃C���f�b�N�X�`�F�b�N
elseif(indY>=gm.HEIGHT)
    ind=-1;        
%�S�̃C���f�b�N�X�̃`�F�b�N
elseif(ind>=gm.nGrid)
    ind=-1;
end

function ind=GetXYIndex(position, width, resolution)
%x-y�����̒l����A�C���f�b�N�X���擾����֐�
ind=fix((position/resolution+width/2.0));

function z=GetObservation()
%�ϑ��_���Z���T�̃��f���Ɋ�āA�����_���Ɏ擾����֐�
z.data=[];% �ϑ��l[range, angle x y;...]
z.ANGLE_TICK=10;%�X�L�������[�U�̊p�x�𑜓x[deg]
z.MAX_RANGE=50;%�X�L�������[�U�̍ő�ϑ�����[m]
z.MIN_RANGE=5;%�X�L�������[�U�̍ŏ������ϑ�����[m]

for angle=0:z.ANGLE_TICK:360
    range=rand()*(z.MAX_RANGE-z.MIN_RANGE)+z.MIN_RANGE;
    rad=toRadian(angle);
    x=range*cos(rad);
    y=range*sin(rad);
    z.data=[z.data;[range rad x y]]; 
end

function ShowGridMap(gm,z)
%�O���b�h�}�b�v��\������֐�
xmin=gm.CENTER(1)-gm.WIDTH*gm.RESO/2;
xmax=gm.CENTER(1)+gm.WIDTH*gm.RESO/2-gm.RESO;
ymin=gm.CENTER(2)-gm.HEIGHT*gm.RESO/2;
ymax=gm.CENTER(2)+gm.HEIGHT*gm.RESO/2-gm.RESO;
%XY�̃C���f�b�N�X�p�ϐ��̍쐬
[X,Y] = meshgrid(xmin:gm.RESO:xmax, ymin:gm.RESO:ymax);
sizeX=size(X);
data=reshape(gm.data,sizeX(1),sizeX(2));%�x�N�g���f�[�^���s���
surf(X,Y,data);hold on;
view(2)
%�ϑ��_
plot3(z.data(:,3),z.data(:,4),zeros(1,length(z.data(:,1)))+1.0,'xy');
plot3(0,0,1.0,'^c');%���{�b�g�̈ʒu

function Z=Normalize(Z)
%�O���b�h�}�b�v�̖ޓx�𐳋K������֐�
sumZ=sum(sum(Z,1,'double'),'double');
Z=Z./sumZ;

function radian = toRadian(degree)
% degree to radian
radian = degree/180*pi;

function degree = toDegree(radian)
% radian to degree
degree = radian/pi*180;






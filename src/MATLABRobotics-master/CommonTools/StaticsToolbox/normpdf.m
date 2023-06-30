function pdf=normpdf(x,mu,sigma)
% ���K���z�̊m�����x�֐�PDF���v�Z����֐�
% StaticsToolBox��normpdf�֐��Ɠ����@�\�ɂ�������
% �Q��:
% ���K���z - MATLAB & Simulink - MathWorks ���{ 
% http://jp.mathworks.com/help/stats/normal-distribution.html

if nargin==1
    mu=0;
    sigma=1;
elseif nargin==2
    sigma=1;
end

prefix=1/sqrt(2*pi)/sigma;
pdf=prefix.*exp(-(x-mu).^2./(2*sigma^2));


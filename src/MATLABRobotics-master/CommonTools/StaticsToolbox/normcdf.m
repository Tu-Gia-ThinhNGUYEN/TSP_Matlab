function cdf=normcdf(x,mu,sigma)
%���K�ݐϕ��z�֐�CDF�ƌv�Z����֐�
%StatticsToolBox��normcdf�֐��Ɠ����@�\�ɂ�������
%�Q��:
%���K�ݐϕ��z�֐� - MATLAB normcdf - MathWorks ���{ 
%http://jp.mathworks.com/help/stats/normcdf.html
if nargin==1
    mu=0;
    sigma=1;
elseif nargin==2
    sigma=1;
end
cdf=[];
resolution=10000;
for i=1:length(x)
    xt = 0 : (x(i) / resolution) : x(i);
    cdf= [cdf sum(normpdf(xt,mu,sigma)* x(i)/resolution)];
end
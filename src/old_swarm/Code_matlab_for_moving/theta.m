function x=theta(x1,y1,x2,y2)
if((x1==0)&&(y1==0))
    a=y2/x2;
else
b=(y2-y1*(x2/x1))/(1-(x2/x1));
a=(y1-b)/x1;
end
x=a;


function u=anpha_target(x1,y1,x2,y2)

if((x1~=x2)||(y1~=y2))
    if x1==x2
        anpha=90;
    else
        if((x1==0)&&(y1==0))
            a=y2/x2;
        else
            b=(y2-y1*(x2/x1))/(1-(x2/x1));
            a=(y1-b)/x1;
        end
            theta=a;
        
        if theta>=0
            anpha = (atand(theta)/180)*pi;
        else
            anpha = (180+atand(theta))/180*pi;
        end
        
    end
    %% Cho xe quay góc về đúng hướng để tịnh tiến ở đây so sánh vị trí x y xem thằng nào ở xa hay ở gần so với tâm O
if (x1==x2)
        if y1<y2
            anpha_target=anpha;
        else
            anpha_target=-(pi+anpha);
        end
    else
        if y1<y2
            anpha_target=anpha;
        else
            anpha_target=-(pi+anpha);
        end
    end
end
u=anpha_target;
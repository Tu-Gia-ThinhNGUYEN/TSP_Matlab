function [tour] = antTour(city,m,n,h,t,alpha,beta,sales,depot)
if mod((n-1),sales)~=0

end
for i=1:m
    mh=h;
    stationObtained = fix((n-1)/sales);
    for j=1:n-1+sales-1
        if j==(stationObtained+1)
            c=city(i,j);
            mh(:,c)=0;
            city(i,j+1)=city(i,1);
            stationObtained = stationObtained + j;
            continue
        end

        c=city(i,j);
        mh(:,c)=0;
        temp=(t(c,:).^alpha).*(mh(c,:).^beta);
        s=(sum(temp));
        p=(1/s).*temp;
        r=rand;
        s=0;
        for k=1:n
            s=s+p(k);
            if  r<=s
                city(i,j+1)=k;
                break
            end
        end
    end
end
tour=city;


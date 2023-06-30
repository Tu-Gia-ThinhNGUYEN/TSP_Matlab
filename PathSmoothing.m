function optPath=PathSmoothing(path)
optPath=path;%元のパスをコピー

%平準化パラメータ
alpha=0.25;
beta=0.3;

torelance=0.00001;%パスの変化量の閾値(変化量がこの値以下の時平滑化を終了)
change=torelance;%パスの位置の変化量
while change>=torelance 
    change=0;%初期化
    for ip=2:(length(path(:,1))-1) %始点と終点は固定
        prePath=optPath(ip,:);%変化量計測用
        optPath(ip,:)=optPath(ip,:)-alpha*(optPath(ip,:)-path(ip,:));
        optPath(ip,:)=optPath(ip,:)-beta*(2*optPath(ip,:)-optPath(ip-1,:)-optPath(ip+1,:));
        change=change+norm(optPath(ip,:)-prePath);
    end
end

end
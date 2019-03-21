function [c,d] = regionfill_all(color,dis,msk)

c = color;
c(:,:,1) = regionfill(color(:,:,1), msk);
c(:,:,2) = regionfill(color(:,:,2), msk);
c(:,:,3) = regionfill(color(:,:,3), msk);

d  = regionfill(dis, msk);

end


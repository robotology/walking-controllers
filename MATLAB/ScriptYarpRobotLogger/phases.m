ds = (robot_logger_device.walking.left_foot.position.desired.data(3,:) == 0) & (robot_logger_device.walking.right_foot.position.desired.data(3,:) == 0);

initSS_left = [];
initSS_right = [];
initDS = [];

isDS = 0;
isSS = 0;

for i = 1:length(ds)
    if(ds(i) == 1)
        if(isDS == 0)
            initDS = [initDS, i];
            isDS = 1;
        end
        isSS = 0;
    end
    if(ds(i) == 0)
        isDS  = 0 ;
        if(isSS == 0)
            if(rf_des_z(i) ~= 0)
                initSS_left = [initSS_left, i];
            else
                initSS_right = [initSS_right, i];
            end
            isSS = 1;
        end
    end
    
end

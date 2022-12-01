function headingerr = getheadingerr(heading1,heading2)
%UNTITLED The input paras should in RAD
%   The input paras should in RAD
headingerr = RAD2DEG(heading1) - RAD2DEG(heading2);
        
headingerr(headingerr>180) = headingerr(headingerr>180)-360;
headingerr(headingerr<-180) = headingerr(headingerr<-180)+360;
end


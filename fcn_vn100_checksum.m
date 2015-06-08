function crc = fcn_vn100_checksum(msg)

% Test checksum 
% d0 = ['FA','01','28','00','E5','68','2C','43','5C', '21', 'B3','3F','22','A3','CE','3F','90','B5','D8', 'B9', 'B0','29',...
%     '28','3A', 'A0', '37' ,'66','BA','B3','D1'];
% 
% d1 = [];
% for i=1:2:length(d0)
%     d1 = [d1 hex2dec(d0(i:i+1))];
% end
% 
% d1 = uint8(d1);
% 
% msg = d1;

pckg = msg(2:end);

crc = uint16(0);

for i=1:length(pckg)
    crc = bitor(bitshift(crc,-8),bitshift(crc,8));
    
    crc = bitxor(crc,uint16(pckg(i)));
    tmp1 = bitand(crc,uint16(255));
    tmp2 = bitshift(tmp1,-4);
    crc = bitxor(crc,tmp2);
    tmp3 = bitshift(crc,8);
    tmp4 = bitshift(tmp3,4);
    crc = bitxor(crc,tmp4);
    tmp5 = bitand(crc,uint16(255));
    tmp6 = bitshift(tmp5,4);
    tmp7 = bitshift(tmp6,1);
    crc = bitxor(crc,tmp7);
end

end
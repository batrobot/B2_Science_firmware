% By Alireza Ramezani, Champaign-IL
function imuBinaryMsg = fcn_imu_binaryMsg
imuBinaryMsg.serialPort = 1;
imuBinaryMsg.devisor = 1;

imuBinaryMsg.msgType.timeStartup = false;
imuBinaryMsg.msgType.TimeGPS = false;
imuBinaryMsg.msgType.TimeSyncIn = false;
imuBinaryMsg.msgType.Ypr = true;
imuBinaryMsg.msgType.Qtn = false;
imuBinaryMsg.msgType.AngRate = false;
imuBinaryMsg.msgType.Pos = false;
imuBinaryMsg.msgType.Vel = false;
imuBinaryMsg.msgType.Accel = false;
imuBinaryMsg.msgType.Imu = false;
imuBinaryMsg.msgType.Magpres = false;

% Size(Byte)
imuBinaryMsg.msgType.size.timeStartup = 8;
imuBinaryMsg.msgType.size.TimeGPS = 8;
imuBinaryMsg.msgType.size.TimeSyncIn = 8;
imuBinaryMsg.msgType.size.Ypr = 12;
imuBinaryMsg.msgType.size.Qtn = 12;
imuBinaryMsg.msgType.size.AngRate = 12;
imuBinaryMsg.msgType.size.Pos = 24;
imuBinaryMsg.msgType.size.Vel = 12;
imuBinaryMsg.msgType.size.Accel = 12;
imuBinaryMsg.msgType.size.Imu = 24;
imuBinaryMsg.msgType.size.Magpres = 12;

imuBinaryMsg.msgType.type.timeStartup = 'uint64';
imuBinaryMsg.msgType.type.TimeGPS = 'uint64';
imuBinaryMsg.msgType.type.TimeSyncIn = 'uint64';
imuBinaryMsg.msgType.type.Ypr = 'single';
imuBinaryMsg.msgType.type.Qtn = 'single';
imuBinaryMsg.msgType.type.AngRate = 'single';
imuBinaryMsg.msgType.type.Pos = 'double';
imuBinaryMsg.msgType.type.Vel = 'single';
imuBinaryMsg.msgType.type.Accel = 'single';
imuBinaryMsg.msgType.type.Imu = 'single';
imuBinaryMsg.msgType.type.Magpres = 'single';


rcvMsgSize = 6; % Include header and tail.
sentMsgSize = 0;
if(imuBinaryMsg.msgType.timeStartup)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.timeStartup;
    sentMsgSize = sentMsgSize +1;
end

if(imuBinaryMsg.msgType.TimeGPS)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.TimeGPS;
    sentMsgSize = sentMsgSize +1;
end

if(imuBinaryMsg.msgType.TimeSyncIn)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.TimeSyncIn;
    sentMsgSize = sentMsgSize +1;
end

if(imuBinaryMsg.msgType.Ypr)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.Ypr;
    sentMsgSize = sentMsgSize +3;
end

if(imuBinaryMsg.msgType.Qtn)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.Qtn;
    sentMsgSize = sentMsgSize +3;
end

if(imuBinaryMsg.msgType.AngRate)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.AngRate;
    sentMsgSize = sentMsgSize +3;
end

if(imuBinaryMsg.msgType.Pos)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.Pos;
    sentMsgSize = sentMsgSize +3;
end

if(imuBinaryMsg.msgType.Vel)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.Vel;
    sentMsgSize = sentMsgSize +3;
end

if(imuBinaryMsg.msgType.Accel)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.Accel;
    sentMsgSize = sentMsgSize +3;
end

if(imuBinaryMsg.msgType.Imu)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.Imu;
    sentMsgSize = sentMsgSize +6;
end

if(imuBinaryMsg.msgType.Magpres)
    rcvMsgSize = rcvMsgSize + imuBinaryMsg.msgType.size.Magpres;
    sentMsgSize = sentMsgSize +3;
end

imuBinaryMsg.msgType.size.rcvMsgSize = rcvMsgSize;
imuBinaryMsg.msgType.size.sentMsgSize = sentMsgSize;


header = uint32(zeros(1,4));
hd1 = uint8(250);
hd2 = uint8(1);

tmp = uint16(0);


tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.timeStartup),0));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.TimeGPS),1));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.TimeSyncIn),2));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.Ypr),3));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.Qtn),4));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.AngRate),5));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.Pos),6));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.Vel),7));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.Accel),8));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.Imu),9));
tmp = tmp + uint16(bitshift(uint8(imuBinaryMsg.msgType.Magpres),10));


hd3 = typecast(tmp,'uint8');
% header = dec2hex([hd1,hd2,hd3]);
% header = [hd1,hd2,21,0];
header = [hd1,hd2,hd3];
imuBinaryMsg.msgType.header = header;

end

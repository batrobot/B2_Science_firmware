function [rcvMsgSize,sentMsgSize] = fcn_vn100_msgSizeCompute()


imuBinaryMsg = fcn_imu_binaryMsg;

rcvMsgSize = 0;
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
    
end
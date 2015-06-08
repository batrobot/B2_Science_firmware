function header = fcn_vn100_headerCompute()
    imuBinaryMsg = fcn_imu_binaryMsg;
    
    header = uint32(zeros(1,4));
    hd1 = uint8(250);
    hd2 = uint8(1);
    
    tmp = uint16(0);
   
    
    tmp = tmp + bitshift(imuBinaryMsg.msgType.timeStartup,0);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.TimeGPS,1);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.TimeSyncIn,2);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.Ypr,3);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.Qtn,4);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.AngRate,5);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.Pos,6);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.Vel,7);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.Accel,8);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.Imu,9);
    tmp = tmp + bitshift(imuBinaryMsg.msgType.Magpres,10);

    hd3 = typecast(tmp,'uint8');
    header = dec2hex([hd1,hd2,hd3]);
end
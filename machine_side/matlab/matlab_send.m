function matlab_send()
udps_ros2 = dsp.UDPSender( ...
    'RemoteIPPort', 8542, ...
    'RemoteIPAddress', '127.0.0.1' ...
    );
udps_ros = dsp.UDPSender( ...
    'RemoteIPPort', 8537, ...
    'RemoteIPAddress', '127.0.0.1' ...
    );
cleanupObj = onCleanup(@()cleanMeUp(udps_ros2,udps_ros));
while 1
    dataSent = uint8('_msg_test_123456789012345678901234567890123456789012345');
    udps_ros2(dataSent);
    udps_ros(dataSent);
    pause(0.1);
end
end

function cleanMeUp(udps_ros2,udps_ros)
    dataEnd = uint8('_msg_end__');
    udps_ros2(dataEnd);
    udps_ros(dataEnd);
end


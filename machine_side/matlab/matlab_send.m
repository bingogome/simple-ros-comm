udps = dsp.UDPSender( ...
    'RemoteIPPort', 8537, ...
    'RemoteIPAddress', '127.0.0.1' ...
    );

while 1
    dataSent = uint8('_msg_test_123456789012345678901234567890123456789012345');
    udps(dataSent);
    pause(0.1);
end
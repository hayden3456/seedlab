function data = read_arduino_serial(com_port, baud_rate)
    arduino = serialport(com_port, baud_rate);
    
    configureTerminator(arduino,"CR/LF");
    
    i = 1;
    
    while(readline(arduino) ~= "GO")
    end
    
    disp("Started Data Collection");
    
    while(1)
        new_read = readline(arduino);
        if new_read == "STOP"
            break;
        end
        raw_data(i,1) = new_read;
    
        i = i + 1;
    end
    
    for i = 1:length(raw_data)
       data(i,:) = str2double(strsplit(raw_data(i),","));
    end
    
    disp("Stopped Data Collection");
end
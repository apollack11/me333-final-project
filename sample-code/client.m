function client(port)
%   provides a menu for accessing PIC32 motor control functions
%
%   client(port)
%
%   Input Arguments:
%       port - the name of the com port.  This should be the same as what
%               you use in screen or putty in quotes ' '
%
%   Example:
%       client('/dev/ttyUSB0') (Linux/Mac)
%       client('COM3') (PC)
%
%   For convenience, you may want to change this so that the port is hardcoded.
   
% Opening COM connection
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

fprintf('Opening port %s....\n',port);

% settings for opening the serial port. baud rate 230400, hardware flow control
% wait up to 120 seconds for data before timing out
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',120); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('    a: Read Current Sensor (ADC Counts)    b: Read Current Sensor (mA)\n    c: Read Encoder (Ticks)                d: Read Encoder (Degrees)\n    e: Reset Encoder                       q: Quit\n');
    % read the user's choice
    selection = input('\nENTER COMMAND: ', 's');
     
    % send the command to the PIC32
    fprintf(mySerial,'%c\n',selection);
    
    % take the appropriate action
    switch selection
        case 'a'
            ADC_count = fscanf(mySerial,'%d');
            fprintf('Read current sensor %d (ADC counts)\n',ADC_count);
        case 'b'
            ADC_current = fscanf(mySerial,'%f');
            fprintf('Read current sensor %0.2f (mA)\n',ADC_current);
        case 'c'
            counts = fscanf(mySerial,'%d');
            fprintf('The motor angle is %d counts.\n',counts);
        case 'd'
            degrees = fscanf(mySerial,'%f');
            fprintf('The motor angle is %0.2f degrees.\n',degrees);
        case 'e'
            degrees = fscanf(mySerial,'%f');
            fprintf('The encoder has been reset to %0.2f degrees.\n', degrees);
        case 'q'
            has_quit = true;             % exit client
        case 'r'
            state = fscanf(mySerial,'%s');
            fprintf('The PIC32 controller mode is currently %s\n',state);
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end

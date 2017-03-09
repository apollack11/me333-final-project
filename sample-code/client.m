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
mySerial = serial(port, 'BaudRate', 230400, 'FlowControl', 'hardware','Timeout',10); 
% opens serial connection
fopen(mySerial);
% closes serial port when function exits
clean = onCleanup(@()fclose(mySerial));                                 

has_quit = false;
% menu loop
while ~has_quit
    fprintf('PIC32 MOTOR DRIVER INTERFACE\n\n');
    % display the menu options; this list will grow
    fprintf('    a: Read Current Sensor (ADC Counts)    b: Read Current Sensor (mA)\n    c: Read Encoder (Ticks)                d: Read Encoder (Degrees)\n    e: Reset Encoder                       f: Set PWM (-100 to 100)\n    g: Set current gains                   h: Get current gains\n    i: Set position gains                  j: Get position gains\n    k: Test current control                l: Go to angle (deg)\n    p: Unpower the Motor                   q: Quit\n    r: Get mode\n');
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
            fprintf('The motor angle is %d counts\n',counts);
        case 'd'
            degrees = fscanf(mySerial,'%f');
            fprintf('The motor angle is %0.2f degrees\n',degrees);
        case 'e'
            degrees = fscanf(mySerial,'%f');
            fprintf('The encoder has been reset to %0.2f degrees\n', degrees);
        case 'f'                         % example operation
            speed = input('Set PWM (-100 to 100) '); % get the speed to send
            fprintf(mySerial, '%d\n',speed); % send the speed
        case 'g'
            Kp = input('Set your desired Kp current gain [recommended: 4.76]: ');
            fprintf(mySerial, '%f\n',Kp);
            Ki = input('Set your desired Ki current gain [recommended: 0.32]: ');
            fprintf(mySerial, '%f\n',Ki);
            fprintf('Sending Kp = %0.2f and Ki = %0.2f to the current controller\n', Kp, Ki);
        case 'h'
            Kp = fscanf(mySerial, '%f');
            Ki = fscanf(mySerial, '%f');
            fprintf('The current controller is using Kp = %0.2f and Ki = %0.2f\n', Kp, Ki);
        case 'i'
            Kp = input('Set your desired Kp position gain [recommended: 4.76]: ');
            fprintf(mySerial, '%f\n',Kp);
            Ki = input('Set your desired Ki position gain [recommended: 0.32]: ');
            fprintf(mySerial, '%f\n',Ki);
            Kd = input('Set your desired Kd position gain [recommended: 10.63]: ');
            fprintf(mySerial, '%f\n',Ki);
            fprintf('Sending Kp = %0.2f, Ki = %0.2f, and Kd = %0.2f to the position controller\n', Kp, Ki, Kd);
        case 'j'
            Kp = fscanf(mySerial, '%f');
            Ki = fscanf(mySerial, '%f');
            Kd = fscanf(mySerial, '%f');
            fprintf('The position controller is using Kp = %0.2f, Ki = %0.2f, and Kd = %0.2f\n', Kp, Ki, Kd);
        case 'k'
            read_plot_matrix(mySerial);
        case 'l'
            angle = input('Enter the desired motor angle in degrees: ');
            fprintf(mySerial, '%f\n', angle);
            fprintf('Motor moving to %0.0f degrees\n', angle);
        case 'p'
            fprintf('Unpowering the motor\n');
        case 'q'
            has_quit = true;             % exit client
        case 'r'
            state = fscanf(mySerial,'%s');
            fprintf('The PIC32 controller mode is currently %s\n',state);
        case 'z'
            ref_current = fscanf(mySerial,'%f');
            ref_current
            Eint_check = fscanf(mySerial,'%d');
            Eint_check
            Eint_position = fscanf(mySerial,'%d');
            Eint_position
            Eprev_check = fscanf(mySerial,'%f');
            Eprev_check
            Error_check = fscanf(mySerial,'%f');
            Error_check
            U_check = fscanf(mySerial,'%f');
            U_check
            Desired_angle = fscanf(mySerial,'%f');
            Desired_angle
        otherwise
            fprintf('Invalid Selection %c\n', selection);
    end
end

end

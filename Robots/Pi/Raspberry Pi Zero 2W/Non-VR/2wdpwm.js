//////This is an example using PWM channels to control the motors. It doesn't work as expected though and may need fine tuning.

///////////////////////////////////
const gpioPins = [27, 22, 23, 24];
const pwmChannels = [0, 1]; // Corresponds to the PWM channel (0 for pwm0)
const period = 20000000; // 20 ms period (50 Hz)
const dutyCycle = 0; // 1 ms duty cycle (5%)
//////////////////////////////////

function driveForward(leftPwmChannel, rightPwmChannel, speed) {
    pipins.setPwmDutyCycle(leftPwmChannel, speed);
    pipins.setPwmDutyCycle(rightPwmChannel, speed);
    pipins.writePinValue(27, 1);
    pipins.writePinValue(22, 0);
    pipins.writePinValue(23, 1);
    pipins.writePinValue(24, 0);
}

function turnRight(leftPwmChannel, rightPwmChannel, leftSpeed, rightSpeed) {
    pipins.setPwmDutyCycle(leftPwmChannel, leftSpeed);
    pipins.setPwmDutyCycle(rightPwmChannel, rightSpeed);
    pipins.writePinValue(27, 0);
    pipins.writePinValue(22, 0);
    pipins.writePinValue(23, 1);
    pipins.writePinValue(24, 0);
}

function turnLeft(leftPwmChannel, rightPwmChannel, leftSpeed, rightSpeed) {
    pipins.setPwmDutyCycle(leftPwmChannel, leftSpeed);
    pipins.setPwmDutyCycle(rightPwmChannel, rightSpeed);
    pipins.writePinValue(27, 1);
    pipins.writePinValue(22, 0);
    pipins.writePinValue(23, 0);
    pipins.writePinValue(24, 0);
}

function driveReverse(leftPwmChannel, rightPwmChannel, speed) {
    pipins.setPwmDutyCycle(leftPwmChannel, speed);
    pipins.setPwmDutyCycle(rightPwmChannel, speed);
    pipins.writePinValue(27, 0);
    pipins.writePinValue(22, 1);
    pipins.writePinValue(23, 0);
    pipins.writePinValue(24, 1);
}

function stop(leftPwmChannel, rightPwmChannel) {
    pipins.setPwmDutyCycle(leftPwmChannel, 0);
    pipins.setPwmDutyCycle(rightPwmChannel, 0);
    pipins.writePinValue(27, 0);
    pipins.writePinValue(22, 0);
    pipins.writePinValue(23, 0);
    pipins.writePinValue(24, 0);
}

function handleInputChannel(inputChannel) {
    inputChannel.onopen = () => {
        console.log("Input channel connected to peer");
        inputChannel.send("Message from input channel");
    };

    inputChannel.onmessage = (event) => {
        console.log("Received input data:", event.data);
        const value = event.data;
        let response = "unknown command";
        ///////////////////////////////////////// Handle inputs here
        switch(value) {
            case "forward":
                driveForward(0, 1, 20000000);
                response = "Moving forward";
                break;
            case "right":
                turnRight(0, 1, 0, 20000000);
                response = "Turning right";
                break;
            case "forwardRight":
                driveForward(0, 1, 15000000, 20000000);
                response = "Moving forward right";
                break;
            case "left":
                turnLeft(0, 1, 20000000, 0);
                response = "Turning left";
                break;
            case "forwardLeft":
                driveForward(0, 1, 20000000, 15000000);
                response = "Moving forward left";
                break;
            case "reverse":
                driveReverse(0, 1, 20000000);
                response = "Reversing";
                break;
            case "park":
                stop(0, 1);
                response = "In park";
                break;
            case "off":
                gpioPins.forEach(pin => pipins.writePinValue(pin, 0));
                response = "Pins off";
                break;
            default:
                response = "unknown command";
                break;
        }
        inputChannel.send(response);
    };

    inputChannel.onclose = () => {
        console.log("Input channel has been closed");
    };

    inputChannel.onerror = (error) => {
        console.error("Input channel error:", error);
    };
}

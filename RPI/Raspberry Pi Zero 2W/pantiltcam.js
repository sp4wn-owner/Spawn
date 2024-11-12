//This is an example using servos to control a pan/tilt camera mount. 

///////////////////////////////////
const pwmChannels = [0, 1]; // Corresponds to the PWM channel (0 for pwm0)
let servoPanPin = 0; //These channels are configured in config.txt. RPI Zero 2W has two hardware PWM channels (0,1)
let servoTiltPin = 1; 
const startingServoPosition = 90; //Set starting positions
let tiltPosition = startingServoPosition; 
let panPosition = startingServoPosition;
const servoIncrement = 2;
const MAX_PAN_VALUE = 145; //set these numbers to confine servos to a range
const MIN_PAN_VALUE = 0;
const MAX_TILT_VALUE = 95;
const MIN_TILT_VALUE = 25;
///////////////////////////////////

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
                tiltPosition = Math.max(MIN_TILT_VALUE, tiltPosition - servoIncrement);
                pipins.setServoPosition(servoTiltPin, tiltPosition);
                response = `Tilt servo position: ${tiltPosition}`;
                break;
            case "right":
                panPosition = Math.max(MIN_PAN_VALUE, panPosition - servoIncrement);
                pipins.setServoPosition(servoPanPin, panPosition);
                response = `Pan servo position: ${panPosition}`;
                break;
            case "left":
                panPosition = Math.min(MAX_PAN_VALUE, panPosition + servoIncrement);
                pipins.setServoPosition(servoPanPin, panPosition);
                response = `Pan servo position: ${panPosition}`;
                break;
            case "reverse":
                
                tiltPosition = Math.min(MAX_TILT_VALUE, tiltPosition + servoIncrement);
                pipins.setServoPosition(servoTiltPin, tiltPosition);
                response = `Tilt servo position: ${tiltPosition}`;
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

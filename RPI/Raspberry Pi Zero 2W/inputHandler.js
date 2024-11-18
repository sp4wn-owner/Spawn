const pipins = require('@sp4wn/pipins');

let servoPanPin = 1; 
let servoTiltPin = 0; 
let tiltPosition = 90; //Set starting positions
let panPosition = 90;
const MIN_VALUE = 0;
const MAX_VALUE = 180;

process.on('message', (command) => {
    let response = "unknown command";
    switch(command) {
        case "forward":
                pipins.writePinValue(27, 1);
                pipins.writePinValue(22, 0);
                pipins.writePinValue(23, 1);
                pipins.writePinValue(24, 0);
                response = "Moving forward";
                break;
            case "left":
                pipins.writePinValue(27, 1);
                pipins.writePinValue(22, 0);
                pipins.writePinValue(23, 0);
                pipins.writePinValue(24, 0);
                response = "Turning left";
                break;
            case "right":
                pipins.writePinValue(27, 0);
                pipins.writePinValue(22, 0);
                pipins.writePinValue(23, 1);
                pipins.writePinValue(24, 0);
                response = "Turning right";
                break;
            case "reverse":
                pipins.writePinValue(27, 0);
                pipins.writePinValue(22, 1);
                pipins.writePinValue(23, 0);
                pipins.writePinValue(24, 1);
                response = "Reversing";
                break;
            case "park":
                pipins.writePinValue(27, 0);
                pipins.writePinValue(22, 0);
                pipins.writePinValue(23, 0);
                pipins.writePinValue(24, 0);
                response = "In park";
                break;
            case "rju":
                //demo servo pwm code
                tiltPosition = Math.min(MAX_VALUE, tiltPosition + 5);
                pipins.setServoPosition(servoTiltPin, tiltPosition);
                response = `Tilt servo position: ${tiltPosition}`;
                break;
            case "rjl":
                panPosition = Math.max(MIN_VALUE, panPosition - 5);
                pipins.setServoPosition(servoPanPin, panPosition);
                response = `Pan servo position: ${panPosition}`;
                break;
            case "rjr":
                panPosition = Math.min(MAX_VALUE, panPosition + 5);
                pipins.setServoPosition(servoPanPin, panPosition);
                response = `Pan servo position: ${panPosition}`;
                break;
            case "rjd":
                tiltPosition = Math.max(MIN_VALUE, tiltPosition - 5);
                pipins.setServoPosition(servoTiltPin, tiltPosition);
                response = `Tilt servo position: ${tiltPosition}`;
                break;
            case "rjc":
                response = "Right stick center";
                break;
            case "0":
                response = "Button pressed: A";
                break;
            case "1":
                response = "Button pressed: B";
                break;
            case "2":
                response = "Button pressed: X";
                break;
            case "3":
                response = "Button pressed: Y";
                break;
            case "4":
                response = "Button pressed: L1";
                break;
            case "5":
                response = "Button pressed: R1";
                break;
            case "6":
                response = "Button pressed: L2";
                break;
            case "7":
                response = "Button pressed: R2";
                break;
            case "8":
                response = "Button pressed: SELECT";
                break;
            case "9":
                response = "Button pressed: START";
                break;
            case "10":
                response = "Button pressed: Left Stick";
                break;
            case "11":
                response = "Button pressed: Right Stick";
                break;
            case "12":
                response = "Button pressed: D-pad UP";
                break;
            case "13":
                response = "Button pressed: D-pad DOWN";
                break;
            case "14":
                response = "Button pressed: D-pad LEFT";
                break;
            case "15":
                response = "Button pressed: D-pad RIGHT";
                break;
            case "off":
                gpioPins.forEach(pin => pipins.writePinValue(pin, 0));
                response = "Pins off";
                break;
            default:
                response = "unknown command";
                break;
    }
    process.send(response);
});

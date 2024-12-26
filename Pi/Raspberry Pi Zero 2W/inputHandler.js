const pipins = require('@sp4wn/pipins');

let servoPanPin = 1; 
let servoTiltPin = 0; 
let tiltPosition = 90;
let panPosition = 90;
const MIN_VALUE = 0;
const MAX_VALUE = 180;
const deadZone = 0.1;

function moveForward() {
    pipins.writePinValue(27, 1);
    pipins.writePinValue(22, 0);
    pipins.writePinValue(23, 1);
    pipins.writePinValue(24, 0);
}

function turnLeft() {
    pipins.writePinValue(27, 1);
    pipins.writePinValue(22, 0);
    pipins.writePinValue(23, 0);
    pipins.writePinValue(24, 0);
}

function turnRight() {
    pipins.writePinValue(27, 0);
    pipins.writePinValue(22, 0);
    pipins.writePinValue(23, 1);
    pipins.writePinValue(24, 0);
}

function reverse() {
    pipins.writePinValue(27, 0);
    pipins.writePinValue(22, 1);
    pipins.writePinValue(23, 0);
    pipins.writePinValue(24, 1);
}

function park() {
    pipins.writePinValue(27, 0);
    pipins.writePinValue(22, 0);
    pipins.writePinValue(23, 0);
    pipins.writePinValue(24, 0);
}

process.on('message', (cmd) => {
    //{"joystickSelector":null,"joystickX":null,"joystickY":null,"buttons":["16"]}

    let response = "unknown command";

    try {
        const { joystickSelector, joystickX, joystickY, buttons } = cmd;

        if (joystickSelector === "joystick1" || joystickSelector === "joystick3") {
            handleJoystickCommands(joystickX, joystickY);
        }

        if (joystickSelector === "joystick2" || joystickSelector === "joystick4") {
            handleServoCommands(joystickX, joystickY);
        }

        if (buttons) {
            handleButtonCommands(buttons);
        }
    } catch (error) {
        console.error('Invalid command format:', cmd, error);
    }

    function handleJoystickCommands(joystickX, joystickY) {
        let command;

        if (joystickY > 0.5) {
            command = "reverse";
        } else if (joystickY < -0.5) {
            command = "forward";
        } else if (joystickX > 0.5) {
            command = "right";
        } else if (joystickX < -0.5) {
            command = "left";
        } else if (Math.abs(joystickX) < deadZone && Math.abs(joystickY) < deadZone) {
            command = "park";
        }

        switch (command) {
            case "forward":
                moveForward();
                response = "Moving forward";
                break;
            case "left":
                turnLeft();
                response = "Turning left";
                break;
            case "right":
                turnRight();
                response = "Turning right";
                break;
            case "reverse":
                reverse();
                response = "Reversing";
                break;
            case "park":
                park();
                response = "In park";
                break;
            default:
                response = "Unknown joystick command";
                break;
        }        
    }

    function handleServoCommands(joystickX, joystickY) {
        let servoCommand;
        if (joystickY > 0.5) {
            servoCommand = "down";
        } else if (joystickY < -0.5) {
            servoCommand = "up";
        } else if (joystickX > 0.5) {
            servoCommand = "right";
        } else if (joystickX < -0.5) {
            servoCommand = "left";
        } else if (Math.abs(joystickX) < deadZone && Math.abs(joystickY) < deadZone) {
            servoCommand = "neutral";
        }

        switch (servoCommand) {
            case "up":
                tiltPosition = Math.min(MAX_VALUE, tiltPosition + 5);
                pipins.setServoPosition(servoTiltPin, tiltPosition);
                response = "Servo: Moving up";
                break;
            case "down":
                tiltPosition = Math.max(MIN_VALUE, tiltPosition - 5);
                pipins.setServoPosition(servoTiltPin, tiltPosition);
                response = "Servo: Moving down";
                break;
            case "right":
                panPosition = Math.min(MAX_VALUE, panPosition + 5);
                pipins.setServoPosition(servoPanPin, panPosition);
                response = "Servo: Moving right";
                break;
            case "left":
                panPosition = Math.max(MIN_VALUE, panPosition - 5);
                pipins.setServoPosition(servoPanPin, panPosition);
                response = "Servo: Moving left";
                break;
            case "neutral":
                response = "Servo: Neutral position";
                break;
            default:
                response = "Servo: unknown command";
                break;
        }
    }

    function handleButtonCommands(buttons) {
        buttons.forEach((button) => {
            switch (button) {
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
                    moveForward();
                    response = "Moving forward";
                    break;
                case "13":
                    reverse();
                    response = "Reversing";
                    break;
                case "14":
                    turnLeft();
                    response = "Turning left";
                    break;
                case "15":
                    turnRight();
                    response = "Turning right";
                    break;
                case "16":
                    park();
                    response = "In park";
                    break;
                case "off":
                    gpioPins.forEach(pin => pipins.writePinValue(pin, 0));
                    response = "Pins off";
                    break;
                default:
                    response = "unknown button command";
                    break;
            }
        });
    }
    process.send(response);
});

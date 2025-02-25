const pipins = require('@sp4wn/pipins');

let servoPanPin = 1;
let servoTiltPin = 0;
let tiltPosition = 90;
let panPosition = 90;
const MIN_VALUE = 0;
const MAX_VALUE = 180;
const SENSITIVITY = 0.5;

process.on('message', (cmd) => {
    let response = "unknown command";

    try {
        if (cmd.quaternion && Array.isArray(cmd.quaternion) && cmd.quaternion.length === 4) {
            handleServoCommands(cmd.quaternion);
            response = "Servo: Updated position from quaternion";
        }
    } catch (error) {
        console.error('Invalid command format:', cmd, error);
        response = "Servo: Error processing command";
    }

    process.send(response);
});

function handleServoCommands(quaternion) {
    const [w, x, y, z] = quaternion;

    // Pitch (y-axis rotation) for tilt
    const sinp = 2 * (w * y - z * x);
    const pitch = Math.abs(sinp) >= 1 ?
        Math.sign(sinp) * Math.PI / 2 :
        Math.asin(sinp);

    // Yaw (z-axis rotation) for pan
    const siny_cosp = 2 * (w * z + x * y);
    const cosy_cosp = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny_cosp, cosy_cosp);

    // Convert to degrees and map to servo range
    const targetTilt = mapAngleToServoRange(pitch * 180 / Math.PI);
    const targetPan = mapAngleToServoRange(yaw * 180 / Math.PI);

    updateServoPositions(targetTilt, targetPan);
}

function mapAngleToServoRange(angle) {
    return Math.round(((angle + 90) / 180) * (MAX_VALUE - MIN_VALUE)) + MIN_VALUE;
}

function updateServoPositions(targetTilt, targetPan) {
    const tiltDiff = targetTilt - tiltPosition;
    const panDiff = targetPan - panPosition;

    if (Math.abs(tiltDiff) > SENSITIVITY) {
        tiltPosition += Math.sign(tiltDiff) * Math.min(Math.abs(tiltDiff), 5);
        tiltPosition = Math.max(MIN_VALUE, Math.min(MAX_VALUE, tiltPosition));
        pipins.setServoPosition(servoTiltPin, Math.round(tiltPosition));
    }

    if (Math.abs(panDiff) > SENSITIVITY) {
        panPosition += Math.sign(panDiff) * Math.min(Math.abs(panDiff), 5);
        panPosition = Math.max(MIN_VALUE, Math.min(MAX_VALUE, panPosition));
        pipins.setServoPosition(servoPanPin, Math.round(panPosition));
    }
}
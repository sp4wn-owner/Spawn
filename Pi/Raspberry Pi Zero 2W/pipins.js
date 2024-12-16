const fs = require('fs');
const { exec } = require('child_process');
const gpioPath = '/sys/class/gpio/';
const pwmPath = '/sys/class/pwm/pwmchip0/';
const BASE_OFFSET = 512; // Define the base offset

// Helper function to run commands with elevated permissions
function runCommand(command, callback) {
  exec(`echo ${command} | sudo tee`, (err, stdout, stderr) => {
    if (err) {
      console.error(`Error running command "${command}":`, stderr);
    }
    if (callback) callback(err, stdout, stderr);
  });
}

// Export the pin to make it available
function exportPin(pin, callback) {
  const pinPath = gpioPath + 'gpio' + (pin + BASE_OFFSET); // Add offset
  if (!fs.existsSync(pinPath)) {
    runCommand(`echo ${pin + BASE_OFFSET} > ${gpioPath}export`, callback); // Add offset
  } else if (callback) {
    callback(null);
  }
}

// Set the direction of the pin ('in' or 'out')
function setPinDirection(pin, direction, callback) {
  runCommand(`echo ${direction} > ${gpioPath}gpio${pin + BASE_OFFSET}/direction`, callback); // Add offset
}

// Write a value to the pin ('1' for high/on, '0' for low/off')
function writePinValue(pin, value, callback) {
  runCommand(`echo ${value} > ${gpioPath}gpio${pin + BASE_OFFSET}/value`, callback); // Add offset
}

// Unexport the pin to clean up
function unexportPin(pin, callback) {
  runCommand(`echo ${pin + BASE_OFFSET} > ${gpioPath}unexport`, callback); // Add offset
}

// PWM functionality
function exportPwm(pwmChannel, callback) {
  const pwmExportPath = pwmPath + 'export';
  if (!fs.existsSync(`${pwmPath}pwm${pwmChannel}`)) { // Check if PWM channel is already exported
    runCommand(`echo ${pwmChannel} > ${pwmExportPath}`, callback); // Export the PWM channel
  } else if (callback) {
    callback(null);
  }
}

function setPwmPeriod(pwmChannel, period, callback) {
  const periodPath = pwmPath + `pwm${pwmChannel}/period`;
  if (fs.existsSync(periodPath)) {
    runCommand(`echo 0 > ${pwmPath}pwm${pwmChannel}/duty_cycle`, (err) => { // Set duty cycle to 0 before changing period
      if (err) {
        console.error(`Error setting duty cycle to 0 for PWM channel ${pwmChannel}:`, err.message);
        if (callback) callback(err);
        return;
      }

      runCommand(`echo ${period} > ${periodPath}`, callback); // Set the period in nanoseconds
    });
  } else {
    console.error(`Error: PWM channel ${pwmChannel} not exported`);
    if (callback) callback(new Error(`PWM channel ${pwmChannel} not exported`));
  }
}

function setPwmDutyCycle(pwmChannel, dutyCycle, callback) {
  const dutyCyclePath = pwmPath + `pwm${pwmChannel}/duty_cycle`;
  if (fs.existsSync(dutyCyclePath)) {
    runCommand(`echo ${dutyCycle} > ${dutyCyclePath}`, callback); // Set the duty cycle in nanoseconds
  } else {
    console.error(`Error: PWM channel ${pwmChannel} not exported`);
    if (callback) callback(new Error(`PWM channel ${pwmChannel} not exported`));
  }
}

function enablePwm(pwmChannel, callback) {
  const enablePath = pwmPath + `pwm${pwmChannel}/enable`;
  if (fs.existsSync(enablePath)) {
    runCommand(`echo 1 > ${enablePath}`, callback); // Enable PWM
  } else {
    console.error(`Error: PWM channel ${pwmChannel} not exported`);
    if (callback) callback(new Error(`PWM channel ${pwmChannel} not exported`));
  }
}

function disablePwm(pwmChannel, callback) {
  const enablePath = pwmPath + `pwm${pwmChannel}/enable`;
  if (fs.existsSync(enablePath)) {
    runCommand(`echo 0 > ${enablePath}`, callback); // Disable PWM
  } else {
    console.error(`Error: PWM channel ${pwmChannel} not exported`);
    if (callback) callback(new Error(`PWM channel ${pwmChannel} not exported`));
  }
}

function unexportPwm(pwmChannel, callback) {
  const unexportPath = pwmPath + 'unexport';
  if (fs.existsSync(unexportPath)) {
    runCommand(`echo ${pwmChannel} > ${unexportPath}`, callback); // Unexport the PWM channel
  } else if (callback) {
    callback(null);
  }
}

// Set servo position
function setServoPosition(pwmChannel, position, callback) {
  const period = 20000000; // 20 ms period (50 Hz)
  const dutyCycle = Math.round((position / 180) * 2000000 + 1000000); // Scale position to duty cycle (1-2 ms)
  
  setPwmPeriod(pwmChannel, period, (err) => {
    if (err) return callback(err);
    setPwmDutyCycle(pwmChannel, dutyCycle, (err) => {
      if (err) return callback(err);
      enablePwm(pwmChannel, callback);
    });
  });
}

// Export functions for use in other modules
module.exports = {
  exportPin,
  setPinDirection,
  writePinValue,
  unexportPin,
  exportPwm,
  setPwmPeriod,
  setPwmDutyCycle,
  enablePwm,
  disablePwm,
  unexportPwm,
  setServoPosition,
};

const fs = require('fs');
const { execSync } = require('child_process');
const gpioPath = '/sys/class/gpio/';
const pwmPath = '/sys/class/pwm/pwmchip0/';
const BASE_OFFSET = 512; // Define the base offset
// Helper function to run commands with elevated permissions
function runCommand(command) {
  try {
    execSync(`sudo ${command}`);
  } catch (err) {
    console.error(`Error running command "${command}":`, err.message);
  }
}
// Export the pin to make it available
function exportPin(pin) {
  const pinPath = gpioPath + 'gpio' + (pin + BASE_OFFSET); // Add offset
  if (!fs.existsSync(pinPath)) {
    try {
      runCommand(`echo ${pin + BASE_OFFSET} > ${gpioPath}export`); // Add offset
    } catch (err) {
      console.error(`Error exporting pin ${pin}:`, err.message);
    }
  }
}
// Set the direction of the pin ('in' or 'out')
function setPinDirection(pin, direction) {
  try {
    runCommand(`echo ${direction} > ${gpioPath}gpio${pin + BASE_OFFSET}/direction`); // Add offset
  } catch (err) {
    console.error(`Error setting direction for pin ${pin}:`, err.message);
  }
}
// Write a value to the pin ('1' for high/on, '0' for low/off)
function writePinValue(pin, value) {
  try {
    runCommand(`echo ${value} > ${gpioPath}gpio${pin + BASE_OFFSET}/value`); // Add offset
  } catch (err) {
    console.error(`Error writing value to pin ${pin}:`, err.message);
  }
}
// Unexport the pin to clean up
function unexportPin(pin) {
  try {
    runCommand(`echo ${pin + BASE_OFFSET} > ${gpioPath}unexport`); // Add offset
  } catch (err) {
    console.error(`Error unexporting pin ${pin}:`, err.message);
  }
}
// PWM functionality
function exportPwm(pwmChannel) {
  const pwmExportPath = pwmPath + 'export';
  if (!fs.existsSync(`${pwmPath}pwm${pwmChannel}`)) { // Check if PWM channel is already exported
    try {
      runCommand(`echo ${pwmChannel} > ${pwmExportPath}`); // Export the PWM channel
    } catch (err) {
      console.error(`Error exporting PWM channel ${pwmChannel}:`, err.message);
    }
  }
}
function setPwmPeriod(pwmChannel, period) {
  const periodPath = pwmPath + `pwm${pwmChannel}/period`;
  if (fs.existsSync(periodPath)) {
    try {
      runCommand(`echo ${period} > ${periodPath}`); // Set the period in nanoseconds
    } catch (err) {
      console.error(`Error setting period for PWM channel ${pwmChannel}:`, err.message);
    }
  } else {
    console.error(`Error: PWM channel ${pwmChannel} not exported`);
  }
}
function setPwmDutyCycle(pwmChannel, dutyCycle) {
  const dutyCyclePath = pwmPath + `pwm${pwmChannel}/duty_cycle`;
  if (fs.existsSync(dutyCyclePath)) {
    try {
      runCommand(`echo ${dutyCycle} > ${dutyCyclePath}`); // Set the duty cycle in nanoseconds
    } catch (err) {
      console.error(`Error setting duty cycle for PWM channel ${pwmChannel}:`, err.message);
    }
  } else {
    console.error(`Error: PWM channel ${pwmChannel} not exported`);
  }
}
function enablePwm(pwmChannel) {
  const enablePath = pwmPath + `pwm${pwmChannel}/enable`;
  if (fs.existsSync(enablePath)) {
    try {
      runCommand(`echo 1 > ${enablePath}`); // Enable PWM
    } catch (err) {
      console.error(`Error enabling PWM channel ${pwmChannel}:`, err.message);
    }
  } else {
    console.error(`Error: PWM channel ${pwmChannel} not exported`);
  }
}
function disablePwm(pwmChannel) {
  const enablePath = pwmPath + `pwm${pwmChannel}/enable`;
  if (fs.existsSync(enablePath)) {
    try {
      runCommand(`echo 0 > ${enablePath}`); // Disable PWM
    } catch (err) {
      console.error(`Error disabling PWM channel ${pwmChannel}:`, err.message);
    }
  } else {
    console.error(`Error: PWM channel ${pwmChannel} not exported`);
  }
}
function unexportPwm(pwmChannel) {
  const unexportPath = pwmPath + 'unexport';
  if (fs.existsSync(unexportPath)) {
    try {
      runCommand(`echo ${pwmChannel} > ${unexportPath}`); // Unexport the PWM channel
    } catch (err) {
      console.error(`Error unexporting PWM channel ${pwmChannel}:`, err.message);
    }
  }
}
// Set servo position
function setServoPosition(pwmChannel, position) {
  const period = 20000000; // 20 ms period (50 Hz)
  const dutyCycle = Math.round((position / 180) * 2000000 + 1000000); // Scale position to duty cycle (1-2 ms)
  
  setPwmPeriod(pwmChannel, period);
  setPwmDutyCycle(pwmChannel, dutyCycle);
  enablePwm(pwmChannel);
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
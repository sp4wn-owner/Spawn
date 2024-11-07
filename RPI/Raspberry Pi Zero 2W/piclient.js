//////This is the main client script. Check out the other examples for how to use servos/pwm when receiving input commands
const WebSocket = require('ws');
const { spawn } = require('child_process');
const { RTCPeerConnection, RTCSessionDescription, RTCIceCandidate } = require('wrtc');
const url = 'https://sp4wn-signaling-server.onrender.com';
const pipins = require('@sp4wn/pipins');


//ENTER USERNAME AND PASSWORD HERE 
////////////////////////////////////
const username = "pi_robot_2wd"; //Username should be all lowercase
const password = "";
const twitchKey = ""; //Copy your key from Twitch stream manager
let isStreamToTwitch = false; //Change to true if you'd like to stream to Twitch. When someone connects to your robot it will stop streaming to Twitch.
///////////////////////////////////
const gpioPins = [27, 22, 23, 24];
const pwmChannels = [0, 1]; //These channels are configured in config.txt. RPI Zero 2W has two hardware PWM channels (0,1)
const period = 20000000; // 20 ms period (50 Hz)
const dutyCycle = 0; // 1 ms duty cycle (5%)
let servoPanPin = 1; 
let servoTiltPin = 0; 
let tiltPosition = 90; //Set starting positions
let panPosition = 90;
const MIN_VALUE = 0;
const MAX_VALUE = 180;
///////////////////////////////////


let isPrivate;
let botpw;
let isStreamToSpawn = false;
let connectionTimeout;
let profilePicture;
let location;
let description;
let tokenrate;
const botdevicetype = "pi";
let peerConnection;
let signalingSocket;
let inputChannel;
let videoChannel;
let intervalIds = [];
let connectedUser;
let configuration;

async function startWebRTC() {
    console.log('Starting WebRTC client...');
    await initializeSignalingAndStartCapture();

    peerConnection = new RTCPeerConnection(configuration);
    try {
        await createDataChannel('video');
        await createDataChannel('input');
        console.log("Video and input channels created");
    } catch (error) {
        console.log("unable to create data channels");
    }
    
    peerConnection.onicecandidate = (event) => {
        if (event.candidate) {
            signalingSocket.send(JSON.stringify({ type: 'candidate', othername: connectedUser, candidate: event.candidate }));
            console.log("sending ice to ", connectedUser);
        }
    };

    peerConnection.oniceconnectionstatechange = () => {
        if (!peerConnection) {
            console.error('Peer connection is not initialized.');
            return; 
        }

        switch (peerConnection.iceConnectionState) {
            case 'new':
                console.log('ICE Connection State is new.');
                break;
            case 'checking':
                console.log('ICE Connection is checking.');
                break;
            case 'connected':
                console.log('ICE Connection has been established.');
                
                
                break;
            case 'completed':
                console.log('ICE Connection is completed.');
                if(!isStreamToTwitch) {
                    startStream();
                }
                break;
            case 'failed':
            case 'disconnected':
                console.log("peer disconnected");   
                cleanup();
            case 'closed':
            break;
        }
    };      
}

async function connectToSignalingServer() {
    return new Promise((resolve, reject) => {
        signalingSocket = new WebSocket(url);

        signalingSocket.onopen = () => {
            clearTimeout(connectionTimeout);
            send({
                type: "robot",
                username: username,
                password: password
            });
        };

        signalingSocket.onmessage = async (event) => {
            const message = JSON.parse(event.data);
            switch (message.type) {

                case "authenticated":
                    handleLogin(message.success, message.pic, message.tokenrate, message.location, message.description, message.isPrivate, message.pw, message.configuration);
                    resolve();
                    break;

                case 'offer':
                    if (peerConnection) {
                        await peerConnection.setRemoteDescription(new RTCSessionDescription(message.offer));
                        const answer = await peerConnection.createAnswer();
                        await peerConnection.setLocalDescription(answer);
                        signalingSocket.send(JSON.stringify({ type: 'answer', answer }));
                    } else {
                        console.log("no answer peer connection");
                    }
                    break;

                case 'answer':
                    if (peerConnection) {
                        try {
                            await peerConnection.setRemoteDescription(new RTCSessionDescription(message.answer));
                        } catch (error) {
                            console.error("Error when setting remote description: ", error);
                        }
                    } else {
                        console.log("no answer peer connection");
                    }
                    break;

                case 'candidate':
                    if (message.candidate) {
                        try {
                            const candidate = new RTCIceCandidate(message.candidate);
                            await peerConnection.addIceCandidate(candidate);
                            console.log('ICE candidate added successfully.');
                        } catch (error) {
                            console.error('Error adding ICE candidate:', error);
                        }
                    } else {
                        console.warn('No ICE candidate in the message.');
                    }
                    break;

                case "watch":
                    watchStream(message.name);
                    break;
            }
        };

        signalingSocket.onclose = () => {
            console.log('Disconnected from signaling server');
            reject(new Error('WebSocket closed unexpectedly')); 
            cleanup();
        };

        signalingSocket.onerror = (error) => {
            console.error('WebSocket error:', error);
            reject(error); 
            cleanup();
        };
    });
}

async function initializeSignalingAndStartCapture() {
    if (!signalingSocket || signalingSocket.readyState !== WebSocket.OPEN) {
        console.log("Connecting to signaling server...");
        connectionTimeout = setTimeout(() => {
            console.log('Connection timed out after 10 seconds');
            cleanup();
          }, 5000);
    
        await connectToSignalingServer(); 
    }

    if (signalingSocket.readyState === WebSocket.OPEN) {
        console.log("Connected to signaling server");
        captureImage();
        startImageCapture(15000);
        addlive(); 
    } else {
        console.error("Failed to connect to signaling server.");
    }
}

function send(message) {
    signalingSocket.send(JSON.stringify(message));
 };
 
 function handleLogin(success, pic, tr, loc, des, priv, pw, config) {
    if (success)  {
        console.log("Successfully logged in");
        configuration = config;
        if(pic) {
            profilePicture = pic;
        }
        if(tr) {
            tokenrate = tr;
        }
        if(loc) {
            location = loc;
        }
        if(des) {
            description = des;
        }
        if(priv) {
            isPrivate = priv;
        }
        if(pw) {
            botpw = pw;
        }
        gpioPins.forEach(pin => {
            pipins.exportPin(pin);
            pipins.setPinDirection(pin, 'out');
            pipins.writePinValue(pin, 0);
            console.log(`GPIO pin ${pin} set as OUTPUT`);
        });
        pwmChannels.forEach(pin => {
            pipins.exportPwm(pin);
            pipins.setPwmPeriod(pin, period);
            pipins.setPwmDutyCycle(pin, dutyCycle);
            pipins.enablePwm(pin);
            console.log(`PWM pin ${pin} enabled`);
        });
        if(isStreamToTwitch) {
            startStream();
        }
    }
    if (!success) {
        console.log("user already logged in or there was an error");
    }
 }

 async function createDataChannel(type) {
    let dataChannel;

    try {
        dataChannel = peerConnection.createDataChannel(type);
        console.log(`Data channel "${type}" created successfully.`);
    } catch (error) {
        console.error(`Failed to create ${type} data channel:`, error);
        return; 
    }

    if (type === 'video') {
        videoChannel = dataChannel;
        handleVideoChannel(videoChannel); 
    } else if (type === 'input') {
        inputChannel = dataChannel;
        handleInputChannel(inputChannel);
    }
}

function handleVideoChannel(videoChannel) {
    videoChannel.onopen = () => {
        console.log("Video channel connected to peer");        
    };

    videoChannel.onclose = () => {
        console.log("Video channel has been closed");
        cleanup();
    };

    videoChannel.onerror = (error) => {
        console.error("Video channel error:", error);
    };
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
        inputChannel.send(response);
    };

    inputChannel.onclose = () => {
        console.log("Input channel has been closed");
    };

    inputChannel.onerror = (error) => {
        console.error("Input channel error:", error);
    };
}

let v4l2Process = null;
let ffmpeg = null;
const delayBeforeOpening = 0; 

function startStream() {

    function startCameraStream() {

        setTimeout(() => {
            v4l2Process = spawn('v4l2-ctl', [
                '--stream-mmap',
                '--stream-to=-',
                '--device=/dev/video0',
                '--set-fmt-video=width=640,height=480,pixelformat=H264',
            ]);        

            if (isStreamToTwitch) {
                console.log("Starting Twitch stream");
                ffmpeg = spawn('ffmpeg', [
                    '-re',
                    '-i', 'pipe:0',
                    '-c:v', 'copy',
                    '-f', 'flv',
                    `rtmp://live.twitch.tv/app/${twitchKey}`
                  ]);
            }            

            v4l2Process.stdout.on('data', (chunk) => {

                if (isStreamToTwitch && ffmpeg && ffmpeg.stdin.writable) {
                    if (isStreamToTwitch && ffmpeg && ffmpeg.stdin.writable) {
                        try {
                          ffmpeg.stdin.write(chunk);
                        } catch (error) {
                          console.error('FFmpeg write error:', error);
                          ffmpeg.stdin.end();
                        }
                    }
                }
          
                if (isStreamToSpawn) {
                  if (videoChannel && videoChannel.readyState === "open") {
                    try {
                        videoChannel.send(chunk);
                      } catch (error) {
                        console.error('Error sending to Data Channel:', error);
                      }
                  }
                }
            });

            v4l2Process.on('exit', (code) => {
                //console.log(`v4l2-ctl process exited with code ${code}`);
                //cleanup();
            });

            v4l2Process.stderr.on('data', (error) => {
                //console.error(`Error from v4l2-ctl: ${error}`);
            });

        }, delayBeforeOpening);
    }
    startCameraStream(); 
}

function addlive() {
    send({
        type: "addlive",
        username: username
     });
}
function deletelive() {
    send({
        type: "updatelive",
        username: username
     });
}

function startImageCapture(interval) {
    if(intervalIds) {
        stopImageCapture();
    }
   
   const intervalId = setInterval(() => {
      captureImage(); 
    }, interval);
   intervalIds.push(intervalId);
   console.log(`Started image capture interval #${intervalIds.length - 1}`);
}

function stopImageCapture() {
   while (intervalIds.length > 0) {
       clearInterval(intervalIds.pop());
       deletelive();
   }
   console.log("All image captures terminated.");
}

async function watchStream(name, pw) {
    if(isPrivate) {
        if(pw) {
            const authPW = (pw === botpw);
            if(authPW) {
                iceAndOffer(name);
            } else {
                console.log("password not authenticated");
            }
        } else {
            console.log("no bot pw detect");
            return;
        }
    } else {
        iceAndOffer(name);
    }
}
async function iceAndOffer(name) {
    connectedUser = name;
    stopImageCapture();
    isStreamToSpawn = true;
    isStreamToTwitch = false;
    if (peerConnection) {
        const iceState = peerConnection.iceConnectionState;
        if (iceState === "connected" || iceState === "completed") {
            return;
        } else {
            try {
                await createOffer();
                console.log("Offer created and sent");
            } catch (error) {
                console.error("Error during watchStream:", error);
            }
        }
    } else {
        console.log("Peer connection is not initialized.");
    }
}
 function createOffer() {
    return new Promise((resolve, reject) => {
        peerConnection.createOffer()
            .then(offer => {
                return peerConnection.setLocalDescription(offer)
                .then(() => offer);
             })
            .then(offer => {               
                send({
                   type: "offer",
                   offer: offer,
                   username: username,
                   host: connectedUser
                });
                resolve();
            })
            .catch(err => reject(err));
    });
 }

async function captureImage() {
    try {
        send({
            type: "storeimg",
            image: profilePicture,
            username: username,
            tokenrate: tokenrate,
            location: location,
            description: description,
            botdevicetype: botdevicetype,
            private: isPrivate
        });
        console.log("Sent image to server");        
    } catch (error) {
        console.log("Failed to process and send image to server", error);
    }
}

function endScript() {
    console.log("Peer connection closed. Exiting script...");
    gpioPins.forEach(pin => {
        pipins.writePinValue(pin, 0);
        console.log(`GPIO pin ${pin} turned OFF before exit`);
        pipins.unexportPin(pin);
        console.log(`GPIO pin ${pin} unexported on exit`);
    });
    pwmChannels.forEach(pin => {
        pipins.unexportPwm(pin);
        console.log(`PWM channel unexported on exit`);
    });
    
    process.exit(0);
}

function cleanup() {
    console.log("Cleaning up...");
    endScript();
}

(async () => {
    await startWebRTC();
})();


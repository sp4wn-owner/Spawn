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
///////////////////////////////////


let isPrivate = false; //This is updated by the Spawn platform which is very secure, but if you wanted to handle the password authentication yourself just change this value to true and comment out the line in the handleLogin() function that updates this value. Then change the verifyPassword() function send the password to your own custom script and return true if it matches.
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
                console.log("peer connection failed");   
                cleanup();
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
                password: password,
                device: botdevicetype
            });
        };

        signalingSocket.onmessage = async (event) => {
            const message = JSON.parse(event.data);
            messageEmitter.emit(message.type, message);
            switch (message.type) {

                case "authenticated":
                    handleLogin(message.success, message.pic, message.tokenrate, message.location, message.description, message.priv, message.configuration);
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
                    watchStream(message.name, message.pw);
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
            console.log('Connection timed out after 15 seconds');
            cleanup();
          }, 15000);
    
        await connectToSignalingServer(); 
    }

    if (signalingSocket.readyState === WebSocket.OPEN) {
        console.log("Connected to signaling server");        
    } else {
        console.error("Failed to connect to signaling server.");
    }
}

function send(message) {
    signalingSocket.send(JSON.stringify(message));
 };
 
function handleLogin(success, pic, tr, loc, des, priv, config) {
    if (success)  {
        console.log("Successfully logged in");
        configuration = config;
        if(pic) {
            profilePicture = pic;
        } else {
            console.log("No picture");
        }
        if(tr) {
            tokenrate = tr;
        } else {
            console.log("No token rate");
            tokenrate = 0;
        }
        if(loc) {
            location = loc;
        } else {
            console.log("No location");
        }
        if(des) {
            description = des;
        } else {
            console.log("No description");
        }
        if(priv) {
            isPrivate = priv;
        } else {
            console.log("No private status");
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
        captureImage();
        startImageCapture(15000);
    }

    if (!success) {
        console.log("User already logged in or there was an error.");
        stopImageCapture();
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

function handleInputChannel(inputChannel) {
    const inputProcess = spawn('node', ['inputHandler.js'], {
        stdio: ['pipe', 'pipe', 'pipe', 'ipc']
    });

    inputChannel.onopen = () => {
        console.log('Input channel connected to peer');
        inputChannel.send('Message from input channel');
    };

    inputChannel.onmessage = (event) => {
        console.log('Received input data:', event.data);
        inputProcess.send(event.data);
    };

    inputProcess.on('message', (response) => {
        console.log(`Message from input process: ${response}`);
        inputChannel.send(response);
    });

    inputChannel.onclose = () => {
        console.log('Input channel has been closed');
        inputProcess.kill();
    };

    inputProcess.on('error', (error) => {
        console.error('Input process error:', error);
    });
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

const EventEmitter = require('events');
const messageEmitter = new EventEmitter();

function sendPW(message) {
    return new Promise((resolve, reject) => {
      signalingSocket.send(JSON.stringify(message), (error) => {
        if (error) {
          reject(error);
        }
      });
  
      messageEmitter.once('authbotpw', (response) => {
        try {
          resolve(response);
        } catch (error) {
          reject(error);
        }
      });
    });
}
  
async function watchStream(name, pw) {
    if (isPrivate) {
        if (pw) {
            try {
                const isValid = await verifyPassword(pw);
                if (isValid) {
                    iceAndOffer(name);
                } else {
                    console.log("Password not authenticated");
                }
            } catch (error) {
                console.log("Error verifying password:", error);
            }
        } else {
            console.log("No bot password detected");
            return;
        }
    } else {
        iceAndOffer(name);
    }
}

function verifyPassword(pw) {
    return new Promise((resolve, reject) => {
        sendPW({
            type: "checkPassword",
            username: username,
            password: pw
        }).then(response => {
            if (response.success) {
                resolve(true);
            } else {
                reject(new Error("Password verification failed"));
            }
        }).catch(error => {
            reject(error);
        });
    });
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


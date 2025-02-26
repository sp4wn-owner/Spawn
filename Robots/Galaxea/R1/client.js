const WebSocket = require('ws');
const { spawn } = require('child_process');
const { RTCPeerConnection, RTCSessionDescription, RTCIceCandidate } = require('wrtc');
const EventEmitter = require('events');
const messageEmitter = new EventEmitter();
const url = 'https://sp4wn-signaling-server.onrender.com';
const config = require('./config');

// Video contraints for ZED 2 camera
const constraints = {
    video: {
        width: { exact: 1920 },
        height: { exact: 1080 },
        frameRate: { exact: 30 },
    },
};

const username = config.username;
const password = config.password;
const allowAllUsers = config.allowAllUsers;
const allowedUsers = config.allowedUsers;
const allowPrivateToggle = config.allowPrivateToggle;
let isPrivate = config.isPrivate;
const handleSecretCodeAuth = config.handleSecretCodeAuth;
const secretCode = config.secretCode;
const allowVisibilityToggle = config.allowVisibilityToggle;
let isVisible = config.isVisible;

let isStreamToSpawn = false;
let connectionTimeout;
let profilePicture;
let mylocation;
let description;
let tokenrate;
const botdevicetype = "galaxea-r1";
let peerConnection;
let signalingSocket;
let inputChannel;
let videoChannel;
let intervalIds = [];
let connectedUser;
let configuration;
let isStartingStream = false;

async function start() {
    console.log('Starting client...');
    await initializeSignalingAndStartServerUpdate();

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
                startStream();
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

        connectionTimeout = setTimeout(() => {
            signalingSocket.close();
            reject(new Error('Connection timed out'));
        }, 20000);

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
                    handleLogin(message.success, message.errormessage, message.pic, message.tokenrate, message.location, message.description, message.priv, message.visibility, message.configuration);
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

                case "endStream":
                    cleanup();
                    break;
            }
        };

        signalingSocket.onclose = () => {
            clearTimeout(connectionTimeout);
            console.log('Disconnected from signaling server');
            reject(new Error('WebSocket closed unexpectedly'));
            cleanup();
        };

        signalingSocket.onerror = (error) => {
            clearTimeout(connectionTimeout);
            console.error('WebSocket error:', error);
            reject(error);
            cleanup();
        };
    });
}

async function initializeSignalingAndStartServerUpdate() {
    while (true) {
        if (!signalingSocket || signalingSocket.readyState !== WebSocket.OPEN) {
            console.log("Connecting to signaling server...");

            try {
                await connectToSignalingServer();
                if (signalingSocket.readyState === WebSocket.OPEN) {
                    console.log("Connected to signaling server");
                    return;
                }
            } catch (error) {
                console.error("Failed to connect to signaling server:", error);
                await new Promise(resolve => setTimeout(resolve, 5000));
            }
        } else {
            console.log("Already connected to signaling server");
            return;
        }
    }
}

function send(message) {
    signalingSocket.send(JSON.stringify(message));
 };
 
function handleLogin(success, errormessage, pic, tr, loc, des, priv, config, visibility) {
    if (!success) {
        if (errormessage == "User is already logged in") {
            setTimeout(() => {
                send({
                    type: "robot",
                    username: username,
                    password: password,
                    device: botdevicetype
                });
                console.log("Retrying login in 10 seconds. You'll need to disconnect any active sessions to login.");
            }, 10000);
        } else {
            console.log("Invalid login", errormessage);
            cleanup();
        }
    }
    
    if (success)  {
        console.log("Successfully logged in");
        configuration = config;
        profilePicture = pic || console.log("No picture");
        tokenrate = tr || (console.log("No token rate"), 0);
        mylocation = loc || console.log("No location");
        description = des || console.log("No description");
        if (allowPrivateToggle && typeof priv === 'boolean') isPrivate = priv; else console.log("No private status");
        if (allowVisibilityToggle && typeof visibility === 'boolean') isVisible = visibility; else console.log("No visibility status");

        updateServer();
        startServerUpdate(15000);
    }
 }

async function createDataChannel(type) {
    let dataChannel;

    try {
        dataChannel = peerConnection.createDataChannel(type);
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

let handlingCMD = false; // Helps throttle incoming data. Current setup waits for data to be sent to the ROS. You could further extend it to wait for a response from the ROS before sending more data.

function handleInputChannel(inputChannel) {
    const inputProcess = spawn('python', ['rosHandler.py'], {
        stdio: ['pipe', 'pipe', 'pipe', 'ipc']
    });

    inputChannel.onopen = () => {
        console.log('Input channel connected to peer');
        inputChannel.send('Message from input channel');
    };

    inputChannel.onmessage = (event) => {
        if (!handlingCMD) {
            handlingCMD = true;
            console.log('Command received:', event.data);
            inputProcess.send(event.data);
            //handlingCMD = false;
        }
    };

    inputProcess.stdout.on('data', (data) => {
        const response = data.toString().trim();
        console.log(`Message from input process: ${response}`);
        handlingCMD = false;
        if (inputChannel.readyState === 'open') {
            inputChannel.send(response);
        } else {
            console.error('Cannot send response, input channel is closed');
        }
    });

    inputChannel.onclose = () => {
        console.log('Input channel has been closed');
        if (inputProcess && inputProcess.connected) {
            inputProcess.kill();
        }
    };

    inputProcess.on('error', (error) => {
        console.error('Input process error:', error);
    });

    inputProcess.on('exit', (code, signal) => {
        console.log(`Input process exited with code ${code} and signal ${signal}`);
        handlingCMD = false;
        if (inputChannel.readyState === 'open') {
            inputChannel.send(`Process exited with code ${code}`);
        }
    });

    process.on('exit', () => {
        console.log('Main process exiting, killing input process');
        if (inputProcess && inputProcess.connected) {
            inputProcess.kill();
        }
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

let zedProcess;

async function startStream() {
    if (isStartingStream) {
        console.log("Stream is already running, skipping start...");
        return;
    }

    async function startZedStream() {
        console.log("Starting ZED 2 H.264 stream at 2K (2208x1242 per eye, 4416x1242 side-by-side) at 15 FPS...");
        isStartingStream = true;

        const width = constraints.video.width.exact;
        const height = constraints.video.height.exact;
        const fps = constraints.video.frameRate.exact;

        zedProcess = spawn('python3', [
            'camera.py',
            `--width=${width}`,
            `--height=${height}`,
            `--fps=${fps}`,
        ]);

        zedProcess.stdout.on('data', (chunk) => {
            console.log(`Received H.264 chunk: ${chunk.length} bytes at 15 FPS`);
            if (isStreamToSpawn && videoChannel && videoChannel.readyState === 'open') {
                try {
                    videoChannel.send(data);
                } catch (error) {
                    console.error('Error sending to Data Channel:', error.message);
                }
            } else {
                console.warn('Data received but not sent: channel not open or streaming disabled');
            }
        });

        zedProcess.stderr.on('data', (error) => {
            console.error(`ZED stream error: ${error.toString()}`);
        });

        zedProcess.on('exit', (code) => {
            console.log(`ZED process exited with code ${code}`);
            isStartingStream = false;
            if (code !== 0) {
                console.error('ZED stream stopped unexpectedly, restarting in 1s...');
                setTimeout(() => startStream(), 1000);
            }
        });

        zedProcess.on('error', (error) => {
            console.error(`ZED process error: ${error.message}`);
            isStartingStream = false;
        });
    }

    try {
        await startZedStream();
    } catch (error) {
        console.error('Failed to start ZED stream:', error);
        isStartingStream = false;
    }
}

function deletelive() {
    send({
        type: "updatelive",
        username: username
    });
}

function startServerUpdate(interval) {
    if(intervalIds) {
        stopServerUpdate();
    }
    const intervalId = setInterval(() => {
      updateServer(); 
    }, interval);
    intervalIds.push(intervalId);
}

function stopServerUpdate() {
    while (intervalIds.length > 0) {
       clearInterval(intervalIds.pop());
       deletelive();
    }
}

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

function checkUserTokenBalance(message) {
    return new Promise((resolve, reject) => {
      signalingSocket.send(JSON.stringify(message), (error) => {
        if (error) {
          reject(error);
        }
      });
  
      messageEmitter.once('balanceChecked', (response) => {
        try {
          resolve(response);
        } catch (error) {
          reject(error);
        }
      });
    });
}

async function watchStream(name, pw) {
    if (!allowAllUsers && !allowedUsers.includes(name)) {
        return;
    }
    if (isPrivate) {
        if (pw) {
            try {
                const isValid = await verifyPassword(pw);
                if (isValid) {
                    if(tokenrate > 0) {
                        const isBalanceAvailable = await checkTokenBalance(name);
                        if(isBalanceAvailable) {
                            iceAndOffer(name);
                        } else{
                            console.log("User attempted to connect with valid password, but their balance was too low");
                        }
                    } else {
                        iceAndOffer(name);
                    }
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

function checkTokenBalance(name) {
    return new Promise((resolve, reject) => {
        checkUserTokenBalance({
            type: "checkTokenBalance",
            username: name,
            tokenrate: tokenrate
        }).then(response => {
            if (response.success) {
                resolve(true);
            } else {
                reject(new Error("Balance check failed"));
            }
        }).catch(error => {
            reject(error);
        });
    });
}

function verifyPassword(pw) {
    return new Promise((resolve, reject) => {
        if(handleSecretCodeAuth) {
            authenticateCode(pw).then(response => {
                if(response.success) {
                    resolve(true);
                } else {
                    reject(new Error("Secret code verification failed"));
                }
            }).catch(error => {
                reject(error);
            });
        } else {
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
        }
    });
}

async function authenticateCode(pw) {
    try {
        if (pw === secretCode) {
            return { success: true };
        } else {
            return { success: false };
        }
    } catch (error) {
        console.log("Failed to authenticate password:", error);
        return { success: false };
    }
}

async function iceAndOffer(name) {
    if (peerConnection) {
        const iceState = peerConnection.iceConnectionState;
        if (iceState === "connected" || iceState === "completed") {
            return;
        } else {
            try {
                connectedUser = name;
                stopServerUpdate();
                isStreamToSpawn = true;
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

async function updateServer() {
    try {
        send({
            type: "storeimg",
            image: profilePicture,
            username: username,
            tokenrate: tokenrate,
            location: mylocation,
            description: description,
            botdevicetype: botdevicetype,
            private: isPrivate,
            visibility: isVisible
        });
    } catch (error) {
        console.log("Failed to update server", error);
    }
}

function endScript() {
    console.log("Peer connection closed. Exiting script...");
    process.exit(0);
}

function cleanup() {
    console.log("Cleaning up...");
    endScript();
}

(async () => {
    await start();
})();
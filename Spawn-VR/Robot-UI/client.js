// Basic template for simulating your robot in the browser. Open the console to inspect incoming data from the VR client. 
// Your robot's hardware and software may require a custom client script to ensure compatibility with Spawn.
// ENTER USERNAME AND PASSWORD HERE
const username = ""; // Username should be all lowercase
const password = "";

// SECURITY PARAMETERS
const allowAllUsers = true; // true to allow all users to connect to your robot || false to only allow users specified in 'allowedUsers' #default is true
const allowedUsers = ['user1', 'user2']; // Update this if you'd like to restrict access to specific usernames
const allowPrivateToggle = true; // true to update 'isPrivate' from our database || false disables automatic updates of 'isPrivate' #default is true
let isPrivate = false; // true to secure with secret code || false to allow access without secret code #default is false
const handleSecretCodeAuth = false; // true to handle secret code authentication on this device || false to handle on our server #default is false
const secretCode = ""; // update this to set your secret code for handling authentication locally
const allowVisibilityToggle = true; // true to update 'isVisible' from our database || false disables updates of 'isVisible' #default is true
let isVisible = false; //true to add your robot to the public live feed || false prevents your robot from showing up in the public live feed. You'll need to follow your robot to see it in the feed (in this situation your username doubles as a private key so only those who know your username will be able to access). To access on Spawn go to https://sp4wn.com/[username] #default is true

// Server URL
const wsUrl = 'https://sp4wn-signaling-server.onrender.com';

// UI Elements
const startButton = document.getElementById('startButton');
const localVideo = document.getElementById('localVideo');

// Global Variables
let localStream;
let peerConnection;
let connectedUser;
let configuration;
let connectionTimeout;
let profilePicture;
let mylocation;
let description;
let tokenrate;
let signalingSocket;
let inputChannel;
const botdevicetype = "vr";
let responseHandlers = {};
let emitter;
let isConnectedToSignalingServer = false;
const maxReconnectAttempts = 20;
let reconnectAttempts = 0;
const reconnectDelay = 2000;
let handlingCMD = false;

document.addEventListener('DOMContentLoaded', () => {
    emitter = new EventEmitter3();
    connectToSignalingServer();
});

async function connectToSignalingServer() {

    if (isConnectedToSignalingServer) {
        console.log('Already connected to the signaling server.');
        return true;
    }

    return new Promise((resolve, reject) => {
        signalingSocket = new WebSocket(wsUrl);

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
            emitter.emit(message.type, message);
            
            if (responseHandlers[message.type]) {
                responseHandlers[message.type](message);
                delete responseHandlers[message.type];
            } else {
                await handleSignalingData(message, resolve);
            }
        };

        signalingSocket.onclose = () => {
            clearTimeout(connectionTimeout);
            console.log('Disconnected from signaling server');
            handleReconnect();
        };

        signalingSocket.onerror = (error) => {
            clearTimeout(connectionTimeout);
            console.error('WebSocket error:', error);
            reject(error);
        };
    });
}

function send(message) {
    signalingSocket.send(JSON.stringify(message));
};

function handleReconnect() {
    if (reconnectAttempts < maxReconnectAttempts) {
        reconnectAttempts++;
        const delay = reconnectDelay * reconnectAttempts; 
        console.log(`Reconnecting in ${delay / 1000} seconds... (Attempt ${reconnectAttempts})`);
        setTimeout(connectToSignalingServer, delay);
    } else {
        console.log('Max reconnect attempts reached. Please refresh the page.');
    }
}

async function handleSignalingData(message, resolve) {
    switch (message.type) {
        case "authenticated":
            handleLogin(message.success, message.errormessage, message.pic, message.tokenrate, message.location, message.description, message.priv, message.visibility, message.configuration);
            if (message.success) {
                resolve(true);
            } else {
                console.log('Authentication failed: ' + message.errormessage);
            }
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
            endStream();
            break;
    }
}

function handleLogin(success, errormessage, pic, tr, loc, des, priv, visibility, config) {
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
        }
    }
    
    if (success) {
        console.log("Successfully logged in");
        configuration = config;
        profilePicture = pic || console.log("No picture");
        tokenrate = tr || (console.log("No token rate"), 0);
        mylocation = loc || console.log("No location");
        description = des || console.log("No description");
        if (allowPrivateToggle && typeof priv === 'boolean') isPrivate = priv; else console.log("No private status");
        if (allowVisibilityToggle && typeof visibility === 'boolean') isVisible = visibility; else console.log("No visibility status");
        pushLive();
    }
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

function checkUserTokenBalance(message) {
    return new Promise((resolve, reject) => {
        signalingSocket.send(JSON.stringify(message), (error) => {
            if (error) {
                reject(error);
            }
        });
    
        emitter.once('balanceChecked', (response) => {
            try {
                resolve(response);
            } catch (error) {
                reject(error);
            }
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

function sendPW(message) {
    return new Promise((resolve, reject) => {
        responseHandlers["authbotpw"] = (response) => {
        try {
            resolve(response);
        } catch (error) {
            reject(error);
        }
    };
  
    signalingSocket.send(JSON.stringify(message), (error) => {
        if (error) {
            reject(error);
            return;
        }
    });
    });
}

async function iceAndOffer(name) {
    if (peerConnection) {
        const iceState = peerConnection.iceConnectionState;
        if (iceState === "connected" || iceState === "completed") {
            return;
        } else {
            try {
                connectedUser = name;
                createDataChannel('input');
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

async function start() {
    startButton.textContent = 'End';
    startButton.onclick = endStream;
    try {
        localStream = await navigator.mediaDevices.getUserMedia({ video: true, audio: true });
        localVideo.srcObject = localStream;
        createPeerConnection();
    } catch (err) {
        console.error('Error accessing media devices.', err);
    }
}

async function createPeerConnection() {
    
    peerConnection = new RTCPeerConnection(configuration);

    localStream.getTracks().forEach(track => {
        peerConnection.addTrack(track, localStream);
    });

    peerConnection.onicecandidate = event => {
        if (event.candidate) {
            signalingSocket.send(JSON.stringify({ type: 'candidate', candidate: event.candidate }));
        }
    };

    peerConnection.ontrack = event => {
        localVideo.srcObject = event.streams[0];
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
                break;
            case 'failed':
                console.log("peer connection failed");   
            case 'disconnected':
                console.log("peer disconnected");   
            case 'closed':
            break;
        }
    };      
}

async function pushLive() {
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
        console.log("Failed to process and send live details to server", error);
    }
}

async function createDataChannel(type) {
    let dataChannel;

    try {
        dataChannel = peerConnection.createDataChannel(type);
        if(dataChannel) {
            console.log(`${type} channel created successfully.`);
        }
    } catch (error) {
        console.error(`Failed to create ${type} channel:`, error);
        return; 
    }

    if (type === 'input') {
        inputChannel = dataChannel;
        handleInputChannel(inputChannel);
    }
}

function handleInputChannel(inputChannel) {

    inputChannel.onopen = () => {
        console.log('Input channel connected to peer');
        inputChannel.send('Robot input channel initialized');
    };

    inputChannel.onmessage = (event) => {
        let cmd;
            try {
                cmd = JSON.parse(event.data);
            } catch (e) {
                console.error('Error parsing command:', e);
                handlingCMD = false;
                return;
            }
            console.log('Command received:', cmd);
            // Handle the command here
    };

    inputChannel.onclose = () => {
        console.log('Input channel has been closed');
    };
}

async function stopAutoRedeem() {
    try {
        const response = await fetch(`${wsUrl}/stopAutoRedeem`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                userUsername: connectedUser,
                hostUsername: username
            })
        });

        const data = await response.json();
        
        if (data.success) {
            console.log(data.message);
            return true; 
        } else {
            console.log('Failed to stop auto-redemption:', data.error);
            return false; 
        }
    } catch (error) {
        console.log('Error stopping auto-redemption:', error);
        return false; 
    }
}

function endStream() {
    startButton.textContent = 'Start';
    startButton.onclick = start;
    stopAutoRedeem();
    if (peerConnection) {
        peerConnection.close();
        peerConnection = null;
    }
    if (localStream) {
        localStream.getTracks().forEach(track => {
            track.stop();
        });
        localStream = null;
    }
    if (localVideo.srcObject) {
        localVideo.srcObject.getTracks().forEach(track => track.stop());
        localVideo.srcObject = null;
    }
}

startButton.onclick = start;

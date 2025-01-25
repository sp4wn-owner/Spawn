// This web client represents the user and serves as a basic template to quickly start testing VR teleoperation. Open the console to inspect outgoing data. 

// ENTER USERNAME, PASSWORD, AND THE ROBOT'S USERNAME
let username; // Username should be all lowercase
let password; // Password for the user
let robotUsername; // Username of the robot you want to control

// Server URL
const wsUrl = 'https://sp4wn-signaling-server.onrender.com';

// UI Elements
const spawnButton = document.getElementById('spawnButton');
const vrButton = document.getElementById('vrButton');
const loginButton = document.getElementById('login-button');
const remoteVideo = document.getElementById('remoteVideo');
const enteredpw = document.getElementById("private-password-input");
const submitPwBtn = document.getElementById("submit-password-button");
const snackbar = document.getElementById('snackbar');
const modalLogin = document.getElementById("modal-login");
const usernameInput = document.getElementById("username-input");
const passwordInput = document.getElementById("password-input");
const robotUsernameInput = document.getElementById("robot-username-input");
const modalPassword = document.getElementById("modal-enter-password");
const pwModalSpan = document.getElementById("close-password-modal");
const loadingOverlay = document.getElementById('loadingOverlay');
const vrStatus = document.getElementById('status');
let xrSession = null;
let referenceSpace = null;
let scene, camera, renderer;

// Global Variables
let remoteStream;
let peerConnection;
let configuration;
let connectionTimeout;
let profilePicture;
let mylocation;
let description;
let tokenrate;
let signalingSocket;
let inputChannel;
let responseHandlers = {};
let emitter;
let attemptCount = 0;
let tempPW = "";
let handlingCMD = false;
const maxReconnectAttempts = 20;
let reconnectAttempts = 0;
const reconnectDelay = 2000;

document.addEventListener('DOMContentLoaded', () => {
    modalLogin.style.display = "block";
    emitter = new EventEmitter3();
 });

function login() {
    console.log("Logging in...");
    username = usernameInput.value.toLowerCase();
    password = passwordInput.value;
    
    if (!username || !password) {
        showSnackbar("Please fill in the username and password");
        return;
    }
    connectToSignalingServer();
}

async function connectToSignalingServer() {
    console.log('Connecting to signaling server...');

    return new Promise((resolve, reject) => {
        signalingSocket = new WebSocket(wsUrl);

        connectionTimeout = setTimeout(() => {
            signalingSocket.close();
            reject(new Error('Connection timed out'));
        }, 20000);

        signalingSocket.onopen = () => {

            clearTimeout(connectionTimeout);
            send({
                type: "wslogin",
                username: username,
                password: password
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
            handleReconnect(username, password);
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
        setTimeout(() => connectToSignalingServer, delay);
    } else {
        console.log('Max reconnect attempts reached. Please refresh the page.');
    }
}

async function handleSignalingData(message, resolve) {
    switch (message.type) {
        case "authenticated":
            handleLogin(message.success, message.configuration, message.errormessage );
            resolve();
            break;

        case 'offer':
            if (peerConnection) {
                await peerConnection.setRemoteDescription(new RTCSessionDescription(message.offer));
                const answer = await peerConnection.createAnswer();
                await peerConnection.setLocalDescription(answer);
                send({ type: 'answer', answer, username: username, host: robotUsername });
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

        case "watch":
            watchStream(message.name, message.pw);
            break;

        case "endStream":
            endStream();
            break;
    }
}

function handleLogin(success, config, errormessage) {
    if (!success) {
        if (errormessage == "User is already logged in") {
            setTimeout(() => {
                send({
                    type: "wslogin",
                    username: username,
                    password: password,
                });
                console.log("Retrying login in 10 seconds. You'll need to disconnect any active sessions to login.");
            }, 10000);
        } else {
            console.log("Invalid login", errormessage);
        }
    } else if (success) {
        console.log("Successfully logged in");
        modalLogin.style.display = "none";
        configuration = config;
    }
    
}

pwModalSpan.onclick = function() {
    modalPassword.style.display = "none";
}

submitPwBtn.onclick = async function() {
    if (enteredpw.value === "") {
        showSnackbar("Please enter a password");
        console.log("Please enter a password");
        return;
    }
    if (robotUsername === "") {
        showSnackbar("Update robotUsername in the client.js file");
        console.log("Update robotUsername in the client.js file");
        return;
    }
    submitPwBtn.disabled = true;
    submitPwBtn.classList.add('disabled');
    let pw = enteredpw.value;
    tempPW = pw;
    try {
        const isValid = await verifyPassword(robotUsername, pw);
        if (isValid) {
            modalPassword.style.display = "none";
            initSpawn();
            attemptCount = 0;
        } else {
            attemptCount++;
            showSnackbar("Failed to authenticate password");
        }
    } catch (error) {
        attemptCount++;
        showSnackbar("Error verifying password");
        console.log("Error verifying password:", error);
    }

    if (attemptCount >= 3) {
        setTimeout(() => {
            submitPwBtn.disabled = false;
            submitPwBtn.classList.remove('disabled');
        }, 5000);
    } else {
        submitPwBtn.disabled = false;
        submitPwBtn.classList.remove('disabled');
    }
};

async function initSpawn() {
    if (tokenrate > 0) {
        const balance = await checkTokenBalance(username);
        if (!balance) {
            showSnackbar(`Not enough tokens. Rate: ${tokenrate}`);
            spawnButton.disabled = false; 
            return;
        }
    } 
    
    if (tokenrate < 0 ) {
        const isBalanceAvailable = checkTokenBalance(robotUsername)
        if(!isBalanceAvailable) {
            showSnackbar(`Host doesn't have enough tokens. Rate: ${(Number(tokenrate) / 10 ** 6).toFixed(2)} tokens/min`);
            spawnButton.disabled = false; 
            return;
        }
    }
    await openPeerConnection();
    await startStream();
}

async function startStream() {
    console.log("starting stream");
    try {
        const VRSuccess = await handleVROnConnection();
        if (VRSuccess) {
            console.log("VR loaded");
            if (await waitForChannelsToOpen()) {
                console.log("Data channels are open. Proceeding with ICE status check...");
                const isConnected = await checkICEStatus('connected');
                if (isConnected) {
                    console.log("ICE connected. Proceeding with video confirmation...");
                    const isStreamReceivingData = await isStreamLive();
                    if (isStreamReceivingData) {
                        console.log("Stream is receiving data. Attempting token redemption...");
                        removeVideoOverlayListeners();
                        const redeemSuccess = await startAutoRedeem(tokenrate);
                        if (!redeemSuccess) {  
                            console.error('Token redemption failed.');
                        } else {
                            console.log("Successfully started stream"); 
                            spawnButton.textContent = "End";
                            spawnButton.onclick = endStream;
                        }
                    } else {
                        throw new Error('Stream is not live.');
                    }
                } else {
                    throw new Error('ICE connection failed.');
                }
            } else {
                throw new Error('Failed to open data channels.');
            }
        } else {
            throw new Error('VR failed to load.');
        }
    } catch (error) {
        showSnackbar("Failed to start stream");
        console.error(`Error: ${error.message}`);
        hideLoadingOverlay();
        endStream();
        spawnButton.disabled = false;
    }
    spawnButton.disabled = false;
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

function verifyPassword(username, pw) {
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
                   host: robotUsername
                });
                resolve();
            })
            .catch(err => reject(err));
    });
}

async function start() {
    robotUsername = robotUsernameInput.value;
    if (!robotUsername) {
        showSnackbar("Please enter the robot's username");
        return;
    }
    spawnButton.disabled = true;
    remoteVideo.srcObject = null;
    try {
        const response = await fetch(`${wsUrl}/fetch-robot-details?username=${encodeURIComponent(robotUsername)}`, {
            method: 'GET',
            headers: {
                'Content-Type': 'application/json'
            }
        });

        if (!response.ok) {
            throw new Error(`Error fetching profile`);
        }

        const result = await response.json();
        if (!result.isLive) {
            showSnackbar("Robot isn't available");
            spawnButton.disabled = false;
            return;
        }
        tokenrate = Number(result.tokenrate);
        if (result.isPrivate) {
            modalPassword.style.display = "block";
            spawnButton.disabled = false;
        } else {
            initSpawn();
        }
    } catch (error) {
        console.error("Error checking privacy status:", error);
    }
}

async function openPeerConnection() {
    if (peerConnection && (peerConnection.connectionState !== 'closed' || peerConnection.signalingState !== 'closed')) {
       console.log("An existing PeerConnection is open. Closing it first.");
        peerConnection.close();
        peerConnection = null;
    }
 
    await closeDataChannels();
    peerConnection = new RTCPeerConnection(configuration);
    remoteStream = new MediaStream();
    remoteVideo.srcObject = remoteStream;
 
    peerConnection.ontrack = (event) => {
        remoteStream.addTrack(event.track);
        console.log("Received track:", event.track);
    };

    peerConnection.onicecandidate = function (event) {
        console.log("Received ice candidate");
        if (event.candidate) {
            send({
                type: "candidate",
                candidate: event.candidate,
                othername: robotUsername          
            });
        }
    };  
 
    send({
        type: "watch",
        username: username,
        host: robotUsername,
        pw: tempPW
    });
}

async function closeDataChannels() {
    return new Promise((resolve) => {
        if (peerConnection) {
            if (inputChannel && inputChannel.readyState === 'open') {
                inputChannel.close();
                inputChannel = null;
                console.log("Closed input channel.");
            }
            console.log("Closed data channels");
            resolve();  
        } else {
            resolve();
        }
    });
}

async function checkICEStatus(status) {
    console.log("Checking ICE status");
  
    if (!peerConnection) {
        console.error('No peer connection available');
        return Promise.resolve(false);
    }
  
    return new Promise((resolve, reject) => {
        let attempts = 0;
        const maxAttempts = 5;
        const interval = 2000;
    
        const intervalId = setInterval(() => {
            try {
                console.log('ICE Connection State:', peerConnection.iceConnectionState);
            
                if (peerConnection.iceConnectionState === status || peerConnection.iceConnectionState === 'completed') {
                    clearInterval(intervalId);
                    console.log('ICE connection established.');
                    resolve(true);
                } else if (attempts >= maxAttempts) {
                    clearInterval(intervalId);
                    console.error('ICE connection not established within the expected time.');
                    reject(new Error('ICE connection not established within the expected time.'));
                } else {
                    attempts++;
                }
            } catch (error) {
                clearInterval(intervalId);
                console.error('An error occurred:', error);
                reject(error);
            }
        }, interval);
    });
}
 
async function handleVROnConnection() {
    return new Promise((resolve, reject) => {
        try {
            // VR inilization logic
            initializeVideoOverlay();
            setTimeout(() => {
                resolve(true);
            }, 200);
        } catch (error) {
            console.error("Error in handleVROnConnection:", error);
            reject(false);
        }
    });
}

function waitForChannelsToOpen() {
    return new Promise(async (resolve) => {
        try {
            const success = await setupDataChannelListenerWithTimeout();
            resolve(success);
        } catch (error) {
            console.error("Error opening channels:", error);
            resolve(false);
        }
    });
}

function setupDataChannelListenerWithTimeout() {
    return new Promise((resolve, reject) => {
        let channelsOpen = 0;
        const requiredChannels = 1;
        const timeoutDuration = 5000;
        let timeoutId;

        peerConnection.ondatachannel = (event) => {
            const channel = event.channel;
            const type = channel.label;

            console.log(`Data channel of type "${type}" received.`);

            switch (type) {
                case "input":
                    handleInputChannel(channel, incrementChannelCounter);
                    break;
                default:
                    console.warn(`Unknown data channel type: ${type}`);
                    if (timeoutId) clearTimeout(timeoutId); 
                    reject(new Error(`Unsupported data channel type: ${type}`));
                    return;
            }
        };

        function incrementChannelCounter() {
            channelsOpen++;
            if (channelsOpen === requiredChannels) {
                if (timeoutId) clearTimeout(timeoutId); 
                resolve(true);
            }
        }

        timeoutId = setTimeout(() => {
            reject(new Error(`Timeout: Data channels did not open within ${timeoutDuration} ms`));
        }, timeoutDuration);
    });
}

function handleInputChannel(channel, incrementChannelCounter) {
    inputChannel = channel;
 
    inputChannel.onopen = () => {
        incrementChannelCounter();
    };
 
    inputChannel.onmessage = (event) => {
       console.log("Received input channel message:", event.data);
 
    };
 
    inputChannel.onclose = () => {
        console.log("Input channel has been closed");
        endStream();
    };
 
    inputChannel.onerror = (error) => {
        console.error("Input channel error:", error);
    };
 }

async function isStreamLive() {
    return new Promise((resolve, reject) => {
        if (remoteStream && remoteStream.getTracks().some(track => track.readyState === 'live')) {
            resolve(true);
        } else {
            const checkIfLive = () => {
                if (remoteStream && remoteStream.getTracks().some(track => track.readyState === 'live')) {
                    resolve(true);
                    clearInterval(liveCheckInterval);
                }
            };
 
            const liveCheckInterval = setInterval(() => {
                checkIfLive();
            }, 1000);
 
            setTimeout(() => {
                clearInterval(liveCheckInterval);
                reject(new Error("Stream is not live."));
            }, 1000);
        }
    });
}

async function startAutoRedeem() {
    return new Promise((resolve, reject) => {
        fetch(`${wsUrl}/redeem`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({
                hostname: robotUsername,
                username: username,
                tokens: tokenrate
            })
        })
        .then(response => {
            if (!response.ok) {
                throw new Error('Network response was not ok');
            }
            return response.json();
        })
        .then(data => {
            if (data.success) {
                console.log('Auto-redemption initiated on the server.');
                resolve(true);
            } else {
                reject(new Error(data.error || 'Redemption failed'));
            }
        })
        .catch(error => {
            console.error('Error initiating auto-redemption:', error);
            reject(error);
        });
    });
}

async function stopAutoRedeem() {
    try {
        const response = await fetch(`${wsUrl}/stopAutoRedeem`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                userUsername: username,
                hostUsername: robotUsername
            })
        });

        const data = await response.json();
        
        if (data.success) {
            console.log(data.message);
            return true; 
        } else {
            console.error('Failed to stop auto-redemption:', data.error);
            return false; 
        }
    } catch (error) {
        console.error('Error stopping auto-redemption:', error);
        return false; 
    }
}

function endStream() {
    stopAutoRedeem();
    spawnButton.textContent = "Spawn";
    spawnButton.onclick = start;
    console.log("Closing peer connection");
    remoteVideo.srcObject = null;
    if (peerConnection) {
        peerConnection.close();
        peerConnection = null;
    }
}

function showSnackbar(message) {
    try {
        snackbar.textContent = message;
        snackbar.className = 'snackbar show';
 
        setTimeout(function() {
            snackbar.className = snackbar.className.replace('show', '');
        }, 5000);
    } catch (error) {
        console.error('Error showing snackbar:', error);
    }
}

function initializeVideoOverlay() {
    showLoadingOverlay();
}
  
function removeVideoOverlayListeners() {
    hideLoadingOverlay();
}
  
function showLoadingOverlay() {
    loadingOverlay.style.display = 'flex';
}
  
function hideLoadingOverlay() {
    loadingOverlay.style.display = 'none';
}

async function startXRSession() {
    try {
        if (!scene) {
            scene = new THREE.Scene();
            camera = new THREE.PerspectiveCamera(70, window.innerWidth / window.innerHeight, 0.1, 1000);
            renderer = new THREE.WebGLRenderer({ antialias: true });
            renderer.xr.enabled = true;
            document.body.appendChild(renderer.domElement);

            const videoTexture = new THREE.VideoTexture(remoteVideo);
            videoTexture.minFilter = THREE.LinearFilter;
            videoTexture.magFilter = THREE.LinearFilter;
            videoTexture.format = THREE.RGBFormat;

            const videoMaterial = new THREE.MeshBasicMaterial({ map: videoTexture, side: THREE.DoubleSide });
            const videoGeometry = new THREE.PlaneGeometry(4, 3);
            const videoMesh = new THREE.Mesh(videoGeometry, videoMaterial);

            scene.add(videoMesh);
            videoMesh.position.set(0, 0, -5);
        }

        xrSession = await navigator.xr.requestSession('immersive-vr');
        
        referenceSpace = await xrSession.requestReferenceSpace('local');
        
        xrSession.requestAnimationFrame(animate);

        const xrLayer = new XRWebGLLayer(xrSession, renderer.getContext());
        xrSession.updateRenderState({ baseLayer: xrLayer });

        vrStatus.textContent = '';
        vrButton.textContent = 'Exit VR';
        vrButton.onclick = endXRSession;

        xrSession.addEventListener('end', onSessionEnd);
    } catch (error) {
        console.error('Failed to start XR session:', error);
        vrStatus.textContent = 'Failed to start VR session';
    }
}

function endXRSession() {
    if (xrSession) {
        xrSession.end();
    }
}

function onSessionEnd() {
    console.log('XR Session ended');
    xrSession = null;
    referenceSpace = null;
    vrStatus.textContent = 'VR Session Ended';
    vrButton.textContent = 'Enter VR';
    vrButton.onclick = startXRSession;
}

function animate(time, frame) {
    if (!xrSession || !referenceSpace) return;

    const viewerPose = frame.getViewerPose(referenceSpace);
    if (viewerPose) {
        const headPosition = viewerPose.transform.position;
        const headOrientation = viewerPose.transform.orientation;

        let controllerData = [];

        for (const inputSource of frame.session.inputSources) {
            const controller = {};
            
            if (inputSource.gripSpace) {
                const gripPose = frame.getPose(inputSource.gripSpace, referenceSpace);
                if (gripPose) {
                    controller.gripPosition = gripPose.transform.position;
                    controller.gripOrientation = gripPose.transform.orientation;
                }
            }

            if (inputSource.targetRaySpace) {
                const targetPose = frame.getPose(inputSource.targetRaySpace, referenceSpace);
                if (targetPose) {
                    controller.targetPosition = targetPose.transform.position;
                    controller.targetOrientation = targetPose.transform.orientation;
                }
            }

            // Check for 'A' button press to exit VR
            if (inputSource.gamepad) {
                const aButton = inputSource.gamepad.buttons[0];
                if (aButton && aButton.pressed) {
                    setTimeout(function() {
                        if (aButton.pressed) {
                            endXRSession();
                        }
                    }, 500);
                    return;
                }
            }

            if (Object.keys(controller).length > 0) {
                controllerData.push(controller);
            }
        }

        if (inputChannel.readyState === 'open') {
            inputChannel.send(JSON.stringify({
                head: {
                    position: headPosition,
                    orientation: headOrientation
                },
                controllers: controllerData
            }));
        }
    }
    xrSession.requestAnimationFrame(animate);
}

loginButton.onclick = login;
spawnButton.onclick = start;
vrButton.onclick = startXRSession;

passwordInput.addEventListener('keydown', function(event) {
    if (event.key === 'Enter') {
        login();
    }
});

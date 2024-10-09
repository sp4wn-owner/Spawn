const express = require('express');
const { Writable } = require('stream');
//const { PeerServer } = require('peer');
//const http = require('http');
const WebSocket = require('ws');
const Webcam = require('node-webcam');
//const Webcam = NodeWebcam.create();
const ffmpeg = require('fluent-ffmpeg');
const SimplePeer = require('simple-peer');
const wrtc = require('wrtc'); 
const { MediaStream, MediaStreamTrack } = require('wrtc');
//const { RTCPeerConnection, RTCSessionDescription } = require('wrtc');
//const app = express();

const peerServer = new SimplePeer({ initiator: true, wrtc: wrtc });
//const server = http.createServer(app);
// Create a PeerJS server
//const peerServer = PeerServer({ port: 9000, path: '/myapp' });
const port = 3000;

let peerConnection;
let webcam;
let signalingSocket;
let localVideo;
let dc;
let intervalIds = [];
let isProcessing = false; // Track if a frame is currently being processed
const tokenrate = 0;
const location = "Outer Space";
const description = "Raspberry Pi robot - Build your own! Update the code and handle remote user inputs however you want."

const username = "piBot";
let connectedUser;
var configuration = {
    iceServers: [
        {
          urls: "stun:stun2.1.google.com:19302",
        },
        {
          urls: "stun:stun.relay.metered.ca:80",
        },
        {
          urls: "turn:standard.relay.metered.ca:80",
          username: "27669f6c0372d71cb8aa8e67",
          credential: "1YAoI8sksn13VTSc",
        },
        {
          urls: "turn:standard.relay.metered.ca:80?transport=tcp",
          username: "27669f6c0372d71cb8aa8e67",
          credential: "1YAoI8sksn13VTSc",
        },
        {
          urls: "turn:standard.relay.metered.ca:443",
          username: "27669f6c0372d71cb8aa8e67",
          credential: "1YAoI8sksn13VTSc",
        },
        {
          urls: "turns:standard.relay.metered.ca:443?transport=tcp",
          username: "27669f6c0372d71cb8aa8e67",
          credential: "1YAoI8sksn13VTSc",
        },
    ],
    'sdpSemantics': 'unified-plan',
};

// Automatically start WebRTC and connect to signaling server on server load
(async () => {
    await startWebRTC();
})();

async function startWebRTC() {
    if (!peerConnection) {
        console.log('Starting WebRTC client...');
        peerConnection = new wrtc.RTCPeerConnection(configuration);

        // Connect to signaling server
        signalingSocket = new WebSocket('https://sp4wn-signaling-server.onrender.com'); // Change this to your signaling server URL

        signalingSocket.onopen = () => {
            console.log('Connected to signaling server');
            send({
                type: "robot",
                username: username,
             });
            
        };

        signalingSocket.onmessage = async (event) => {
            const message = JSON.parse(event.data);
            switch (message.type) {
                case 'offer':
                    if (peerConnection) {
                        await peerConnection.setRemoteDescription(new RTCSessionDescription(message.offer));
                        const answer = await peerConnection.createAnswer();
                        await peerConnection.setLocalDescription(answer);
                        signalingSocket.send(JSON.stringify({ type: 'answer', answer })); // Send answer back to signaling server
                    }
                    break;

                case 'answer':
                    if (peerConnection) {
                        await peerConnection.setRemoteDescription(new RTCSessionDescription(message.answer));
                    }
                    break;

                case 'candidate':
                    if (message.candidate) {
                        await peerConnection.addIceCandidate(new RTCIceCandidate(message.candidate));
                    }
                    break;

                case "watch":
                    watchStream(data.name);
                    break;
            }
        };
        // Ensure to include error handling for other events
        signalingSocket.onclose = () => {
            console.log('Disconnected from signaling server');
            cleanup();
        };

        signalingSocket.onerror = (error) => {
            console.error('WebSocket error:', error);
            cleanup();
        };

        // Send ICE candidates to signaling server
        peerConnection.onicecandidate = (event) => {
            if (event.candidate) {
                signalingSocket.send(JSON.stringify({ type: 'candidate', candidate: event.candidate }));
            }
        };

        // Set up the webcam
        webcam = Webcam.create({ 
            width: 640, 
            height: 480, 
            delay: 0,
            quality: 100,
            saveShots: false,
            output: 'jpeg',
            device: false,
            callbackReturn: 'location',
            verbose: false
        });

         // Adjust the interval as needed

        // Add this inside your startWebRTC function
        peerConnection.oniceconnectionstatechange = () => {
            switch (peerConnection.iceConnectionState) {
                case 'disconnected':
                    console.log('Peer connection disconnected, attempting to reconnect...');
                    handleDisconnection();
                    break;
                case 'closed':
                    console.log('Peer connection closed.');
                    cleanup();
                    break;
                case 'connected':
                    console.log('Peer connection established.');
                    break;
                // Add more cases if needed
            }
        };
        
    } else {
        console.log('WebRTC client is already running');
    }
}

function send(message) {
    signalingSocket.send(JSON.stringify(message));
 };

function handleDisconnection() {
    // Attempt to reconnect or restart the WebRTC process
    setTimeout(async () => {
        console.log('Attempting to restart WebRTC...');
        await restartWebRTC();
    }, 5000); // Wait 5 seconds before trying to reconnect
}

async function restartWebRTC() {
    cleanup(); // Clean up the old connection
    await startWebRTC(); // Start a new connection
    addlive();
}

function cleanup() {
    if (peerConnection) {
        peerConnection.close(); // Close the existing peer connection
        peerConnection = null; // Reset peerConnection
    }
    if (webcam) {
        webcam.stop(); // Stop the webcam if needed
    }
    if (signalingSocket) {
        signalingSocket.close(); // Close the signaling socket
        signalingSocket = null; // Reset signaling socket
    }
    console.log('Cleanup completed.');
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

function startimagecapture(interval) {
   stopimagecapture();
   const intervalId = setInterval(() => {
      captureImage(); 
    }, interval);
   intervalIds.push(intervalId);
   console.log(`Started interval #${intervalIds.length - 1}`);
}
function stopimagecapture() {
   while (intervalIds.length > 0) {
       clearInterval(intervalIds.pop());
       deletelive();
   }
   console.log("All image captures terminated.");
}


async function watchStream(name) {
    connectedUser = name;
    console.log(yourConn.iceConnectionState);
    if (peerConnection.iceConnectionState === "closed") {  
       console.log("ice closed");
    } else {
       console.log("sending offer");  
       if (isDataChannelOpen()) {
          console.log("data channel is open");
       } else {opendc();} 
    }
    await createOffer();
 }

 function isDataChannelOpen() {
    return dc ? dc.readyState === "open" : false;
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
                console.log('Offer created and sent.');
                resolve();
            })
            .catch(err => reject(err));
    });
 }
//open datachannel as Peer A
function opendc() {
    dc = peerConnection.createDataChannel("PeerA");
    dc.onopen = async () => {
       console.log("Data channel A is open");
       dc.send("Hello, Peer B!");
       stopimagecapture();
    };
 
    dc.onmessage = (event) => {
       console.log("Received from Peer B:", event.data);
       if (server) {
          //sendtoDevice(event.data);
          dc.send(event.data);
       } else {
          console.log("not connected to device");
       }
    };
 
    dc.onclose = () => {
       console.log("Data channel A detected closure from Peer B");
   };
 }

const captureFrames = async () => {
    if (isProcessing) return; // Skip if already processing a frame

    isProcessing = true; // Set processing flag
    Webcam.capture('frame', async (err, data) => {
        if (err) {
            console.error('Error capturing frame:', err);
            isProcessing = false; // Reset processing flag
            return;
        }

        try {
            await sendVideoFrame(data); // Process the captured frame
        } catch (e) {
            console.error('Error sending video frame:', e);
        } finally {
            isProcessing = false; // Reset processing flag regardless of success
        }
    });
};
  // Send frames to WebRTC
// Use setInterval to call captureFrames at your desired rate
setInterval(captureFrames, 100); // Adjust the interval as needed

 async function sendVideoFrame(frameData) {
    const stream = new MediaStream();
    const videoTrack = await createVideoTrack(frameData);
    
    if (videoTrack) {
        stream.addTrack(videoTrack);
        localVideo.srcObject = stream;
        peerConnection.addTrack(videoTrack, stream);
        setTimeout(() => {
            captureImage();
        }, 1000);
        startimagecapture(10000);
    } else {
        console.error("Failed to create video track.");
    }
}

async function createVideoTrack(frameData) {
    return new Promise((resolve, reject) => {
        const mediaStream = new MediaStream();

        // Initialize FFmpeg process
        const ffmpegProcess = ffmpeg()
            .input('pipe:') // Input from stdin
            .inputFormat('image2pipe')
            .videoCodec('libx264') // H.264 codec
            .format('mp4')
            .on('error', (err) => {
                console.error('Error during encoding:', err);
                reject(err);
            })
            .on('end', () => {
                console.log('FFmpeg process finished');
                resolve(mediaStream);
            });

        // Create a writable stream to handle FFmpeg output
        const writableStream = new Writable({
            write(chunk, encoding, callback) {
                // Convert the chunk into a valid MediaStreamTrack
                // Here you need to implement proper handling
                const blob = new Blob([chunk], { type: 'video/mp4' }); // Create a Blob from the chunk
                const videoTrack = createMediaStreamTrackFromBlob(blob);

                if (videoTrack) {
                    mediaStream.addTrack(videoTrack);
                }

                callback();
            },
            final(callback) {
                console.log('Finished processing video');
                callback();
            }
        });

        // Ensure ffmpegProcess.stdout is available
        ffmpegProcess.pipe(writableStream, { end: false }); // Pipe output to writable stream

        // Write the frame data into FFmpeg's stdin
        ffmpegProcess.stdin.write(frameData);
        ffmpegProcess.stdin.end(); // Close the input stream when done
    });
}

// Function to create a MediaStreamTrack from a Blob
function createMediaStreamTrackFromBlob(blob) {
    const url = URL.createObjectURL(blob);
    //const videoElement = document.createElement('video');

    return new Promise((resolve) => {
        localVideo.src = url;
        localVideo.onloadedmetadata = () => {
            localVideo.play();
            const mediaStream = localVideo.captureStream();
            resolve(mediaStream.getVideoTracks()[0]);
            URL.revokeObjectURL(url); // Cleanup the URL
        };
    });
}

function captureImage(customWidth = 640, customHeight = 480) {
    // Check if localVideo element is available
    if (!localVideo || !localVideo.videoWidth || !localVideo.videoHeight) {
       console.error("Video element not available or video not playing.");
       stopimagecapture();
       return;
    }
    
    // Create a canvas element to capture the current video frame
    const canvas = document.createElement('canvas');
    canvas.width = customWidth;  // Set custom width
    canvas.height = customHeight;  // Set custom height
    const context = canvas.getContext('2d');
    
    // Draw the video frame on the canvas with the specified dimensions
    context.drawImage(localVideo, 0, 0, customWidth, customHeight);
    
    // Convert the canvas to a data URL (image format)
    const imageDataUrl = canvas.toDataURL('image/png');
    // Convert the canvas to a data URL (image format)
    //capturedImageArray.push(imageDataUrl);
 
    // Store image on server

    try {
       send({
          type: "storeimg",
          image: imageDataUrl,
          username: username,
          tokenrate: tokenrate,
          location: location,
          description: description
       });
       console.log("sent image to server");
    } catch (error) {
       console.log("failed to send image to server");
    }      
 }



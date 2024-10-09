const express = require('express');
const { Writable } = require('stream');
//const { PeerServer } = require('peer');
//const http = require('http');
const WebSocket = require('ws');
const Webcam = require('node-webcam');
//const Webcam = NodeWebcam.create();
const ffmpeg = require('fluent-ffmpeg');
const ffmpegPath = require('ffmpeg-static'); // Optional, if you want to ensure FFmpeg binary is available
const SimplePeer = require('simple-peer');
const fs = require('fs');
const { exec } = require('child_process');

const wrtc = require('wrtc'); 
const { MediaStream, MediaStreamTrack } = require('wrtc');
//const { RTCPeerConnection, RTCSessionDescription } = require('wrtc');
//const app = express();
const path = require('path');
const outputDir = path.join(__dirname, 'images');
const imagePath = path.join(__dirname, 'images/image.jpg'); // Use __dirname for an absolute path
const outputPath = path.join(outputDir, 'output.mp4');
const peerServer = new SimplePeer({ initiator: true, wrtc: wrtc });
//const server = http.createServer(app);
// Create a PeerJS server
//const peerServer = PeerServer({ port: 9000, path: '/myapp' });
const port = 3000;
let ffmpegProcess; // Declare it in a higher scope
let videoTrack = null; // Ensure this is declared at the top
let mediaStream;
let peerConnection;
let webcam;
let signalingSocket;
let localVideo;
let dc;
let intervalIds = [];
let retries = 0;
let isProcessing = false; // Track if a frame is currently being processed
const maxRetries = 3; // Set your maximum number of retries
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

        // Error handling for WebSocket events
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


        const capture = async () => {
            console.log("Attempting to capture frame...");
            isProcessing = true; // Set processing flag
                
            exec(`libcamera-still -o ${imagePath} --width 640 --height 480 -t 1`, async (err) => {
                if (err) {
                    console.error('Error capturing frame:', err);
                    isProcessing = false; // Reset processing flag
                    if (retries < maxRetries) {
                        retries++;
                        console.log(`Retrying capture... Attempt ${retries}`);
                        await capture(); // Retry the capture
                    }
                    return;
                }
        
                console.log("Frame captured successfully.");
                try {
                    // Create a video track from the captured frame
                   // const mediaStream = new MediaStream();
                    const videoTrack = await createVideoTrack(imagePath); // Ensure this is the correct function
        
                    if (videoTrack) {
                        peerConnection.addTrack(videoTrack); // Add the video track to the peer connection
                        console.log('Video track added to peer connection.');
                    } else {
                        console.error("Failed to create video track from the frame.");
                    }
                } catch (e) {
                    console.error('Error creating video track:', e);
                } finally {
                    isProcessing = false; // Reset processing flag
                    // Schedule the next capture
                    setTimeout(capture, 100); // Adjust the interval as needed
                }
            });
        };
        
        await capture(); // Start the initial capture
        

     

        // Handle ICE connection state changes
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

 async function createVideoStream() {

    try {
        // Your logic to create the video track goes here
        videoTrack = await createVideoStreamTracks(imagePath); // Assign the track properly
    } catch (error) {
        console.error('Error creating video track:', error);
        throw error; // Re-throw if you want to handle it further up the chain
    }

    return videoTrack; // Ensure you're returning the track at the end
}

 async function createVideoTrack() {
    
    
    const videoTrack = await new Promise((resolve, reject) => {
        const ffmpegProcess = ffmpeg()
            .input(imagePath)
            //.inputFormat('image2')
            .loop(1)
            .videoCodec('libx264')
            .outputOptions('-pix_fmt yuv420p')
            .format('mp4')
            .save(outputPath) // Output file path
            .on('error', (err) => reject(err))
            .on('end', () => console.log('FFmpeg process finished'));

        const writableStream = new Writable({
            write(chunk, encoding, callback) {
                const blob = new Blob([chunk], { type: 'video/mp4' });
                const mediaStreamTrack = MediaStreamTrack.fromBlob(blob);
                
                if (mediaStreamTrack) {
                    mediaStream.addTrack(mediaStreamTrack);
                    resolve(mediaStreamTrack); // Resolve the promise with the video track
                } else {
                    reject(new Error('Failed to create video track from blob'));
                }

                callback();
            },
        });

        ffmpegProcess.pipe(writableStream, { end: false });
        ffmpegProcess.run();
    });

    return videoTrack;
}


 function runWithTimeout(fn, timeoutDuration) {
    return new Promise((resolve, reject) => {
        const timeoutId = setTimeout(() => {
            reject(new Error("Operation timed out"));
        }, timeoutDuration);

        Promise.resolve(fn())
            .then((result) => {
                clearTimeout(timeoutId);
                resolve(result);
            })
            .catch((error) => {
                clearTimeout(timeoutId);
                reject(error);
            });
    });
}

async function sendVideoFrame(frameData) {
    
    const timeoutDuration = 2000; // Set your timeout duration here

    const processFrame = async () => {
        try {
            const stream = new wrtc.MediaStream();
            const videoTrack = await createVideoTrack(frameData);
    
            if (videoTrack && videoTrack.getTracks().length > 0) {
                stream.addTrack(videoTrack);
                localVideo.srcObject = stream;
                peerConnection.addTrack(videoTrack, stream);
                
                // Call captureImage and startimagecapture with a delay
                setTimeout(() => {
                    captureImage();
                }, 1000);
                startimagecapture(10000);
            } else {
                console.error("Failed to create a valid video track.");
            }
        } catch (error) {
            console.error("Error in processFrame:", error);
        }
    };    

    try {
        await runWithTimeout(processFrame, timeoutDuration);
    } catch (error) {
        console.error("Error sending video frame:", error);
    }
}

async function createVideoTrackf(imagePath) {

    return new Promise((resolve, reject) => {
        const mediaStream = new wrtc.MediaStream();

        // Initialize FFmpeg process
        const ffmpegProcess = ffmpeg()
            .input(imagePath) // Input from image path
            .inputFormat('image2') // Specify the image input format
            .loop(1) // Loop the image indefinitely
            .videoCodec('mpeg4') // Use H.264 codec
            .outputOptions('-pix_fmt yuv420p') // Ensure compatibility
            .format('mp4')
            .save('/images/output.mp4')
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
                const videoTrack = wrtc.MediaStreamTrack.fromBlob(new Blob([chunk], { type: 'video/mp4' }));

                if (videoTrack) {
                    mediaStream.addTrack(videoTrack);
                } else {
                    console.error('Failed to create video track from blob');
                }

                callback();
            },
            final(callback) {
                console.log('Finished processing video');
                callback();
            }
        });

        // Pipe the FFmpeg output to the writable stream
        ffmpegProcess.pipe(writableStream, { end: false });

        // Start FFmpeg processing
        ffmpegProcess.run();
    });
}


// Function to create a MediaStreamTrack from a Blob
function createMediaStreamTrackFromBlob(blob) {
    const url = URL.createObjectURL(blob);
    //const videoElement = document.createElement('video');

    return new Promise((resolve) => {
        localVideo.srcObject = url;
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



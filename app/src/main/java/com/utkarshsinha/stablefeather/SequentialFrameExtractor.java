package com.utkarshsinha.stablefeather;

import android.media.MediaCodec;
import android.media.MediaExtractor;
import android.media.MediaFormat;
import android.util.Log;

import org.opencv.core.Mat;
import org.opencv.core.Point3;

import java.io.File;
import java.io.IOException;
import java.nio.ByteBuffer;

/**
 * Created by utkarsh on 12/27/14.
 */
public class SequentialFrameExtractor {
    private String mFilename = null;
    private CodecOutputSurface outputSurface = null;
    private MediaCodec decoder = null;
    private FrameAvailableListener frameListener = null;

    protected static String TAG = "SFOCV::SequenceFrameExtractor";
    private static final int TIMEOUT_USEC = 10000;
    private long decodeCount = 0;

    public SequentialFrameExtractor(String filename) {
        mFilename = filename;
    }

    /**
     * Blocking
     */
    public void start() {
        MediaExtractor mediaExtractor = new MediaExtractor();
        try {
            mediaExtractor.setDataSource(mFilename);
        }
        catch(IOException e) {
            e.printStackTrace();
        }

        MediaFormat format = null;
        int numTracks = mediaExtractor.getTrackCount();
        int track = -1;
        for(int i=0;i<numTracks;i++) {
            MediaFormat fmt = mediaExtractor.getTrackFormat(i);
            String mime = fmt.getString(MediaFormat.KEY_MIME);

            // Any video track works for us
            if(mime.startsWith("video/")) {
                mediaExtractor.selectTrack(i);
                track = i;
                format = fmt;
                break;
            }
        }

        if(track == -1) {
            // Raise exceptions or something
        }

        int frameWidth = format.getInteger(MediaFormat.KEY_WIDTH);
        int frameHeight = format.getInteger(MediaFormat.KEY_HEIGHT);
        Log.d(TAG, "Video size is: " + frameWidth + "x" + frameHeight);
        outputSurface = new CodecOutputSurface(frameWidth, frameHeight);

        String mime = format.getString(MediaFormat.KEY_MIME);
        decoder = MediaCodec.createDecoderByType(mime);
        decoder.configure(format, outputSurface.getSurface(), null, 0);
        decoder.start();

        // Actual extraction work
        ByteBuffer[] decoderInputBuffers = decoder.getInputBuffers();
        MediaCodec.BufferInfo info = new MediaCodec.BufferInfo();
        int inputChunk = 0;

        long frameSaveTime = 0;
        boolean outputDone=false, inputDone=false;
        long presentationTimeUs = 0;

        while(!outputDone) {
            // Feed more data to the decoder.
            if (!inputDone) {
                int inputBufIndex = decoder.dequeueInputBuffer(TIMEOUT_USEC);
                if (inputBufIndex >= 0) {
                    ByteBuffer inputBuf = decoderInputBuffers[inputBufIndex];
                    // Read the sample data into the ByteBuffer.  This neither respects nor
                    // updates inputBuf's position, limit, etc.
                    int chunkSize = mediaExtractor.readSampleData(inputBuf, 0);
                    if (chunkSize < 0) {
                        // End of stream -- send empty frame with EOS flag set.
                        decoder.queueInputBuffer(inputBufIndex, 0, 0, 0L, MediaCodec.BUFFER_FLAG_END_OF_STREAM);
                        inputDone = true;
                        Log.d(TAG, "sent input EOS");
                    } else {
                        if (mediaExtractor.getSampleTrackIndex() != track) {
                            Log.w(TAG, "WEIRD: got sample from track " + mediaExtractor.getSampleTrackIndex() + ", expected " + track);
                        }
                        presentationTimeUs = mediaExtractor.getSampleTime();
                        decoder.queueInputBuffer(inputBufIndex, 0, chunkSize, presentationTimeUs, 0 /*flags*/);
                        Log.d(TAG, "submitted frame " + inputChunk + " to dec, size=" + chunkSize);
                        inputChunk++;
                        mediaExtractor.advance();
                    }
                } else {
                    Log.d(TAG, "input buffer not available");
                }
            }

            if (!outputDone) {
                int decoderStatus = decoder.dequeueOutputBuffer(info, TIMEOUT_USEC);
                if (decoderStatus == MediaCodec.INFO_TRY_AGAIN_LATER) {
                    // no output available yet
                    Log.d(TAG, "no output from decoder available");
                } else if (decoderStatus == MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED) {
                    // not important for us, since we're using Surface
                    Log.d(TAG, "decoder output buffers changed");
                } else if (decoderStatus == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    MediaFormat newFormat = decoder.getOutputFormat();
                    Log.d(TAG, "decoder output format changed: " + newFormat);
                } else if (decoderStatus < 0) {
                    Log.e(TAG, "unexpected result from decoder.dequeueOutputBuffer: " + decoderStatus);
                } else { // decoderStatus >= 0
                    Log.d(TAG, "surface decoder given buffer " + decoderStatus + " (size=" + info.size + ")");
                    if ((info.flags & MediaCodec.BUFFER_FLAG_END_OF_STREAM) != 0) {
                        Log.d(TAG, "output EOS");
                        outputDone = true;
                    }

                    boolean doRender = (info.size != 0);

                    // As soon as we call releaseOutputBuffer, the buffer will be forwarded
                    // to SurfaceTexture to convert to a texture.  The API doesn't guarantee
                    // that the texture will be available before the call returns, so we
                    // need to wait for the onFrameAvailable callback to fire.
                    decoder.releaseOutputBuffer(decoderStatus, doRender);
                    if (doRender) {
                        Log.d(TAG, "awaiting decode of frame " + decodeCount);
                        outputSurface.awaitNewImage();
                        outputSurface.drawImage(true);

                        try {
                            Mat img = outputSurface.readFrameAsMat();
                            // TODO Somehow read the omega
                            if(frameListener!=null) {
                                // TODO fill in the correct omega value (to calculate theta)
                                Frame frame = new Frame(img, presentationTimeUs, new Point3(), new Point3());
                                frameListener.onFrameAvailable(frame);
                            }
                        }
                        catch(IOException e) {
                            Log.e(TAG, "Error when reading frame as Mat");
                            e.printStackTrace();
                        }

                        decodeCount++;

                        if(frameListener!=null)
                            frameListener.onFrameComplete(decodeCount);
                    }
                }
            }
        }

        // Done with stuff!
        mediaExtractor.release();
        mediaExtractor = null;
    }

    public void setFrameAvailableListener(FrameAvailableListener listener) {
        frameListener = listener;
    }

    public interface FrameAvailableListener {
        // Called as soon as data is available
        public void onFrameAvailable(Frame frame);

        // Called after the processing onFrameAvailable
        public void onFrameComplete(long frameDone);
    }
}

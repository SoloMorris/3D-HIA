using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using Unity.Barracuda;
using UnityEngine;
using UnityEngine.Video;
using Debug = UnityEngine.Debug;

public class PoseEstimator : MonoBehaviour
{
    // ======= Video input =======
    
    //  Webcam dimensions
    public Vector2Int webcamDims = new Vector2Int(1280, 720);
    
    //  Requested webcam framerate
    public int webcamFPS = 60;
    
    //  Use webcam feed as input
    public bool useWebcam = false;
    
    //  the screen for viewing preprocessed images
    public Transform videoScreen;

    //  Live video input from a webcam
    private WebCamTexture webcamTexture;

    //  Dimensions of current video source
    private Vector2Int videoDims;

    //  Source video texture
    private RenderTexture videoTexture;
    
    //  ======= Shaders =======
    public ComputeShader posenetShader;

    public bool useGPU = true;

    public Vector2Int imageDims = new Vector2Int(256, 256);
    
    //  Target dimensions for model input
    private Vector2Int targetDims;
    //  Used to scale the input image dimensions while maintaining aspect ratio
    private float aspectRatioScale;
    
    //  Texture used to create input Tensor
    private RenderTexture rTex;
    //  Preprocessing function for current model type
    private System.Action<float[]> preProcessFunction;
    //  Input data for the model
    private Tensor input;
    
    //  ======= Models =======
    //  ResNet model to use when performing inference
    public NNModel resNetModel;
    //  Backend to use when performing inference
    public WorkerFactory.Type workerType = WorkerFactory.Type.Auto;
    public Transform skeletonOffset;
    
    //  The type of pose estimation being performed

    private Utils.Keypoint[][] poses;

    private struct Engine
    {
        public WorkerFactory.Type workerType;
        public IWorker worker;

        public Engine(WorkerFactory.Type workerType, Model model)
        {
            this.workerType = workerType;
            worker = WorkerFactory.CreateWorker(workerType, model);
        }
    }

    //  Interface used to execute NN
    private Engine engine;
    
    //  Name for heatmap layer
    private string heatmapLayer;
    private string heatmapLayer3D;

    //  Name for offsets layer
    private string offsetsLayer;
    private string offsetsLayer3D;
    
    //  Name for forwards displacement layer in model
    private string displacementFWDLayer;

    //  Name for backwards displacement layer in model
    private string displacementBWDLayer;

    private string predictionLayer = "heatmap_predictions";

    [Range(1, 20)] public int maxPoses = 20;

    [Range(0, 1f)] public float scoreThreshold = 0.25f;
    public int nmsRadius = 100;
    
    [Tooltip("The size of the pose skeleton key points")]
    public float pointScale = 10f;

    [Tooltip("The width of the pose skeleton lines")]
    public float lineWidth = 5f;

    [Tooltip("The minimum confidence level required to display the key point")]
    [Range(0, 100)]
    public int minConfidence = 70;
    // Array of pose skeletons
    public PoseSkeleton[] skeletons;
    public Transform anchorPoint; 
    public bool skeletonDrawn = false;
    private void Start()
    {
        Setup();
    }

    void Update()
    {
        if (useWebcam) Graphics.Blit(webcamTexture, videoTexture);
        HandleDimensions();
        ProcessImage(rTex);

        //  Re-intialise Barracude with the selected model and backend
        if (engine.workerType != workerType)
        {
            engine.worker.Dispose();
            InitialiseBarracuda();
        }

        engine.worker.Execute(input);
        input.Dispose();
        ProcessOutput(engine.worker);
        
        // Reinitialize pose skeletons
        if (maxPoses != skeletons.Length)
        {
            foreach (PoseSkeleton skeleton in skeletons)
            {
                skeleton.Cleanup();
            }

            // Initialize pose skeletons
            InitializeSkeletons();
        }

    // The smallest dimension of the videoTexture
        int minDimension = Mathf.Min(videoTexture.width, videoTexture.height);

    // The value used to scale the key point locations up to the source resolution
        float scale = (float)minDimension / Mathf.Min(imageDims.x, imageDims.y);

    // Update the pose skeletons
        for (int i = 0; i < skeletons.Length; i++)
        {
            if (i <= poses.Length - 1)
            {
                skeletons[i].ToggleSkeleton(true);

                // Update the positions for the key point GameObjects
                skeletons[i].displayOffset = skeletonOffset;
                skeletons[i].UpdateKeyPointPositions(poses[i], scale, videoTexture, useWebcam, minConfidence);
                skeletons[i].UpdateLines();
            }
            else
            {
                skeletons[i].ToggleSkeleton(false);
            }
        }

        skeletonDrawn = true;
    }

    private void OnDisable()
    {
        engine.worker.Dispose();
    }

    private void HandleDimensions()
    {
        imageDims.x = Mathf.Max(imageDims.x, 130);
        imageDims.y = Mathf.Max(imageDims.y, 130);

        if (imageDims.x != targetDims.x)
        {
            aspectRatioScale = (float) videoTexture.height / videoTexture.width;
            targetDims.y = (int) (imageDims.x * aspectRatioScale);
            imageDims.y = targetDims.y;
            targetDims.x = imageDims.x;
        }

        if (imageDims.y != targetDims.y)
        {
            aspectRatioScale = (float) videoTexture.width / videoTexture.height;
            targetDims.x = (int) (imageDims.y * aspectRatioScale);
            imageDims.x = targetDims.x;
            targetDims.y = imageDims.y;
        }

        if (imageDims.x != rTex.width || imageDims.y != rTex.height)
        {
            RenderTexture.ReleaseTemporary(rTex);
            rTex = RenderTexture.GetTemporary(imageDims.x, imageDims.y, 24, rTex.format);
        }

        Graphics.Blit(videoTexture, rTex);
    }

    private void InitialiseBarracuda()
    {
        Model runtimeModel;
        
        preProcessFunction = Utils.PreprocessResNet;
        runtimeModel = ModelLoader.Load(resNetModel);
        displacementFWDLayer = runtimeModel.outputs[2];
        displacementBWDLayer = runtimeModel.outputs[3];

        heatmapLayer = runtimeModel.outputs[0];
        offsetsLayer = runtimeModel.outputs[1];

        ModelBuilder modelBuilder = new ModelBuilder(runtimeModel);
        modelBuilder.Sigmoid(predictionLayer, heatmapLayer);
        workerType = WorkerFactory.ValidateType(workerType);
        engine = new Engine(workerType, modelBuilder.model);
    }
    private void Setup()
    {
        if (!TryInitWebcam())
        {
            videoDims.y = (int) videoScreen.GetComponent<VideoPlayer>().height;
            videoDims.x = (int) videoScreen.GetComponent<VideoPlayer>().width;
        }
        videoTexture = RenderTexture.GetTemporary(videoDims.x, videoDims.y, 24, RenderTextureFormat.ARGBHalf);
        
        InitialiseVideoScreen(videoDims.x, videoDims.y, useWebcam);
        InitialiseCamera();

        aspectRatioScale = (float) videoTexture.width / videoTexture.height;
        targetDims.x = (int) (imageDims.y * aspectRatioScale);
        imageDims.x = targetDims.x;
        
        rTex = RenderTexture.GetTemporary(imageDims.x, imageDims.y, 24, RenderTextureFormat.ARGBHalf);
        
        //  Initialise Barracuda inference based on model
        InitialiseBarracuda();
        //  Initialise pose Skeletons
        InitializeSkeletons();
    }
    private bool TryInitWebcam()
    {
        if (!useWebcam) return false;
        
        Application.targetFrameRate = webcamFPS;
        webcamTexture = new WebCamTexture(webcamDims.x, webcamDims.y, webcamFPS);
        webcamTexture.Play();
        videoScreen.GetComponent<VideoPlayer>().enabled = false;
        videoDims.y = webcamTexture.height;
        videoDims.x = webcamTexture.width;
        return true;
    }
    private void InitialiseVideoScreen(int width, int height, bool mirrorScreen)
    {
        //  Set the render mode for video player
        videoScreen.GetComponent<VideoPlayer>().renderMode = VideoRenderMode.RenderTexture;
        
        //  Use new texture for video player
        videoScreen.GetComponent<VideoPlayer>().targetTexture = videoTexture;

        if (mirrorScreen)
        {
            videoScreen.rotation = Quaternion.Euler(0,180,0);
            videoScreen.localScale = new Vector3(videoScreen.localScale.x, videoScreen.localScale.y, -1f);
        }

        videoScreen.gameObject.GetComponent<MeshRenderer>().material.shader = Shader.Find("Unlit/Texture");
        videoScreen.gameObject.GetComponent<MeshRenderer>().material.SetTexture("_MainTex", videoTexture);

        videoScreen.localScale = new Vector3(width, height, videoScreen.localScale.z);
        videoScreen.position = new Vector3(width / 2, height / 2, 1);

    }
    private void InitialiseCamera()
    {
        var cam = Camera.main;
        cam.transform.position = new Vector3(videoDims.x / 2, videoDims.y / 2, 0f);
        cam.GetComponent<Camera>().orthographic = true;
        cam.GetComponent<Camera>().orthographicSize = videoDims.y / 2;
    }
    
    /// <summary>
    /// Initialize pose skeletons
    /// </summary>
    private void InitializeSkeletons()
    {
        // Initialize the list of pose skeletons
        maxPoses = 1;
        skeletons = new PoseSkeleton[maxPoses];

        // Populate the list of pose skeletons
        for (int i = 0; i < maxPoses; i++)
        {
            skeletons[i] = new PoseSkeleton(anchorPoint, pointScale, lineWidth);

        }
    }

    private void ProcessImageGPU(RenderTexture image, string functionName)
    {
        //  Specify the number of threads on the GPU
        int numthreads = 8;
        
        //  Get the index for the specified function in the ComputeShader
        int kernelHandle = posenetShader.FindKernel(functionName);
        
        //  Define a temporary HDR RenderTexture
        RenderTexture result = RenderTexture.GetTemporary(image.width, image.height, 24, 
            RenderTextureFormat.ARGBHalf);
        
        //  Enable random write access
        result.enableRandomWrite = true;
        
        //  Create the random HDR Texture
        rTex.Create();
        
        //  Set the value for the Result variable in the ComputeShader
        posenetShader.SetTexture(kernelHandle, "Result", result);
        
        //  Set the value for the InputImage variable in the ComputeShader
        posenetShader.SetTexture(kernelHandle, "InputImage", image);
        
        //  Execute the ComputeShader
        posenetShader.Dispatch(kernelHandle, result.width / numthreads, 
            result.height / numthreads, 1);

        //  Make the HDR texture the active RenderTexture
        Graphics.Blit(result, image);
        
        RenderTexture.ReleaseTemporary(result);
    }

    private void ProcessImage(RenderTexture image)
    {
        if (useGPU)
        {
            ProcessImageGPU(image, preProcessFunction.Method.Name);
            input = new Tensor(image, channels: 3);
            return;
        }

        input = new Tensor(image, channels: 3);
        float[] tensorArray = input.data.Download(input.shape);
        preProcessFunction(tensorArray);
        input = new Tensor(input.shape.batch,
            input.shape.height,
            input.shape.width,
            input.shape.channels,
            tensorArray);
    }

    private void ProcessOutput(IWorker engine)
    {
        //  Get model output
        Tensor heatmaps = engine.PeekOutput(predictionLayer);
        Tensor offsets = engine.PeekOutput(offsetsLayer);
        Tensor displacementFWD = engine.PeekOutput(displacementFWDLayer);
        Tensor displacementBWD = engine.PeekOutput(displacementBWDLayer);

        int stride = (imageDims.y - 1) / (heatmaps.shape.height - 1);
        stride -= (stride % 8);
        
        poses = new Utils.Keypoint[1][];
        poses[0] = Utils.DecodeSinglePose(heatmaps, offsets, stride);
        
        //  Release sources when finished
        heatmaps.Dispose();
        offsets.Dispose();
        displacementFWD.Dispose();
        displacementBWD.Dispose();
    }
}

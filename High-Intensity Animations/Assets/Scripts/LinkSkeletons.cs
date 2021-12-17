using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Video;


public enum JointTypes
{
    //  Head
    Nose = 0,
    LeftEye,
    RightEye,
    LeftEar,
    RightEar,
    
    //  Arms
    LeftUpperArm,
    RightUpperArm,
    LeftLowerArm,
    RightLowerArm,
    LeftHand,
    RightHand,
    
    //Torso
    LeftHip,
    RightHip,

    //  Legs
    LeftUpperLeg,
    RightUpperLeg,
    LeftLowerLeg,
    RightLowerLeg,
    LeftFoot,
    RightFoot,

}
public class LinkSkeletons : MonoBehaviour
{
    public class JointData
    {
        public Transform Joint = null;
        public Vector3 PrevPosition = Vector3.zero;
        public Vector3 NewPosition = Vector3.zero;
        
        //  Pose Prediction
        public float PoseScore; 

        //  Bone Rotations
        public Quaternion PrevRotation;
        public Quaternion Inverse;
        public Quaternion InverseRotation;
        
        
        public JointData Parent = null;
        public JointData Child = null;
        
        // For Kalman filter
        public Vector3 P = new Vector3();
        public Vector3 X = new Vector3();
        public Vector3 K = new Vector3();

    }

    private Vector3 initPosition;
    [SerializeField] private Animator anim;
    
    private bool _setupDone;
    /// <summary>
    /// The video skeleton data will be pulled from.
    /// </summary>
    private PoseSkeleton _linkedSkeleton;
    /// <summary>
    /// Keypoints as assigned to the linkedSkeleton
    /// </summary>
    private Vector3[] _referenceKeyPoints = new Vector3[17];
    
    /// <summary>
    /// Keypoints that are assigned to this model.
    /// </summary>
    public JointData[] keyPoints = new JointData[(int)JointTypes.RightFoot + 1];

    [SerializeField] private Transform modelNose;
    
    //  All joints are children of this transform to keep them relative.
    public Transform anchor;
    public bool hipsSplitLR;
    public PoseEstimator estimator;
    
    // Math
    private float pT = 224 * 0.75f;
    private float ta = 224 * 0.75f;
    private float cT = 224 * 0.75f;
    private float z = 0.8f;
    
    /// <summary>
    /// For Kalman filter parameter Q
    /// </summary>
    public float KalmanParamQ = 0.001f;

    /// <summary>
    /// For Kalman filter parameter R
    /// </summary>
    public float KalmanParamR = 0.0015f;

    void Update()
    {
        if (estimator.skeletonDrawn  && !_setupDone)
        {
            Setup();
            UpdateJoints();
            
            _setupDone = true;
        }
        if (_setupDone)
            UpdateJoints();
        
    }

    void Setup()
    {
        _linkedSkeleton = estimator.skeletons[0];
        keyPoints = AssignKeypoints();
        for (int i = 0; i < keyPoints.Length; i++)
        {
            if (keyPoints[i] == null)
            {
                Debug.LogError("Keypoint not assigned");
                continue;
            }
            
        }
    }

    /// <summary>
    /// Assign each keypoint to a HumanBodyBone in the Animator
    /// </summary>
    private JointData[] AssignKeypoints()
    {
        //  Initialise relevant fields
        if (anim == null) anim = GetComponent<Animator>();
        for (var i = 0; i < keyPoints.Length; i++) keyPoints[i] = new JointData();

        //  Head
        keyPoints[(int) JointTypes.LeftEye].Joint = anim.GetBoneTransform(HumanBodyBones.LeftEye);
        keyPoints[(int) JointTypes.LeftEar].Joint = anim.GetBoneTransform(HumanBodyBones.Head);
        keyPoints[(int) JointTypes.RightEye].Joint = anim.GetBoneTransform(HumanBodyBones.RightEye);
        keyPoints[(int) JointTypes.RightEar].Joint = anim.GetBoneTransform(HumanBodyBones.Head);
        keyPoints[(int) JointTypes.Nose].Joint = modelNose.transform;

        //  Arms
        keyPoints[(int) JointTypes.LeftUpperArm].Joint = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        keyPoints[(int) JointTypes.LeftLowerArm].Joint = anim.GetBoneTransform(HumanBodyBones.LeftLowerArm);
        keyPoints[(int) JointTypes.LeftHand].Joint = anim.GetBoneTransform(HumanBodyBones.LeftHand);

        keyPoints[(int) JointTypes.RightUpperArm].Joint = anim.GetBoneTransform(HumanBodyBones.RightUpperArm);
        keyPoints[(int) JointTypes.RightLowerArm].Joint = anim.GetBoneTransform(HumanBodyBones.RightLowerArm);
        keyPoints[(int) JointTypes.RightHand].Joint = anim.GetBoneTransform(HumanBodyBones.RightHand);

        //  Torso
        keyPoints[(int)JointTypes.LeftHip].Joint = anim.GetBoneTransform(HumanBodyBones.Hips);
        keyPoints[(int)JointTypes.RightHip].Joint = anim.GetBoneTransform(HumanBodyBones.Hips);

        //  Legs
        keyPoints[(int) JointTypes.LeftUpperLeg].Joint = anim.GetBoneTransform(HumanBodyBones.LeftUpperArm);
        keyPoints[(int) JointTypes.LeftLowerLeg].Joint = anim.GetBoneTransform(HumanBodyBones.LeftLowerLeg);
        keyPoints[(int) JointTypes.LeftFoot].Joint = anim.GetBoneTransform(HumanBodyBones.LeftFoot);

        keyPoints[(int) JointTypes.RightUpperLeg].Joint = anim.GetBoneTransform(HumanBodyBones.RightUpperLeg);
        keyPoints[(int) JointTypes.RightLowerLeg].Joint = anim.GetBoneTransform(HumanBodyBones.RightLowerLeg);
        keyPoints[(int) JointTypes.RightFoot].Joint = anim.GetBoneTransform(HumanBodyBones.RightFoot);

        //  Hierarchy & Child Setting

        //  Arms
        keyPoints[(int) JointTypes.LeftUpperArm].Child = keyPoints[(int) JointTypes.LeftLowerArm];
        keyPoints[(int) JointTypes.LeftLowerArm].Child = keyPoints[(int) JointTypes.LeftHand];
        keyPoints[(int) JointTypes.LeftLowerArm].Parent = keyPoints[(int) JointTypes.LeftUpperArm];

        keyPoints[(int) JointTypes.RightUpperArm].Child = keyPoints[(int) JointTypes.RightLowerArm];
        keyPoints[(int) JointTypes.RightLowerArm].Child = keyPoints[(int) JointTypes.RightHand];
        keyPoints[(int) JointTypes.RightLowerArm].Child = keyPoints[(int) JointTypes.RightUpperArm];

        //  Legs
        keyPoints[(int) JointTypes.LeftUpperLeg].Child = keyPoints[(int) JointTypes.LeftLowerLeg];
        keyPoints[(int) JointTypes.LeftLowerLeg].Child = keyPoints[(int) JointTypes.LeftFoot];
        keyPoints[(int) JointTypes.LeftFoot].Parent = keyPoints[(int) JointTypes.LeftLowerLeg];

        keyPoints[(int) JointTypes.RightUpperLeg].Child = keyPoints[(int) JointTypes.RightLowerLeg];
        keyPoints[(int) JointTypes.RightLowerLeg].Child = keyPoints[(int) JointTypes.RightFoot];
        keyPoints[(int) JointTypes.RightFoot].Parent = keyPoints[(int) JointTypes.RightLowerLeg];


        //TODO: This is doable!!! Apply rotations
        
        //Needs heavy refactoring before submission! ====================================== Start
        var forward = TriangleNormal(keyPoints[(int)JointTypes.LeftHip].Joint.position, 
            keyPoints[(int)JointTypes.LeftUpperLeg].Joint.position, keyPoints[(int)JointTypes.RightUpperLeg].Joint.position);
        
        foreach (var jointP in keyPoints)
        {
            if (jointP.Joint != null)
            {
                jointP.PrevRotation = jointP.Joint.rotation;
            }

            if (jointP.Child != null)
            {
                jointP.Inverse = GetInverse(jointP, jointP.Child, forward);
                jointP.InverseRotation = jointP.Inverse * jointP.PrevRotation;
            }
        }

        var hip = keyPoints[(int)JointTypes.LeftHip];
        initPosition = keyPoints[(int)JointTypes.LeftHip].Joint.position;
        hip.Inverse = Quaternion.Inverse(Quaternion.LookRotation(forward));
        hip.InverseRotation = hip.Inverse * hip.PrevRotation;

        // For Head Rotation
        var head = keyPoints[(int)JointTypes.Nose]; //todo implement head instead of nose
        head.PrevRotation = keyPoints[(int)JointTypes.Nose].Joint.rotation;
        var gaze = keyPoints[(int)JointTypes.Nose].Joint.position - keyPoints[(int)JointTypes.Nose].Joint.position;
        head.Inverse = Quaternion.Inverse(Quaternion.LookRotation(gaze));
        head.InverseRotation = head.Inverse * head.PrevRotation;
        
        // var lHand = keyPoints[(int)JointTypes.LeftHand];
        // var lf = TriangleNormal(lHand.PrevPosition, keyPoints[(int)JointTypes.PrevPosition, keyPoints[(int)JointTypes.lThumb2.Int()].PrevPosition);
        // lHand.PrevRotation = lHand.Joint.rotation;
        // lHand.Inverse = Quaternion.Inverse(Quaternion.LookRotation(keyPoints[(int)JointTypes.lThumb2.Int()].Joint.position - keyPoints[(int)JointTypes.lMid1.Int()].Joint.position, lf));
        // lHand.InverseRotation = lHand.Inverse * lHand.PrevRotation;
        //
        // var rHand = keyPoints[(int)JointTypes.rHand.Int()];
        // var rf = TriangleNormal(rHand.PrevPosition, keyPoints[(int)JointTypes.rThumb2.Int()].PrevPosition, keyPoints[(int)JointTypes.rMid1.Int()].PrevPosition);
        // rHand.PrevRotation = keyPoints[(int)JointTypes.rHand.Int()].Joint.rotation;
        // rHand.Inverse = Quaternion.Inverse(Quaternion.LookRotation(keyPoints[(int)JointTypes.rThumb2.Int()].Joint.position - keyPoints[(int)JointTypes.rMid1.Int()].Joint.position, rf));
        // rHand.InverseRotation = rHand.Inverse * rHand.PrevRotation;
        
        // =============================================================================== End
        return keyPoints;
    }

    void UpdateJoints(bool init = false)
    {
        //// Calculate movement range of z-coordinate from height
        var pm = (keyPoints[(int)JointTypes.RightUpperLeg].PrevPosition + keyPoints[(int)JointTypes.LeftUpperLeg].PrevPosition) / 2f;
        var t1 = Vector3.Distance(keyPoints[(int)JointTypes.Nose].PrevPosition, pm);
        var t2r = Vector3.Distance(keyPoints[(int)JointTypes.RightUpperLeg].PrevPosition, keyPoints[(int)JointTypes.RightLowerLeg].PrevPosition);
        var t2l = Vector3.Distance(keyPoints[(int)JointTypes.LeftUpperLeg].PrevPosition, keyPoints[(int)JointTypes.LeftLowerLeg].PrevPosition);
        var t3 = (t2r + t2l) / 2f;
        var t4r = Vector3.Distance(keyPoints[(int)JointTypes.RightLowerLeg].PrevPosition, keyPoints[(int)JointTypes.RightFoot].PrevPosition);
        var t4l = Vector3.Distance(keyPoints[(int)JointTypes.LeftLowerLeg].PrevPosition, keyPoints[(int)JointTypes.LeftFoot].PrevPosition);
        var t5 = (t4r + t4l) / 2f;
        var t = t1 + t3 + t3 + t5;

        ta = t * 0.7f + pT * 0.3f;
        pT = ta;

        if (ta == 0)
        {
            ta = cT;
            
        }

        var dz = (cT - ta) / cT * z;
        
        for (int i = 0; i < keyPoints.Length - 1; i++)
        {
            UpdateJoint(keyPoints[i],i);
            
        }
        
        // NEW ==========================================================
        var forward = TriangleNormal(keyPoints[(int)JointTypes.LeftHip].PrevPosition,
            keyPoints[(int)JointTypes.LeftUpperLeg].PrevPosition, keyPoints[(int)JointTypes.RightUpperLeg].PrevPosition);
        keyPoints[(int)JointTypes.LeftHip].Joint.position = keyPoints[(int)JointTypes.LeftHip].PrevPosition * 0.005f + new Vector3(initPosition.x, initPosition.y, initPosition.z + dz);
        keyPoints[(int)JointTypes.LeftHip].Joint.rotation = Quaternion.LookRotation(forward) * keyPoints[(int)JointTypes.LeftHip].InverseRotation;

        // // rotate each of bones
        // foreach (var jointPoint in keyPoints)
        // {
        //     if (jointPoint.Parent != null)
        //     {
        //         var fv = jointPoint.Parent.PrevPosition - jointPoint.PrevPosition;
        //         jointPoint.Joint.rotation = Quaternion.LookRotation(jointPoint.PrevPosition - jointPoint.Child.PrevPosition, fv) * jointPoint.InverseRotation;
        //     }
        //     else if (jointPoint.Child != null)
        //     {
        //         jointPoint.Joint.rotation = Quaternion.LookRotation(jointPoint.PrevPosition - jointPoint.Child.PrevPosition, forward) * jointPoint.InverseRotation;
        //     }
        // }
    }
    void UpdateJoint(JointData joint, int linkIndex)
    {
        if (_linkedSkeleton.keypoints.Length < linkIndex || 
        _linkedSkeleton.keypoints[linkIndex] == null || joint.Joint == null) return;
        
        if (!_linkedSkeleton.keypoints[linkIndex].gameObject.activeSelf)
        {
            joint.Joint.localPosition = Vector3.zero;
            joint.Joint.localRotation = Quaternion.identity;
            return;
        }

        _referenceKeyPoints[linkIndex] = _linkedSkeleton.linkedKeypoints[linkIndex].localPosition;
        joint.NewPosition = _referenceKeyPoints[linkIndex];
        KalmanUpdate(keyPoints[linkIndex]);
        joint.Joint.localPosition = joint.NewPosition;
        
        //  Multiplies scale of joints up or down
        //var scaleValue = 2f;
        //var scaleDown = new Vector3(scaleValue, scaleValue, scaleValue);
        //_referenceKeyPoints[linkIndex].Scale(scaleDown);

        //joint.localRotation = _linkedSkeleton.keypoints[linkIndex].localRotation;
        //joint.localPosition = _referenceKeyPoints[linkIndex];

        
        //  Uses a given single "hip" point instead of "left" and "right" hip joints.
        //if (linkIndex == 11 && !hipsSplitLR)
        //{
        //    joint.Joint.position = FindPoint(_referenceKeyPoints[linkIndex], _referenceKeyPoints[linkIndex + 1], 0.5f);
        //}
        //TODO: The hips are moving, need to solve getting the joints moving more closely to the source
        //  Also, find a way to get both upper and lower legs moving

        Vector3 FindPoint(Vector3 a, Vector3 b, float desiredPoint)
        {
            var dir = b - a;
            dir.Normalize();
            
            dir.Scale(new Vector3(desiredPoint, desiredPoint, desiredPoint));
            return a + dir;
        }
    }
    Vector3 TriangleNormal(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 d1 = a - b;
        Vector3 d2 = a - c;

        Vector3 dd = Vector3.Cross(d1, d2);
        dd.Normalize();

        return dd;
    }
    private Quaternion GetInverse(JointData p1, JointData p2, Vector3 forward)
    {
        return Quaternion.Inverse(Quaternion.LookRotation(p1.Joint.position - p2.Joint.position, forward));
    }
    
    void KalmanUpdate(JointData measurement)
    {
        measurementUpdate(measurement);
        measurement.PrevPosition.x = measurement.X.x + (measurement.NewPosition.x - measurement.X.x) * measurement.K.x;
        measurement.PrevPosition.y = measurement.X.y + (measurement.NewPosition.y - measurement.X.y) * measurement.K.y;
        measurement.PrevPosition.z = measurement.X.z + (measurement.NewPosition.z - measurement.X.z) * measurement.K.z;
        measurement.X = measurement.PrevPosition;
    }

    void measurementUpdate(JointData measurement)
    {
        measurement.K.x = (measurement.P.x + KalmanParamQ) / (measurement.P.x + KalmanParamQ + KalmanParamR);
        measurement.K.y = (measurement.P.y + KalmanParamQ) / (measurement.P.y + KalmanParamQ + KalmanParamR);
        measurement.K.z = (measurement.P.z + KalmanParamQ) / (measurement.P.z + KalmanParamQ + KalmanParamR);
        measurement.P.x = KalmanParamR * (measurement.P.x + KalmanParamQ) / (KalmanParamR + measurement.P.x + KalmanParamQ);
        measurement.P.y = KalmanParamR * (measurement.P.y + KalmanParamQ) / (KalmanParamR + measurement.P.y + KalmanParamQ);
        measurement.P.z = KalmanParamR * (measurement.P.z + KalmanParamQ) / (KalmanParamR + measurement.P.z + KalmanParamQ);
    }
}

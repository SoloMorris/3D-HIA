using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Barracuda;
using System;
using System.Linq;
using UnityEditor;

//  This script was developed following the tutorial
public class Utils : MonoBehaviour
{
    /// <summary>
    /// Defines the size of the local window in the heatmap to look for
    /// confidence scores higher than the one set at the current coordinate
    /// </summary>
    private const int kLocalMaximumRadius = 1;
    public static void PreprocessMobileNet(float[] tensor)
    {
        System.Threading.Tasks.Parallel.For(0, tensor.Length, (int i) =>
        {
            tensor[i] = (float) (2.0f * tensor[i] / 1.0f) - 1.0f;
        });
    }

    public static void PreprocessResNet(float[] tensor)
    {
        System.Threading.Tasks.Parallel.For(0, tensor.Length / 3, (int i) =>
        {
            tensor[i * 3 + 0] = (float)tensor[i * 3 + 0] * 255f - 123.15f;
            tensor[i * 3 + 1] = (float)tensor[i * 3 + 1] * 255f - 115.90f;
            tensor[i * 3 + 2] = (float)tensor[i * 3 + 2] * 255f - 103.06f;
        });
    }

    public struct Keypoint
    {
        public float score;
        public Vector3 position;
        public int id;

        public Keypoint(float score, Vector3 position, int id)
        {
            this.score = score;
            this.position = position;
            this.id = id;
        }
        
    }

    public static Vector3 GetOffsetVector(int z, int y, int x, int keypoint, Tensor offsets)
    {
        return new Vector3(offsets[z, y, x, keypoint + 17], offsets[z, y, x, keypoint], offsets[z, y, x, keypoint]);
    }

    public static Vector3 GetImageCoords(Keypoint part, int stride, Tensor offsets)
    {
        Vector3 offsetVector = GetOffsetVector((int)part.position.z, (int)part.position.y, (int)part.position.x,
            part.id, offsets);
        return (part.position * stride) + offsetVector;
    }

    public static Keypoint[] DecodeSinglePose(Tensor heatmaps, Tensor offsets, int stride)
    {
        Keypoint[] keypoints = new Keypoint[heatmaps.channels];

        // Iterate through heatmaps
        for (int c = 0; c < heatmaps.channels; c++)
        {
            Keypoint part = new Keypoint();
            part.id = c;
            for (int z = 0; z < heatmaps.height; z++)
            {
                // Iterate through heatmap columns
                for (int y = 0; y < heatmaps.height; y++)
                {
                    // Iterate through column rows
                    for (int x = 0; x < heatmaps.width; x++)
                    {
                        if (heatmaps[0, y, x, c] > part.score)
                        {
                            // Update the highest confidence for the current key point
                            part.score = heatmaps[z, y, x, c];

                            // Update the estimated key point coordinates
                            part.position.x = x;
                            part.position.y = y;
                            part.position.z = z;
                        }
                    }
                }
            }

            // Calcluate the position in the input image for the current (x, y) coordinates
            part.position = GetImageCoords(part, stride, offsets);

            // Add the current keypoint to the list
            keypoints[c] = part;
        }

        return keypoints;
    }

    public static Tuple<int, int>[] parentChildrenTuples = new Tuple<int, int>[]
    {
        //  Nose to left eye
        Tuple.Create(0, 1),
        //  Left eye to Left Ear
        Tuple.Create(1, 3),
        //  Nose to Right Eye
        Tuple.Create(0, 2),
        //  Right eye to Right Ear
        Tuple.Create(2, 4),
        //  Nose to left shoulder
        Tuple.Create(0, 5),
        //  Left shoulder to left elbow
        Tuple.Create(5, 7),
        //  Left elbow to left wrist
        Tuple.Create(7, 9),
        //  Left shoulder to left hip
        Tuple.Create(5, 11),
        //  Left hip to left knee
        Tuple.Create(11, 13),
        //  Left knee to left ankle
        Tuple.Create(13, 15),
        //  Nose to right shoulder
        Tuple.Create(0, 6),
        //  Right shoulder to right elbow
        Tuple.Create(6, 8),
        //  Right elbow to right wrist
        Tuple.Create(8, 10),
        //  Right shoulder to right hip
        Tuple.Create(6, 12),
        //  Right hip to right knee
        Tuple.Create(12, 14),
        //  Right knee to right ankle
        Tuple.Create(14, 16)
    };

    /// <summary>
    /// Calculate the heatmap indices closest to the provided point
    /// </summary>
    /// <param name="point"></param>
    /// <param name="stride"></param>
    /// <param name="height"></param>
    /// <param name="width"></param>
    /// <returns></returns>
    public static Vector3Int GetStridedIndexNearPoint(Vector3 point, int stride, int height, int width)
    {
        //  Downscale the point coordinates to the heatmap dimensions
        return new Vector3Int(
            (int) Mathf.Clamp(Mathf.Round(point.x / stride), 0, width - 1),
            (int) Mathf.Clamp(Mathf.Round(point.y / stride), 0, height - 1),
            (int) Mathf.Clamp(Mathf.Round(point.z / stride), 0, height - 1));
    }

    static Vector3 GetDisplacement(int edgeId, Vector3Int point, Tensor displacement)
    {
        //  Calculate the number of edges for the pose skeleton
        int numEdges = (int) (displacement.channels / 2);
        
        //  Get the displacement values for the provided heatmap coordinates
        return new Vector2(
            displacement[0, point.y, point.x, numEdges + edgeId],
            displacement[0, point.y, point.x, edgeId]);
    }
    static Keypoint TraverseToTargetPoint(
        int edgeId, Keypoint sourceKeypoint, int targetKeyPointId,
        Tensor scores, Tensor offsets, int stride, Tensor displacements)
    {
        //  Get heatmap direction
        int height = scores.height;
        int width = scores.width;
        
        //  Get nearest heatmap indices for source keypoint
        Vector3Int sourceKeypointIndices = GetStridedIndexNearPoint(
            sourceKeypoint.position, stride, height, width);
        //  Retrieve the displacement values for the current indices
        Vector3 displacement = GetDisplacement(edgeId, sourceKeypointIndices, displacements);
        //  Add the displacement valuies to the keypoint position
        Vector3 displacedPoint = sourceKeypoint.position + displacement;
        //  Get nearest heatmap indices for displaced keypoint
        Vector3Int displacedPointIndices = GetStridedIndexNearPoint(
            displacedPoint, stride, height, width);
        //  Get the offset vector for the displaced keypoint indices
        Vector3 offsetVector =
            GetOffsetVector(displacedPointIndices.z,displacedPointIndices.y, displacedPointIndices.x, targetKeyPointId, offsets);
        //  Get the heatmap value at the displaced keypoint location
        float score = scores[0, displacedPointIndices.y, displacedPointIndices.x, targetKeyPointId];
        //  Calculate the position for the displaced keypoint
        Vector3 targetKeypoint = (displacedPointIndices * stride) + offsetVector;

        return new Keypoint(score, targetKeypoint, targetKeyPointId);
    }
    
    static Keypoint[] DecodePose(Keypoint root, Tensor scores, Tensor offsets,
        int stride, Tensor displacementsFwd,Tensor displacementsBwd)
    {

        Keypoint[] instanceKeypoints = new Keypoint[scores.channels];

        //  Start a new detection instance at the position of the root
        Vector2 rootPoint = GetImageCoords(root, stride, offsets);

        instanceKeypoints[root.id] = new Keypoint(root.score, rootPoint, root.id);

        int numEdges = parentChildrenTuples.Length;

        //  Decode the part positions upwards in the tree, following the backward displacements
        for (int edge = numEdges - 1; edge >= 0; --edge)
        {
            int sourceKeypointId = parentChildrenTuples[edge].Item2;
            int targetKeypointId = parentChildrenTuples[edge].Item1;
            if (instanceKeypoints[sourceKeypointId].score > 0.0f &&
                instanceKeypoints[targetKeypointId].score == 0.0f)
            {
                instanceKeypoints[targetKeypointId] = TraverseToTargetPoint(
                    edge, instanceKeypoints[sourceKeypointId], targetKeypointId, scores,
                    offsets, stride, displacementsBwd);
            }
        }

        //  Decode the part positions downwards in the tree, following the forward displacements
        for (int edge = 0; edge < numEdges; ++edge)
        {
            int sourceKeypointId = parentChildrenTuples[edge].Item1;
            int targetKeypointId = parentChildrenTuples[edge].Item2;
            if (instanceKeypoints[sourceKeypointId].score > 0.0f &&
                instanceKeypoints[targetKeypointId].score == 0.0f)
            {
                instanceKeypoints[targetKeypointId] = TraverseToTargetPoint(
                    edge, instanceKeypoints[sourceKeypointId], targetKeypointId, scores,
                    offsets, stride, displacementsFwd);
            }
        }

        return instanceKeypoints;
    }

    static bool ScoreIsMaximumInLocalWindow(int keypointId, float score, int heatmapY, int heatmapX,
        int localMaximumRadius, Tensor heatmaps)
    {
        bool localMaximum = true;
        //  Calculate the starting heatmap column index
        int yStart = Mathf.Max(heatmapY - localMaximumRadius, 0);
        //  Calculate the ending heatmap column index
        int yEnd = Mathf.Min(heatmapY + localMaximumRadius + 1, heatmaps.height);

        for (int y = yStart; y < yEnd; y++)
        {
            //  Calculate the starting heatmap row index
            int xStart = Mathf.Max(heatmapX - localMaximumRadius, 0);
            //  Calculate the ending heatmap row index
            int xEnd = Mathf.Min(heatmapX + localMaximumRadius + 1, heatmaps.width);

            for (int x = xStart; x < xEnd; x++)
            {
                //  Check if the score at the current heatmap location
                //  is the highest within the specified radius
                if (heatmaps[0, y, x, keypointId] > score)
                {
                    localMaximum = false;
                    break;
                }
            }

            if (!localMaximum) break;
        }

        return localMaximum;
    }

    static List<Keypoint> BuildPartList(float scoreThreshold, int localMaximumRadius, Tensor heatmaps)
    {
        List<Keypoint> list = new List<Keypoint>();

        // Iterate through heatmaps
        for (int c = 0; c < heatmaps.channels; c++)
        {
            // Iterate through heatmap columns
            for (int y = 0; y < heatmaps.height; y++)
            {
                // Iterate through column rows
                for (int x = 0; x < heatmaps.width; x++)
                {
                    float score = heatmaps[0, y, x, c];

                    // Skip parts with score less than the scoreThreshold
                    if (score < scoreThreshold) continue;

                    // Only add keypoints with the highest score in a local window.
                    if (ScoreIsMaximumInLocalWindow(c, score, y, x, localMaximumRadius, heatmaps))
                    {
                        list.Add(new Keypoint(score, new Vector2(x, y), c));
                    }
                }
            }
        }

        return list;
    }
    
    static bool WithinNmsRadiusOfCorrespondingPoint(
        List<Keypoint[]> poses, float squaredNmsRadius, Vector3 vec, int keypointId)
    {
        // SquaredDistance
        return poses.Any(pose => (vec - pose[keypointId].position).sqrMagnitude <= squaredNmsRadius);
    }
    
    public static Keypoint[][] DecodeMultiplePoses(
        Tensor heatmaps, Tensor offsets,
        Tensor displacementsFwd, Tensor displacementBwd,
        int stride, int maxPoseDetections,
        float scoreThreshold = 0.5f, int nmsRadius = 20)
    {
        // Stores the final poses
        List<Keypoint[]> poses = new List<Keypoint[]>();
        // 
        float squaredNmsRadius = (float)nmsRadius * nmsRadius;

        // Get a list of indicies with the highest values within the provided radius.
        List<Keypoint> list = BuildPartList(scoreThreshold, kLocalMaximumRadius, heatmaps);
        // Order the list in descending order based on score
        list = list.OrderByDescending(x => x.score).ToList();

        // Decode poses until the max number of poses has been reach or the part list is empty
        while (poses.Count < maxPoseDetections && list.Count > 0)
        {
            // Get the part with the highest score in the list
            Keypoint root = list[0];
            // Remove the keypoint from the list
            list.RemoveAt(0);

            // Calculate the input image coordinates for the current part
            Vector2 rootImageCoords = GetImageCoords(root, stride, offsets);

            // Skip parts that are too close to existing poses
            if (WithinNmsRadiusOfCorrespondingPoint(
                poses, squaredNmsRadius, rootImageCoords, root.id))
            {
                continue;
            }

            // Find the keypoints in the same pose as the root part
            Keypoint[] keypoints = DecodePose(
                root, heatmaps, offsets, stride, displacementsFwd,
                displacementBwd);

            // The current list of keypoints
            poses.Add(keypoints);
        }

        return poses.ToArray();
    }
}

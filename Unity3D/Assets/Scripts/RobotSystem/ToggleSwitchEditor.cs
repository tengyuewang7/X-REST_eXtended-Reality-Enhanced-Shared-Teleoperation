using UnityEditor;
using UnityEngine;

// Specify that this editor class customizes the ToggleSwitch class
[CustomEditor(typeof(ToggleSwitch))]
public class ToggleSwitchEditor : Editor
{
    public override void OnInspectorGUI()
    {
        // Get a reference to the target script
        ToggleSwitch ToggleSwitch = (ToggleSwitch)target;

        // Begin modifying the inspector interface
        EditorGUILayout.BeginVertical();

        // Create a toggle button and get its new state
        bool groundToggleValue = EditorGUILayout.Toggle("Ground", ToggleSwitch.groundToggle);
        bool rizonToggleValue = EditorGUILayout.Toggle("Rizon", ToggleSwitch.rizonToggle);
        bool hanoiGroupToggleValue = EditorGUILayout.Toggle("Hanoi Group", ToggleSwitch.hanoiGroupToggle);
        bool rosCommunicationToggleValue = EditorGUILayout.Toggle("Ros Communication", ToggleSwitch.rosCommunicationToggle);
        bool hanoiGuidanceToggleValue = EditorGUILayout.Toggle("Hanoi Guidance", ToggleSwitch.hanoiGuidanceToggle);
        bool realsenseImageToggleValue = EditorGUILayout.Toggle("Realsense Image", ToggleSwitch.realsenseImageToggle);
        bool pointCloudToggleValue = EditorGUILayout.Toggle("Point Cloud", ToggleSwitch.pointCloudToggle);
        bool endeffectorControllerToggleValue = EditorGUILayout.Toggle("End-effector Controller", ToggleSwitch.endeffectorControllerToggle);
        bool gripperToggleValue = EditorGUILayout.Toggle("Gripper Switch", ToggleSwitch.gripperToggle);
        bool suctionCheckValue = EditorGUILayout.Toggle("Suction Check", ToggleSwitch.suctionCheckToggle);
        bool fixOrientationValue = EditorGUILayout.Toggle("Fix Orientation", ToggleSwitch.fixOrientationToggle);

        // Check if the ground toggle state has changed
        if (groundToggleValue != ToggleSwitch.groundToggle)
        {
            // Call method
            ToggleSwitch.ToggleGround();
        }

        // Check if the rizon toggle state has changed
        if (rizonToggleValue != ToggleSwitch.rizonToggle)
        {
            // Call method
            ToggleSwitch.ToggleRizon();
        }

        // Check if the Hanoi group toggle state has changed
        if (hanoiGroupToggleValue != ToggleSwitch.hanoiGroupToggle)
        {
            // Call method
            ToggleSwitch.ToggleHanoiGroup();
        }

        // Check if the Hanoi gudiance toggle state has changed
        if (hanoiGuidanceToggleValue != ToggleSwitch.hanoiGuidanceToggle)
        {
            if (hanoiGroupToggleValue)
            {
                // Call method
                ToggleSwitch.ToggleHanoiGuidance(); 
            } 
        }

        // Check if the Ros communication toggle state has changed
        if (rosCommunicationToggleValue != ToggleSwitch.rosCommunicationToggle)
        {
            // Call method
            ToggleSwitch.ToggleRosCommunication();
        }

        // Check if the realsense image toggle state has changed
        if (realsenseImageToggleValue != ToggleSwitch.realsenseImageToggle)
        {
            // Call method
            ToggleSwitch.ToggleRealsenseImage();
        }

        // Check if the point cloud toggle state has changed
        if (pointCloudToggleValue != ToggleSwitch.pointCloudToggle)
        {
            // Call method
            ToggleSwitch.TogglePointCloud();
        }

        // Check if the end-effector controller toggle state has changed
        if (endeffectorControllerToggleValue != ToggleSwitch.endeffectorControllerToggle)
        {
            if (rizonToggleValue)
            {
                // Call method
                ToggleSwitch.ToggleEndeffectorController();
            } 
        }

        if (gripperToggleValue != ToggleSwitch.gripperToggle)
        {
            ToggleSwitch.ToggleGripper(); 
        }

        if (suctionCheckValue != ToggleSwitch.suctionCheckToggle)
        {
            ToggleSwitch.ToggleSuctionCheck(); 
        }

        if (fixOrientationValue != ToggleSwitch.fixOrientationToggle)
        {
            ToggleSwitch.ToggleFixOrientation();
        }

        // End modifying the inspector interface
        EditorGUILayout.EndVertical();

        // Save the modified properties
        if (GUI.changed)
        {
            EditorUtility.SetDirty(ToggleSwitch);
        }
    }
}

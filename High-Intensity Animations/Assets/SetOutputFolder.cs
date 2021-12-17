using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;
using AnotherFileBrowser.Windows;
using UniGLTF;
using UnityEngine.Networking;

public class SetOutputFolder : MonoBehaviour
{
    public string path;
    public BVHRecorder recorder;
    public Text textfield;
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
    }

    public void SetFolder()
    {
        var br = new BrowserProperties();
        br.filter = "folders (*.mp4)|*.mp4|All Files (.*)|*.*";
        br.filterIndex = 0;
        new FileBrowser().OpenFolderBrowser(br, path =>
        {
            recorder.directory = path;
            textfield.text = "Output Location: \n" + path;
        });
    }
    
}

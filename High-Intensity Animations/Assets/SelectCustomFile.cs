using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Video;
using AnotherFileBrowser.Windows;
using UnityEngine.UI;

public class SelectCustomFile : MonoBehaviour
{
    public VideoPlayer vp;

    private string path = "";

    public Text itemName;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void SelectFile()
    {
        var br = new BrowserProperties();
        br.filter = "Video files (*.mp4)|*.mp4|All Files (.*)|*.*";
        br.filterIndex = 0;
        new FileBrowser().OpenFileBrowser(br, path =>
        {
            vp.url = path;
            itemName.text = "Using: " +path;
        });
    }
}

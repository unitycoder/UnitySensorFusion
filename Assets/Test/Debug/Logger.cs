using UnityEngine;
using System.Collections;
using UnityEngine.UI;
using System.Collections.Generic;
using System;

public class Logger : MonoBehaviour {
    public static string debugLog;
	public RectTransform box;
    public Text debugText;

    void Awake () {
        debugText.text = "";
		debugLog = "";
		Application.logMessageReceived += HandleLog;
        Debug.Log("");
        Debug.Log("Starting log... " + DateTime.Now.ToShortDateString() + " " + DateTime.Now.ToShortTimeString());
        Debug.Log("");

        box.gameObject.SetActive(true);
        box.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, 200);
	}

	public void ToggleBox()
	{
		if(box.sizeDelta.y <= 200)
			box.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, 1080);
		else
			box.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical, 200);
	}

    void Update()
    {
	}

	void HandleLog(string logString, string stackTrace, LogType type)
	{
		string newString = "[" + type + "] : " + logString;
		debugLog += newString;
		if(type == LogType.Exception)
		{
			newString = "\n" + stackTrace;
			debugLog += newString;
		}
		debugLog += "\n\n";

		debugLog = debugLog.Substring(Mathf.Max(debugLog.Length - 2048, 0));

		debugText.text = debugLog;
	}
}

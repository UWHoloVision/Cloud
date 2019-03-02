using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/*
 * Simple utility for debugging
 */
public class LoggerBehaviour : MonoBehaviour
{
    private Text UIText;
    private const int LOG_LIMIT = 20;
    private Queue<string> logQueue = new Queue<string>();

    private System.Random rand = new System.Random();

    // Start is called before the first frame update
    void Start()
    {
        this.UIText = GetComponent<Text>();
        Application.logMessageReceived += logMessageReceived;
    }

    private void logMessageReceived(string condition, string stackTrace, LogType type)
    {
        switch(type)
        {
            case LogType.Error:
            case LogType.Log:
            case LogType.Exception:
                logQueue.Enqueue(condition);
                break;
        }

        var displayText = "";
        foreach (var log in logQueue)
        {
            displayText += log + "\n";
        }
        UIText.text = displayText;

        if (logQueue.Count >= LOG_LIMIT)
        {
            logQueue.Dequeue();
        }
    }

    // Update is called once per frame
    void Update()
    {
    }
}

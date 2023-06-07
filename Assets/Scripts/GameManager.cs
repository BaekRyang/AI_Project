using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour
{

    [Serializable]
    public enum ButtonActions
    {
        InstantiateChaser,
        InstantiateEvader
    }

    public void ButtonEvent(int buttonType)
    {
        ButtonActions buttonAction = (ButtonActions)buttonType;
        
        switch (buttonAction)
        {
            case ButtonActions.InstantiateChaser:
                Instantiate(Resources.Load("Prefabs/Chaser"), Vector3.zero, Quaternion.identity);
                break;
            case ButtonActions.InstantiateEvader:
                Instantiate(Resources.Load("Prefabs/Evader"), Vector3.zero, Quaternion.identity);
                break;
        }
    }
}

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Serialization;

public class Creature : MonoBehaviour
{
    public bool  clicked;
    public float clickedTime;
    public int   doubleClickCount;
    public float doubleClickTime;

    public void OnMouseDown()
    {
        clicked     = true;
        clickedTime = Time.time;
    }

    public void OnMouseUp()
    {
        clicked = false;
        if (doubleClickCount == 1)
        {
            if (doubleClickTime + 0.2f > Time.time)
            {
                GameManager.Instance.zoomInCamera.Follow = null;
                GameManager.Instance.zoomInCamera.gameObject.SetActive(false);
                doubleClickCount = 0;
                return;
            }
            doubleClickCount = 0;
        }
        
        if (clickedTime + 0.2f > Time.time && doubleClickCount == 0)
        {
            GameManager.Instance.zoomInCamera.Follow = transform;
            GameManager.Instance.zoomInCamera.gameObject.SetActive(true);
            doubleClickCount++;
            doubleClickTime = Time.time;
        }

        
    }

    public void OnMouseDrag()
    {
        if (clicked)
        {
            Vector3 mousePosition = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            var relativePos = mousePosition - transform.position;
            transform.Translate(new Vector3(relativePos.x, relativePos.y, 0));
        }
    }
}
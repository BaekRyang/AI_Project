using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;

public class Creature : MonoBehaviour
{
    public bool clicked;
    
    public void OnMouseDown()
    {
        clicked = true;
    }

    public void OnMouseUp()
    {
        clicked = false;
    }

    public void OnMouseDrag()
    {
        if (clicked)
        {
            var position = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            transform.position = new Vector2(position.x, position.y);
        }
    }
}

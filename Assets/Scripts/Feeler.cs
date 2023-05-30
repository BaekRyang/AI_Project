using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class Feeler
{
    public Vector2 forwarding;
    public float   detectLength;
    
    public Feeler(Vector2 forwarding, float detectLength)
    {
        this.forwarding   = forwarding;
        this.detectLength = detectLength;
    }

    public RaycastHit2D Cast(Vector2 position, LayerMask layerMask)
    {
        Debug.DrawLine(position, forwarding, Color.red);
        var direction = (forwarding - position).normalized;
        var hit       = Physics2D.Raycast(position, direction, detectLength, layerMask);
        return hit;
    }
}

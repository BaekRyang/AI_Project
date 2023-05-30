using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Creature : MonoBehaviour
{
    public float maxSpeed = 1.0f;

    public Rigidbody2D rigidbody;

    private void Awake()
    {
        rigidbody = GetComponent<Rigidbody2D>();
    }

    protected Vector2 Calculate()
    {
        return Vector2.one;
    }
}

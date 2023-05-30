using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Evader : Creature
{
    public Movements _movements;

    private void Start()
    {
        _movements = new Movements(this);
    }

    private void Update()
    {
        rigidbody.velocity = _movements.Wander();
    }
}

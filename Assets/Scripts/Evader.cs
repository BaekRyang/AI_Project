using System;
using System.Collections;
using System.Collections.Generic;
using Movements;
using UnityEngine;
using UnityEngine.EventSystems;

public class Evader : MovingCreature
{
    private void Start()
    {
        movements = new Movements.Movements(this, AgentType.Evader); //위치를 참조해야 하므로 this를 넘겨준다.
    }
}

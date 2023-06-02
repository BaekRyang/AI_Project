using System.Collections;
using System.Collections.Generic;
using Movements;
using UnityEngine;

public class Chaser : MovingCreature
{
    private void Start()
    {
        movements = new Movements.Movements(this, AgentType.Chaser); //위치를 참조해야 하므로 this를 넘겨준다.
    }
}

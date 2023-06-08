using System.Collections;
using System.Collections.Generic;
using Movements;
using Unity.VisualScripting;
using UnityEngine;

public class Chaser : MovingCreature
{
    private void Start()
    {
        movements = new Movements.Movements(this, AgentType.Chaser); //위치를 참조해야 하므로 this를 넘겨준다.
        StartCoroutine(movements.CheckAgent());

    }
    
    private void OnCollisionEnter2D (Collision2D collision)
    {
        if (collision.gameObject.layer == LayerMask.NameToLayer("Evader"))
        {
            Destroy(collision.gameObject);
        }
    }
}

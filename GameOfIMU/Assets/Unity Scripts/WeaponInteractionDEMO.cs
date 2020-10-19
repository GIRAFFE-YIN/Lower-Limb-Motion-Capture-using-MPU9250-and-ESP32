using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * This class is attached to the Flare Gun prefab and is responsible to apply
 * the received changes to it.
 * Only the trigger is implemented. (SetTrigger from the parent class will be executed)
 */

public class WeaponInteractionDEMO : WeaponInteraction
{
    // Start is called before the first frame update and will set specific transformation parameters 
    void Start()
    {
        triggerMinRot = 130;
        triggerMaxRot = 80;
        triggerTransform = transform.Find("Plane.003").transform;
    }
}

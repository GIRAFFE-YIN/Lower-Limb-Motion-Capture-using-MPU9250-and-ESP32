using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * This class is attached to the SCAR prefab and is responsible to apply
 * the received changes to it.
 * Implemented features:
 * The trigger (SetTrigger from the parent class will be executed)
 * The selector switch 
 * The movable pieces/ cocking handle
 * The magazine (Check if a magazine is present)
 */

public class WeaponInteractionSCAR :WeaponInteraction
{ 
    private Transform movablePiecesTransform;
    private Transform selectorSwitchTransform;
    private GameObject magazineObject;
    private readonly float movablePiecesMaxXpos = 0.117f;
    private double shootingSpeed = 60f / 650; // The Scar has a shooting speed of 625 rounds per minute 
    private DateTime timeLastShot; // Control for AUTO mode: Initiate the last shot to when the weapon is created
    private bool semiShotEnable; // Control for SEMI mode: In semi mode a shot can only be fired after the trigger has been released between shots

    // Start is called before the first frame update
    void Start()
    {
        triggerMinRot = -15;
        triggerMaxRot = 15;
        shotThreshold = 130; // Every weapon has another threshold. For now this is only relevant for the SCAR because it is the only weapon that can have magazines.
        timeLastShot = DateTime.Now;
        triggerTransform = transform.Find("Trigger").transform;
        movablePiecesTransform = transform.Find("Movable Pieces").transform;
        selectorSwitchTransform = transform.Find("Selector Switch").transform;
        magazineObject = transform.Find("Magazine").gameObject;
        selectorSwitchPosition = SwitchPosition.SAFE;
    }

    // Update is called once per frame
    void Update()
    {
        // Selector Switch
        SetSelectorSwitch(selectorSwitchPosition);

        // Movable pieces
        SetMovablePieces(movablePiecesCurrentValue);

        // Trigger
        SetTrigger(triggerCurrentValue);

        // Magazine
        SetMagazine(magazinePresent);

        // Check if a shot can be fired
        CheckShotFired();

        // Gives the bullets left in the magazine
        SetAmountOfBulletsInMagazine(magazineID);
    }

    // Set the position of the selector switch with the value received from the UDP packet
    private void SetSelectorSwitch(SwitchPosition switchPosition)
    {
        switch (switchPosition)
        {
            case SwitchPosition.SAFE:
                selectorSwitchTransform.localEulerAngles = new Vector3(0, 0, 30);
                break;

            case SwitchPosition.SEMI:
                selectorSwitchTransform.localEulerAngles = new Vector3(0, 0, 0);
                break;

            case SwitchPosition.AUTO:
                selectorSwitchTransform.localEulerAngles = new Vector3(0, 0, -30);
                break;
        }
    }

    // Set the position of the movable pieces with the value received from the UDP packet
    private void SetMovablePieces(int movablePiecesValue)
    {
        movablePiecesCurrentValue = Mathf.Clamp(movablePiecesValue, 0, 255);
        float piecesXPos = movablePiecesMaxXpos * ((float)movablePiecesCurrentValue / 255);
        movablePiecesTransform.localPosition = new Vector3(piecesXPos, 0, 0);

        if (movablePiecesValue == 255)// && !weaponLoaded)
        {
            almostLoaded = true; // has to be tested
            //SetWeaponLoaded();
        }

        // Has to bet tested
        if (almostLoaded)
        {
            // If the magazine with bullets gets removed before he cocking ahndle is in its forward position, a bullet will not be loaded in the chamber
            if(magazinePresent && amountOfBulletsInMagazine > 0)
            {
                if(movablePiecesValue == 0)
                {
                    SetWeaponLoaded(); // If there was already a bullet in the chamber, that bullet will be thrown out and the chamber will be loaded with a new one
                    almostLoaded = false;
                }
            }
            else
            {
                almostLoaded = false;
                weaponLoaded = false;
            }
        }
    }

    //if no magazine is present, disable the magazine object (-> disable redering)
    private void SetMagazine(bool present)
    {
        magazineObject.SetActive(present);
        magazineManager.AddMagazine(magazineID);
    }

    private void CheckShotFired()
    {
        // if the weapon is out of safe mode with a bullet in the chamber while the trigger is pulled further than the threshold 
        // and the cocking handle is in its forward position, than a shot can be fired.
        if(triggerCurrentValue > shotThreshold && movablePiecesCurrentValue==0)
        {
            // Every time the trigger is pulled and the weapon is loaded, a bullet will be fired. (loaded bullet in the chamber != magazine present)
            // Later an animation can be added to this part. This way evry time the gun fires, it will be visible in the GUI or in VR
            if (weaponLoaded)
            {
                // In SEMI the next shot can only be effectuated if teh trigger has been released between shots
                if (selectorSwitchPosition == SwitchPosition.SEMI && semiShotEnable)
                {
                    SetWeaponLoaded(); // Modern weapons get loaded automatically if a bullet is fired and a magazine with bullets is present
                    semiShotEnable = false;
                    // add shooting animation here
                }
                // In AUTO the next shot can only be effectuated if there has been enough time between the shots (can't go over the maximum shooting speed of the weapon)
                else if (selectorSwitchPosition == SwitchPosition.AUTO && (DateTime.Now - timeLastShot).TotalSeconds > shootingSpeed)
                {
                    SetWeaponLoaded(); // Modern weapons get loaded automatically if a bullet is fired and a magazine with bullets is present
                    timeLastShot = DateTime.Now;
                    // add shooting animation here
                }
            }
        }
        else
        {
            semiShotEnable = true; // If the trigger is released a new shot can be effectuated in SEMI mode
        }
    }

}

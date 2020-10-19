using UnityEngine;

/*
 * This class is attached to the FiveSeven prefab and is responsible to apply
 * the received changes to it.
 * Only the trigger is implemented. The method for the trigger in this class
 * defers from its parent class by the axis over which the rotation must occur
 */

public class WeaponInteraction57 : WeaponInteraction
{
    private Transform movablePieceTransform;
    private Transform selectorSwitchLTransform;
    private Transform selectorSwitchRTransform;
    private float movablePieceMinPos;
    private float movablePieceMaxPos;
    private float selectorSwitchMinRot;
    private float selectorSwitchMaxRot;
    // Start is called before the first frame update
    void Start()
    {
        triggerMinRot = 13;
        triggerMaxRot = -15;
        movablePieceMinPos = 0.228999f;
        movablePieceMaxPos = 0.27f;
        selectorSwitchMinRot = 15;
        selectorSwitchMaxRot = 0;
        triggerTransform = transform.Find("Trigger").transform;
        movablePieceTransform = transform.Find("Slide").transform;
        selectorSwitchLTransform = transform.Find("SelectorSwitchL").transform;
        selectorSwitchRTransform = transform.Find("SelectorSwitchR").transform;
    }

    // Update is called once per frame
    void Update()
    {
        //Selector Switch
        SetSelectorSwitch(selectorSwitchPosition);

        //Movable pieces
        SetMovablePieces(movablePiecesCurrentValue);

        //Trigger
        SetTrigger(triggerCurrentValue);
    }


    // Set the position of the trigger by rotating over the X-axis (for other objects the rotation happens over the Z-axis)
    protected override void SetTrigger(int triggerValue)
    {
        triggerCurrentValue = Mathf.Clamp(triggerValue, 0, 255);
        float triggerXRot = triggerMinRot + (triggerMaxRot - triggerMinRot) * ((float)triggerValue / 255); // Maps the current value of the trigger given by the client (value between 0 and 255) between triggerMinRot and triggerMaxRot
        triggerTransform.localEulerAngles = new Vector3(triggerXRot, triggerTransform.localEulerAngles.y, triggerTransform.localEulerAngles.z);
    }

    private void SetSelectorSwitch(SwitchPosition switchPosition)
    {
        switch (switchPosition)
        {
            case SwitchPosition.SAFE:
                selectorSwitchLTransform.localEulerAngles = new Vector3(selectorSwitchMinRot, selectorSwitchLTransform.localEulerAngles.y, selectorSwitchLTransform.localEulerAngles.z);
                selectorSwitchRTransform.localEulerAngles = new Vector3(selectorSwitchMinRot, selectorSwitchRTransform.localEulerAngles.y, selectorSwitchRTransform.localEulerAngles.z);
                break;

            case SwitchPosition.SEMI:
                selectorSwitchLTransform.localEulerAngles = new Vector3(selectorSwitchMaxRot, selectorSwitchLTransform.localEulerAngles.y, selectorSwitchLTransform.localEulerAngles.z);
                selectorSwitchRTransform.localEulerAngles = new Vector3(selectorSwitchMaxRot, selectorSwitchRTransform.localEulerAngles.y, selectorSwitchRTransform.localEulerAngles.z);
                break;
            case SwitchPosition.AUTO: // Same as SEMI, because there is no AUTO mode on a Five Seven
                selectorSwitchLTransform.localEulerAngles = new Vector3(selectorSwitchMaxRot, selectorSwitchLTransform.localEulerAngles.y, selectorSwitchLTransform.localEulerAngles.z);
                selectorSwitchRTransform.localEulerAngles = new Vector3(selectorSwitchMaxRot, selectorSwitchRTransform.localEulerAngles.y, selectorSwitchRTransform.localEulerAngles.z);
                break;
        }
    }

    // Set the position of the movable pieces with the value received from the UDP packet
    private void SetMovablePieces(int movablePiecesValue)
    {
        movablePiecesCurrentValue = Mathf.Clamp(movablePiecesValue, 0, 255);
        float movablePieceXPos = movablePieceMinPos + (movablePieceMaxPos - movablePieceMinPos) * ((float)movablePiecesValue / 255);
        movablePieceTransform.localPosition = new Vector3(movablePieceXPos, movablePieceTransform.localPosition.y, movablePieceTransform.localPosition.z);
    }
}

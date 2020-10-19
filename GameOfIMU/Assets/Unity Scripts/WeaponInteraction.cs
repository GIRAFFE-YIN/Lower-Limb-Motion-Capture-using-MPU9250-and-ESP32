using System;
using System.Collections;
using System.Collections.Generic;
using System.Net;
using UnityEngine;

/*
 * This class is the parent class that is responsible to decode and apply 
 * the received changes on the weapon to which it is linked to.
 * It is the child classes that are linked to the weapon prefabs.
 * The common methods of the child classes are implemnted in this class.
 * Every child class has a specific start method to set transformation parameters.
 * Some child classes have extra methods implemented to apply changes to different
 * features of the weapon in question.
 * Other child classes override common methods because a variation on the method
 * must be executed.
 */

public abstract class WeaponInteraction : MonoBehaviour
{
    public IPAddress ipAddress;
    public WeaponType weaponType;
    public SwitchPosition selectorSwitchPosition;
    public int triggerCurrentValue;
    public int movablePiecesCurrentValue;
    public bool magazinePresent;
    public int magazineID;
    public int amountOfBulletsInMagazine;
    public bool weaponLoaded; // If the criteria are met to load the weapon and the weapon gets loaded, this variable is set.

    protected Transform triggerTransform;
    protected float triggerMinRot;
    protected float triggerMaxRot;
    protected int shotThreshold; // Every weapon has another threshold. For now this is only relevant for the SCAR because it is the only weapon that can have magazines.
    protected static MagazineManager magazineManager = new MagazineManager(); // implemented in the parent class in case if other weapons get updated to have the magazine feature
    protected bool almostLoaded; // Used to load the weapon with the cocking handle

    private WriteTextIO textIO;
    private DateTime lastUpdate; // Used to check if a weapon has been active
    private static int weaponTypesBits;

    // Update is called once per frame. By default only the trigger gets
    void Update()
    {
        SetTrigger(triggerCurrentValue);
    }

    // Set the position of the trigger by rotating over the Z-axis
    protected virtual void SetTrigger(int triggerValue)
    {
        triggerCurrentValue = Mathf.Clamp(triggerValue, 0, 255);
        float triggerZRot = triggerMinRot + (triggerMaxRot - triggerMinRot) * ((float)triggerValue / 255); // Maps the current value of the trigger given by the client (value between 0 and 255) between triggerMinRot and triggerMaxRot
        triggerTransform.localEulerAngles = new Vector3(triggerTransform.localEulerAngles.x, triggerTransform.localEulerAngles.y, triggerZRot);
    }

    // Translate the data and edit the parameters of the virtual weapon
    public void PacketTranslater(byte[] packet) // if reading problems occur, this can be due to the endian (the sequence in which the bits are send, LSB first or last?)
    {
        // Check if the data is still send by the same type of weapon. It should be impossible for a weapon to change type (this is just in case)
        // If the client keeps sending packets with the wrong weapontype tag, it will be considered as inactive and the weapon will get deleted after a certain time
        if (GetWeaponTypeOutOfData(packet) == weaponType)
        {
            // Create storage containers
            byte[] triggerValue = new byte[4];
            byte[] movablePiecesValue = new byte[4];
            byte[] magazineIDValue = new byte[4];
            byte[] flags = new byte[4];


            // Store data in containers
            Array.Copy(packet, 0, triggerValue, 0, 1); //Copy(Array sourceArray, long sourceIndex, Array destinationArray, long destinationIndex, long length)
            Array.Copy(packet, 1, movablePiecesValue, 0, 1);
            Array.Copy(packet, 2, magazineIDValue, 0, 1);
            Array.Copy(packet, 3, flags, 0, 1);

            // Set current values
            triggerCurrentValue = BitConverter.ToInt32(triggerValue, 0);
            movablePiecesCurrentValue = BitConverter.ToInt32(movablePiecesValue, 0);// BitConverter start reading at a certain index till the end. 
            magazineID = BitConverter.ToInt32(magazineIDValue, 0);
            byte flagByte = flags[0];

            magazinePresent = !(flagByte % 2 == 0); // Last bit in the flag byte shows the mag status. So, if there is no mag present the value of the flag is even.

            byte switchPositionByte = (byte)(flagByte * Math.Pow(2, weaponTypesBits)); // bit shift left the enough of times to get rid of the used MSB's to encode the weapontype
            switchPositionByte /= (byte)Math.Pow(2, weaponTypesBits + 1); // BSR enough times to set the bits needed to recognize the selector switches position as LSB's
            selectorSwitchPosition = (SwitchPosition)switchPositionByte;

            // Create list data to be written in a textfile
            ArrayList dataArrayList = new ArrayList
            {
            triggerCurrentValue,
            selectorSwitchPosition,
            movablePiecesCurrentValue,
            magazinePresent,
            magazineID
            };

            textIO.WriteTextFile(dataArrayList);
            lastUpdate = DateTime.Now;// Use of this has to be reconsidered => it takes too much time
        }
    }

    public static WeaponType GetWeaponTypeOutOfData(byte[] packet)
    {
        byte[] flags = new byte[4];
        Array.Copy(packet, 3, flags, 0, 1);
        byte flagByte = flags[0];
        int numberOfWeaponTypes = Enum.GetNames(typeof(WeaponType)).Length; // Number of enum elements
        weaponTypesBits = (int)Math.Ceiling(Math.Log(numberOfWeaponTypes, 2)); //log2 of the different number of weapons. Log2 is always rounded up (=Ceiling)
        byte weaponTypeByte = (byte)(flagByte / Math.Pow(2, 8 - weaponTypesBits)); // bit shift right (the amount of bits that can be discarded = 3 + amount of bits that are not used)

        return (WeaponType)weaponTypeByte;
    }

    public DateTime GetLastUpdate()
    {
        return lastUpdate;
    }

    public void SetWeaponType(WeaponType weaponType)
    {
        this.weaponType = weaponType;
    }

    // Setter for ipAdress
    public void SetIPAddress(IPAddress ipAddress)
    {
        this.ipAddress = ipAddress;
    }

    public void SetTextIO(WeaponType weaponType, IPAddress IPAddress)
    {
        textIO = new WriteTextIO(weaponType.ToString(), ipAddress.ToString());
    }

    public static MagazineManager GetMagazineManager()
    {
        return magazineManager;
    }

    public void SetAmountOfBulletsInMagazine(int magazineID)
    {
        amountOfBulletsInMagazine = magazineManager.GetBulletsLeft(magazineID);
    }

    protected void SetWeaponLoaded()
    {
        if (magazinePresent && amountOfBulletsInMagazine > 0)
        {
            weaponLoaded = true; // If a magazine is present with bullets left and the cocking handle has been pulled completely and released completely, the weapon can be loaded
            magazineManager.DecrementBulletCount(magazineID); // When a weapon gets loaded, a bullet is loaded into the chamber and thus removed from the magazine
        }
        else
        {
            weaponLoaded = false;
        }

    }


}

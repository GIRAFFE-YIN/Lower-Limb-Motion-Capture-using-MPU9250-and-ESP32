using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MagazineManager
{
    private int defaultAmountOfBullets;
    private Dictionary<int, int> magazines;

    public MagazineManager()
    {
        defaultAmountOfBullets = 30;
        magazines = new Dictionary<int, int>();
    }

    public void AddMagazine(int magazineID)
    {
        // If magazine is not already registered it will be registered and will be fully loaded
        if (!magazines.ContainsKey(magazineID) && magazineID != 0) //magazineID = 0 is teh default ID if no magazine is present
        {
            magazines.Add(magazineID, defaultAmountOfBullets);
        }
    }

    public void DecrementBulletCount(int magazineID)
    {
        if(magazines.TryGetValue(magazineID, out int amountOfBulletsLeft))
        {
            if(amountOfBulletsLeft >= 0)
            {
                magazines[magazineID] = --amountOfBulletsLeft;
            }
        }
    }

    public int GetBulletsLeft(int magazineID)
    {
        int bulletsLeft = 0;
        if (magazines.TryGetValue(magazineID, out int amountOfBulletsLeft))
        {
            bulletsLeft = amountOfBulletsLeft;
        }
        return bulletsLeft;
    }

    public void ChargeAllMags()
    {
        List<int> keys = new List<int>(magazines.Keys);
        foreach (int key in keys)
        {
            magazines[key] = defaultAmountOfBullets;
        }
    }
}

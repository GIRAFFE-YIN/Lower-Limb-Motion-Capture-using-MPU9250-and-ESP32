using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

/*
 * This class creates a text file and writes the data received by the UDP clients in it.
*/
public class WriteTextIO
{
    public int counter;
    public string fileName;

    public WriteTextIO(string nameOfFile, string ipAddress)
    {
        counter = 0;
        CreateFile(nameOfFile, ipAddress);
    }

    // Creates a textfile and writes the title in it
    public void CreateFile(string name, string ipAddress)
    {
        try
        {
            string path = Directory.GetCurrentDirectory();
            fileName = path + "-" + name + ".txt";
            while (File.Exists(fileName))
            {
                fileName = path + "-" + name + ++counter + ".txt";
            }
            File.WriteAllText(fileName, "Data received from UDP client: " + ipAddress + "\n");
            string header = "\t Time \t\t      Trigger \t\t Selector switch \t Cocking handle \t Magazine present \t    Magazine ID \n\n"; //spacing was determined with trial and error
            File.AppendAllText(fileName, header);
        }
        catch (Exception err)
        {
            Console.WriteLine(err);
        }

    }

    // Writes to the textfile
    public void WriteTextFile(ArrayList dataArrayList)
    {
        // Both time and the content of the UDP packet is written to the file   
        string time = DateTime.Now.ToString("hh:mm:ss:ffff");
        string text = time;
        foreach (var data in dataArrayList)
        {
            text += "\t\t\t" + data;
        }
        text += "\n";

        File.AppendAllText(fileName, text);
    }
}

public interface IWriteTextIO
{
    void CreateFile();
    void WriteTextFile(string lastClientIP, string dataString);
}
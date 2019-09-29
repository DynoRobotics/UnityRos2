// C# example.
using UnityEditor;
using System.Diagnostics;

public class ScriptBatch
{
    [MenuItem("ROS/Windows Build")]
    public static void BuildGameWindows()
    {
        // Get filename.
        string path = EditorUtility.SaveFolderPanel("Choose Location of Built Game", "", "");
        string[] levels = new string[] { "Assets/Scenes/RosExample.unity"};

        // Build player.
        BuildPipeline.BuildPlayer(levels, path + "/RosApplication.exe", BuildTarget.StandaloneWindows64, BuildOptions.None);

        // Copy a file from the project folder to the build folder, alongside the built game.
        FileUtil.CopyFileOrDirectory("Assets/Resources/start_player.py", path + "/start_player.py");
        FileUtil.CopyFileOrDirectory("Assets/Resources/start_player.bat", path + "/start_player.bat");

    }

    [MenuItem("ROS/Linux Build")]
    public static void BuildGameLinux()
    {
        // Get filename.
        // FIXME(sam): creates folder for some reason... Allow setting name of game?
        string path = EditorUtility.SaveFolderPanel("Choose Location of Built ROS Application", "", "");
        // string path = System.IO.Path.GetDirectoryName(file_path);
        string[] levels = new string[] { "Assets/Scenes/RosExample.unity"};

        // Build player.
        BuildPipeline.BuildPlayer(levels, path + "/RosApplication", BuildTarget.StandaloneLinux64, BuildOptions.None);

        // Copy a file from the project folder to the build folder, alongside the built game.
        FileUtil.CopyFileOrDirectory("Assets/Resources/start_player.py", path + "/start_player.py");
        FileUtil.CopyFileOrDirectory("Assets/Resources/start_player.bash", path + "/start_player.bash");

    }
}

// C# example.
using UnityEditor;
using System.Diagnostics;

public class ScriptBatch
{
    [MenuItem("ROS/Windows Build")]
    public static void BuildGame()
    {
        // Get filename.
        string path = EditorUtility.SaveFolderPanel("Choose Location of Built Game", "", "");
        string[] levels = new string[] { "Assets/Scenes/RosExample.unity"};

        // Build player.
        BuildPipeline.BuildPlayer(levels, path + "/BuiltGame.exe", BuildTarget.StandaloneWindows64, BuildOptions.None);

        // Copy a file from the project folder to the build folder, alongside the built game.
        FileUtil.CopyFileOrDirectory("Assets/Resources/start_player.py", path + "/start_player.py");
        FileUtil.CopyFileOrDirectory("Assets/Resources/start_player.bat", path + "/start_player.bat");

    }
}


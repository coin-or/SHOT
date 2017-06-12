$files = @("..\src\SHOT.cpp","..\src\Tasks\TaskPrintIterationHeader.cpp", "..\src\Tasks\TaskPrintProblemStats.cpp", "..\src\Tasks\TaskPrintSolution.cpp")
foreach ($file in $files) 
{
    Copy-Item $file ([io.path]::ChangeExtension($file, '.bak')) 
    Get-Content $file -Encoding utf8 | Out-File temp.cpp -Encoding Oem
    Move-Item temp.cpp $file -force
}


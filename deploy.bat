:: 启动mkdocs虚拟环境（注意替换为本地的anaconda3目录、虚拟环境名为mkdocs）
%windir%\System32\WindowsPowerShell\v1.0\powershell.exe -ExecutionPolicy ByPass -NoExit -Command "& 'D:\software\anaconda3\shell\condabin\conda-hook.ps1' ; conda activate 'D:\software\anaconda3' "; conda activate mkdocs; mkdocs serve;

echo hello!
pause
:: 编译文档
mkdocs build
:: 部署到github的页面上
mkdocs gh-deploy
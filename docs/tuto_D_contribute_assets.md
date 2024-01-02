# 如何升级内容

我们的内容驻留在单独的 [Git LFS 仓库][contentrepolink] 中。作为构建系统的一部分，我们生成并上传一个包，其中包含带有当前日期和提交标记的该内容的最新版本。我们定期升级 [CARLA 仓库][carlarepolink]，并提供最新版本内容包的链接。本文档包含将此链接更新到最新版本所需的手动步骤。

1. **复制您要链接的内容包的标签。**<br>
   T可以通过查看最新 [Jenkins 构建][jenkinslink]的
工件部分中生成的包名称来找到此标签，例如
   `20190617_086f97f.tar.gz`。

2. **将标记粘贴到 ContentVersions.txt 中。**<br>
   通过将标记粘贴到文件末尾来
编辑 [Edit ContentVersions.txt][cvlink] ，
   例如 `Latest: 20190617_086f97f` (不包括 `.tar.gz` 部分）。

3. **打开拉取请求。**<br>
   提交更改并打开新的拉取请求。

[contentrepolink]: https://bitbucket.org/carla-simulator/carla-content
[carlarepolink]: https://github.com/carla-simulator/carla
[jenkinslink]: http://35.181.165.160:8080/blue/organizations/jenkins/carla-content/activity
[cvlink]: https://github.com/carla-simulator/carla/edit/master/Util/ContentVersions.txt
